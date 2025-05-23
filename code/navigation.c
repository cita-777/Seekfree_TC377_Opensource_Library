/**
 * *****************************************************************************
 * @file        navigation.c
 * @brief       导航系统实现，简化版本，主要用于智能车科目导航
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-04-04
 * @copyright   cita
 * *****************************************************************************
 */

#include "navigation.h"
#include "zf_common_headfile.h"

/*----------------------------------宏定义-----------------------------------*/
// 导航模式定义
#define NAV_MODE_GPS_ONLY 0   // 仅使用GPS模式
#define NAV_MODE_IMU_ONLY 1   // 仅使用IMU模式
#define NAV_MODE_MANUAL 2     // 手动模式

// 地球半径（米）
#define EARTH_RADIUS 6378137.0
// 弧度角度转换
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886

// 坐标系参数
#define MAX_WAYPOINTS 4       // 最大路径点数量
#define WAYPOINT_RADIUS 2.0   // 航点切换半径(米)

// 科目状态定义
#define COURSE_STATE_IDLE 0        // 未运行
#define COURSE_STATE_RUNNING 1     // 正在运行
#define COURSE_STATE_COMPLETED 2   // 已完成

// 科目一阶段定义
#define COURSE1_STAGE_START 0      // 起点到掉头起点
#define COURSE1_STAGE_TURN 1       // 掉头区
#define COURSE1_STAGE_RETURN 2     // 掉头结束点到终点
#define COURSE1_STAGE_COMPLETE 3   // 完成

// 导航参数
#define TURN_ANGLE_THRESHOLD 165.0   // 掉头完成的角度阈值(度)
#define STEER_GAIN 1.0               // 转向增益
#define MAX_STEER 35.0               // 最大转向角度
#define NORMAL_SPEED 100             // 正常速度
#define TURNING_SPEED 60             // 转弯速度
#define SLOW_SPEED 40                // 慢速

/*----------------------------------类型定义----------------------------------*/
// 导航系统结构体
typedef struct
{
    uint8_t mode;                          // 导航模式
    uint8_t current_waypoint;              // 当前航点索引
    double  waypoints[MAX_WAYPOINTS][2];   // 航点数组 [纬度, 经度]

    // 当前位置和航向
    double current_lat;   // 当前纬度
    double current_lon;   // 当前经度
    float  current_yaw;   // 当前航向角(度)

    // 控制输出
    double  target_steer;   // 目标转向角
    int16_t target_speed;   // 目标速度

    // 科目相关参数
    uint8_t course_state;    // 科目状态
    uint8_t course1_stage;   // 科目一阶段
    float   initial_yaw;     // 初始航向角(用于掉头判断)
} NavSystem;

/*----------------------------------全局变量----------------------------------*/
static NavSystem nav_sys;   // 导航系统实例

/*----------------------------------私有函数----------------------------------*/
// 辅助导航函数
static double calculate_distance(double lat1, double lon1, double lat2, double lon2);
static double calculate_azimuth(double lat1, double lon1, double lat2, double lon2);
static double normalize_angle(double angle);

/*--------------------------------公共接口函数--------------------------------*/
/**
 * @brief 初始化导航系统
 */
void navigation_init(void)
{
    // 初始化系统参数
    nav_sys.mode             = NAV_MODE_GPS_ONLY;
    nav_sys.current_waypoint = 0;
    nav_sys.target_steer     = 0.0;
    nav_sys.target_speed     = 0;

    // 初始化科目相关参数
    nav_sys.course_state  = COURSE_STATE_IDLE;
    nav_sys.course1_stage = COURSE1_STAGE_START;
    nav_sys.initial_yaw   = 0.0f;

    // 初始化默认科目点位(可通过后续采集更新)
    // 默认航点 - 将在采集时更新
    nav_sys.waypoints[0][0] = 31.2304;    // 起点纬度
    nav_sys.waypoints[0][1] = 121.4737;   // 起点经度
    nav_sys.waypoints[1][0] = 31.2314;    // 掉头起点纬度
    nav_sys.waypoints[1][1] = 121.4747;   // 掉头起点经度
    nav_sys.waypoints[2][0] = 31.2312;    // 掉头结束点纬度
    nav_sys.waypoints[2][1] = 121.4745;   // 掉头结束点经度
    nav_sys.waypoints[3][0] = 31.2304;    // 终点纬度
    nav_sys.waypoints[3][1] = 121.4737;   // 终点经度

    printf("Navigation system initialized.\r\n");
}

/**
 * @brief 简化导航更新函数
 * @param gps_lat 当前GPS纬度
 * @param gps_lon 当前GPS经度
 * @param gps_valid GPS数据是否有效
 * @param imu_yaw IMU航向角 (度)
 */
void navigation_update(double gps_lat, double gps_lon, uint8_t gps_valid, float imu_yaw)
{
    // 更新当前位置
    if (gps_valid)
    {
        nav_sys.current_lat = gps_lat;
        nav_sys.current_lon = gps_lon;
    }

    // 更新当前航向角(只使用IMU的yaw角)
    nav_sys.current_yaw = imu_yaw;

    // 标准化航向角到[0,360)
    while (nav_sys.current_yaw < 0) nav_sys.current_yaw += 360.0f;
    while (nav_sys.current_yaw >= 360.0f) nav_sys.current_yaw -= 360.0f;

    // 如果当前没有运行科目，直接返回
    if (nav_sys.course_state != COURSE_STATE_RUNNING)
    {
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        return;
    }

    // 科目一导航逻辑
    double target_lat = 0.0, target_lon = 0.0;
    double dist_to_target = 0.0;

    // 根据当前阶段选择目标点
    switch (nav_sys.course1_stage)
    {
    case COURSE1_STAGE_START:
        // 目标是掉头起点
        target_lat = nav_sys.waypoints[1][0];
        target_lon = nav_sys.waypoints[1][1];

        dist_to_target = calculate_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // 如果到达掉头起点
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course1_stage = COURSE1_STAGE_TURN;
            nav_sys.initial_yaw   = nav_sys.current_yaw;   // 记录进入掉头区的初始航向
            printf("到达掉头起点，开始掉头\r\n");
        }
        break;

    case COURSE1_STAGE_TURN:
        // 目标是掉头结束点
        target_lat = nav_sys.waypoints[2][0];
        target_lon = nav_sys.waypoints[2][1];

        // 计算航向变化
        float yaw_change = normalize_angle(nav_sys.current_yaw - nav_sys.initial_yaw);
        if (yaw_change < 0) yaw_change += 360.0;

        // 如果完成掉头(航向变化超过阈值)
        if (yaw_change > TURN_ANGLE_THRESHOLD)
        {
            nav_sys.course1_stage = COURSE1_STAGE_RETURN;
            printf("掉头完成，开始返回终点\r\n");
        }
        break;

    case COURSE1_STAGE_RETURN:
        // 目标是终点
        target_lat = nav_sys.waypoints[3][0];
        target_lon = nav_sys.waypoints[3][1];

        dist_to_target = calculate_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // 如果到达终点
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course1_stage = COURSE1_STAGE_COMPLETE;
            nav_sys.course_state  = COURSE_STATE_COMPLETED;

            // 停止车辆
            nav_sys.target_steer = 0;
            nav_sys.target_speed = 0;

            printf("到达终点，科目一完成\r\n");
            return;
        }
        break;

    case COURSE1_STAGE_COMPLETE:
        // 已完成，保持停止状态
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        return;
    }

    // 计算到目标点的方位角
    double target_azimuth = calculate_azimuth(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

    // 计算航向误差
    double heading_error = normalize_angle(target_azimuth - nav_sys.current_yaw);

    // 计算距离(仅在非掉头阶段使用)
    if (nav_sys.course1_stage != COURSE1_STAGE_TURN)
    {
        dist_to_target = calculate_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);
    }

    // 计算转向控制量
    nav_sys.target_steer = heading_error * STEER_GAIN;

    // 限制转向量
    if (nav_sys.target_steer > MAX_STEER) nav_sys.target_steer = MAX_STEER;
    if (nav_sys.target_steer < -MAX_STEER) nav_sys.target_steer = -MAX_STEER;

    // 设置速度
    if (nav_sys.course1_stage == COURSE1_STAGE_TURN)
    {
        // 掉头阶段使用转弯速度
        nav_sys.target_speed = TURNING_SPEED;
    }
    else if (fabs(heading_error) > 45.0)
    {
        // 大角度转向，降低速度
        nav_sys.target_speed = TURNING_SPEED;
    }
    else if (dist_to_target < 5.0)
    {
        // 接近目标点，降低速度
        nav_sys.target_speed = SLOW_SPEED;
    }
    else
    {
        // 正常速度
        nav_sys.target_speed = NORMAL_SPEED;
    }

    // 输出控制命令
    if (nav_sys.mode != NAV_MODE_MANUAL)
    {
        servo_set(SERVO_MOTOR_MID + (int16_t)nav_sys.target_steer);
        drv8701_motor_set(nav_sys.target_speed);
    }
}

/**
 * @brief 切换导航模式
 * @param mode 目标模式
 */
void navigation_set_mode(uint8_t mode)
{
    if (mode <= NAV_MODE_MANUAL)
    {
        nav_sys.mode = mode;
        printf("导航模式切换为: %d\r\n", mode);
    }
}

/**
 * @brief 获取当前导航状态
 * @param lat 纬度指针
 * @param lon 经度指针
 * @param yaw 航向角指针
 */
void navigation_get_status(double* lat, double* lon, float* yaw)
{
    if (lat) *lat = nav_sys.current_lat;
    if (lon) *lon = nav_sys.current_lon;
    if (yaw) *yaw = nav_sys.current_yaw;
}

/**
 * @brief 开始运行科目一
 */
void navigation_start_course1(void)
{
    nav_sys.course_state     = COURSE_STATE_RUNNING;
    nav_sys.course1_stage    = COURSE1_STAGE_START;
    nav_sys.mode             = NAV_MODE_GPS_ONLY;
    nav_sys.current_waypoint = 0;

    printf("科目一开始运行\r\n");
}

/**
 * @brief 停止当前科目
 */
void navigation_stop_course(void)
{
    nav_sys.course_state = COURSE_STATE_IDLE;

    // 停止车辆
    nav_sys.target_steer = 0;
    nav_sys.target_speed = 0;
    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_set(0);

    printf("科目停止运行\r\n");
}

/**
 * @brief 获取当前科目状态
 * @return 当前科目状态
 */
uint8_t navigation_get_course_state(void)
{
    return nav_sys.course_state;
}

/**
 * @brief 采集GPS点位
 * @param point_index 点位索引 (0-起点, 1-掉头起点, 2-掉头结束点, 3-终点)
 * @param lat 纬度
 * @param lon 经度
 */
void navigation_collect_point(uint8_t point_index, double lat, double lon)
{
    if (point_index < MAX_WAYPOINTS)
    {
        nav_sys.waypoints[point_index][0] = lat;
        nav_sys.waypoints[point_index][1] = lon;

        printf("采集点位[%d]: 纬度 %.6f, 经度 %.6f\r\n", point_index, lat, lon);
    }
}

/**
 * @brief 发送导航数据到上位机
 */
void navigation_send_data(void)
{
    seekfree_assistant_oscilloscope_data.data[0]     = nav_sys.current_lat;
    seekfree_assistant_oscilloscope_data.data[1]     = nav_sys.current_lon;
    seekfree_assistant_oscilloscope_data.data[2]     = nav_sys.current_yaw;
    seekfree_assistant_oscilloscope_data.data[3]     = nav_sys.target_steer;
    seekfree_assistant_oscilloscope_data.data[4]     = nav_sys.target_speed;
    seekfree_assistant_oscilloscope_data.data[5]     = nav_sys.course1_stage;
    seekfree_assistant_oscilloscope_data.data[6]     = nav_sys.course_state;
    seekfree_assistant_oscilloscope_data.data[7]     = nav_sys.mode;
    seekfree_assistant_oscilloscope_data.channel_num = 8;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

/*--------------------------------私有函数实现--------------------------------*/
/**
 * @brief 计算两点间距离
 */
static double calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
    // 将经纬度转换为弧度
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lon1_rad = lon1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double lon2_rad = lon2 * DEG_TO_RAD;

    // 半正矢公式计算距离
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;
    double a    = sin(dlat / 2) * sin(dlat / 2) + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2) * sin(dlon / 2);
    double c    = 2 * atan2(sqrt(a), sqrt(1 - a));

    return EARTH_RADIUS * c;
}

/**
 * @brief 计算从点1到点2的方位角
 */
static double calculate_azimuth(double lat1, double lon1, double lat2, double lon2)
{
    // 将经纬度转换为弧度
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lon1_rad = lon1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double lon2_rad = lon2 * DEG_TO_RAD;

    // 计算方位角
    double y           = sin(lon2_rad - lon1_rad) * cos(lat2_rad);
    double x           = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(lon2_rad - lon1_rad);
    double azimuth_rad = atan2(y, x);

    // 转换为度数，并确保在[0,360)范围内
    double azimuth_deg = azimuth_rad * RAD_TO_DEG;
    return (azimuth_deg < 0) ? (azimuth_deg + 360.0) : azimuth_deg;
}

/**
 * @brief 标准化角度到[-180, 180]范围
 */
static double normalize_angle(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}