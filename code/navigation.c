/**
 * *****************************************************************************
 * @file        navigation.c
 * @brief       简化导航系统实现，使用GPS点位和IMU方向角
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-05-24
 * @copyright   cita
 * *****************************************************************************
 */

#include "navigation.h"
#include "zf_common_headfile.h"

/*----------------------------------宏定义-----------------------------------*/
// 地球半径（米）
#define EARTH_RADIUS 6378137.0
// 弧度角度转换
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886

// 坐标系参数
#define MAX_WAYPOINTS 20      // 最大路径点数量 (增加以支持科目二和科目三)
#define WAYPOINT_RADIUS 2.0   // 航点切换半径(米)

// 科目一阶段定义
#define COURSE1_STAGE_START 0      // 起点到掉头起点
#define COURSE1_STAGE_TURN 1       // 掉头区
#define COURSE1_STAGE_RETURN 2     // 掉头结束点到终点
#define COURSE1_STAGE_COMPLETE 3   // 完成

// 科目二阶段定义
#define COURSE2_STAGE_START 0      // 起点出发到第一个锥桶
#define COURSE2_STAGE_FORWARD 1    // 正向绕桩段
#define COURSE2_STAGE_TURN 2       // 掉头段
#define COURSE2_STAGE_BACKWARD 3   // 反向绕桩段
#define COURSE2_STAGE_END 4        // 返回终点
#define COURSE2_STAGE_COMPLETE 5   // 完成

// 科目三阶段定义（基于科目一结构，增加路面元素）
#define COURSE3_STAGE_START 0      // 起点到掉头起点
#define COURSE3_STAGE_TURN 1       // 掉头区
#define COURSE3_STAGE_RETURN 2     // 掉头结束点到终点
#define COURSE3_STAGE_COMPLETE 3   // 完成

// 路面元素定义
#define ROAD_NORMAL 0   // 正常路面
#define ROAD_SLOPE 1    // 坡道
#define ROAD_BUMPY 2    // 颠簸路段

// 导航参数
#define TURN_ANGLE_THRESHOLD 165.0   // 掉头完成的角度阈值(度)
#define STEER_GAIN 1.0               // 转向增益
#define MAX_STEER 20.0               // 最大转向角度

// 速度参数
#define NORMAL_SPEED 10   // 正常速度
#define TURNING_SPEED 6   // 转弯速度
#define SLOW_SPEED 40     // 慢速
#define SLOPE_SPEED 5     // 坡道速度
#define BUMPY_SPEED 3     // 颠簸路段速度
#define SLALOM_SPEED 7    // 绕桩速度

// 科目二参数
#define MAX_CONES 10           // 最大锥桶数量
#define CONE_SWITCH_DIST 1.5   // 锥桶切换距离(米)

/*----------------------------------类型定义----------------------------------*/
// 导航系统结构体
typedef struct
{
    uint8_t current_waypoint;              // 当前航点索引
    double  waypoints[MAX_WAYPOINTS][2];   // 航点数组 [纬度, 经度]
    uint8_t total_waypoints;               // 总航点数量

    // 当前位置和航向
    double current_lat;   // 当前纬度
    double current_lon;   // 当前经度
    float  current_yaw;   // 当前航向角(度)

    // 控制输出
    double  target_steer;   // 目标转向角
    int16_t target_speed;   // 目标速度

    // 科目相关参数
    uint8_t course_state;    // 科目状态
    uint8_t course_type;     // 科目类型 (1=科目一, 2=科目二, 3=科目三)
    uint8_t course1_stage;   // 科目一阶段
    uint8_t course2_stage;   // 科目二阶段
    uint8_t course3_stage;   // 科目三阶段
    uint8_t current_road;    // 当前路面类型(用于科目三)

    // 科目二特定参数
    double  cones[MAX_CONES][2];   // 锥桶位置 [纬度, 经度]
    uint8_t total_cones;           // 锥桶总数
    uint8_t current_cone;          // 当前锥桶索引
    uint8_t slalom_direction;      // 当前绕桩方向(0=左侧, 1=右侧)

    float initial_yaw;   // 初始航向角(用于掉头判断)
} NavSystem;

/*----------------------------------全局变量----------------------------------*/
static NavSystem nav_sys;   // 导航系统实例

/*----------------------------------私有函数----------------------------------*/
// 辅助导航函数
static double  normalize_angle(double angle);
static void    update_course1(void);
static void    update_course2(void);
static void    update_course3(void);
static uint8_t detect_road_element(double lat, double lon);

/*--------------------------------公共接口函数--------------------------------*/
/**
 * @brief 初始化导航系统
 */
void navigation_init(void)
{
    // 初始化系统参数
    nav_sys.current_waypoint = 0;
    nav_sys.target_steer     = 0.0;
    nav_sys.target_speed     = 0;
    nav_sys.total_waypoints  = 4;   // 科目一默认有4个航点

    // 初始化科目相关参数
    nav_sys.course_state  = COURSE_STATE_IDLE;
    nav_sys.course_type   = 0;
    nav_sys.course1_stage = COURSE1_STAGE_START;
    nav_sys.course2_stage = COURSE2_STAGE_START;
    nav_sys.course3_stage = COURSE3_STAGE_START;
    nav_sys.current_road  = ROAD_NORMAL;
    nav_sys.initial_yaw   = 0.0f;

    // 初始化科目二参数
    nav_sys.total_cones      = 5;   // 默认5个锥桶
    nav_sys.current_cone     = 0;
    nav_sys.slalom_direction = 0;   // 初始绕桩方向(左侧)

    // 初始化默认科目点位(可通过后续采集更新)
    // 科目一
    nav_sys.waypoints[0][0] = 31.2304;    // 起点纬度
    nav_sys.waypoints[0][1] = 121.4737;   // 起点经度
    nav_sys.waypoints[1][0] = 31.2314;    // 掉头起点纬度
    nav_sys.waypoints[1][1] = 121.4747;   // 掉头起点经度
    nav_sys.waypoints[2][0] = 31.2312;    // 掉头结束点纬度
    nav_sys.waypoints[2][1] = 121.4745;   // 掉头结束点经度
    nav_sys.waypoints[3][0] = 31.2304;    // 终点纬度
    nav_sys.waypoints[3][1] = 121.4737;   // 终点经度

    // 科目二初始锥桶位置（示例）
    nav_sys.cones[0][0] = 31.2304;    // 第1个锥桶纬度
    nav_sys.cones[0][1] = 121.4737;   // 第1个锥桶经度
    nav_sys.cones[1][0] = 31.2306;    // 第2个锥桶纬度
    nav_sys.cones[1][1] = 121.4739;   // 第2个锥桶经度
    nav_sys.cones[2][0] = 31.2308;    // 第3个锥桶纬度
    nav_sys.cones[2][1] = 121.4741;   // 第3个锥桶经度
    nav_sys.cones[3][0] = 31.2310;    // 第4个锥桶纬度
    nav_sys.cones[3][1] = 121.4743;   // 第4个锥桶经度
    nav_sys.cones[4][0] = 31.2312;    // 第5个锥桶纬度
    nav_sys.cones[4][1] = 121.4745;   // 第5个锥桶经度

    printf("导航系统已初始化.\r\n");
}

void navigation_proc(void)
{

    navigation_update(gnss.latitude, gnss.longitude, gnss.state, g_imu_angle.yaw);   // 默认调用，实际数据由外部更新
    // timer_enable();   // 显示当前导航状态
    // printf("Show yaw angle: %.2f\n", g_imu_angle.yaw);
    // timer_disable();
    disable_handler("navigation");   // 禁用导航处理函数，避免重复调用
}
/**
 * @brief 导航更新函数
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

    // 根据科目类型更新导航
    switch (nav_sys.course_type)
    {
    case 1:   // 科目一
        update_course1();
        break;

    case 2:   // 科目二
        update_course2();
        break;

    case 3:   // 科目三
        update_course3();
        break;

    default:
        // 未知科目类型，停止车辆
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        break;
    }

    // 输出控制命令
    servo_set(SERVO_MOTOR_MID + (int16_t)nav_sys.target_steer);
    drv8701_motor_set(nav_sys.target_speed);
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
    nav_sys.course_type      = 1;
    nav_sys.course1_stage    = COURSE1_STAGE_START;
    nav_sys.current_waypoint = 0;

    printf("科目一开始运行\r\n");
}

/**
 * @brief 开始运行科目二
 */
void navigation_start_course2(void)
{
    nav_sys.course_state     = COURSE_STATE_RUNNING;
    nav_sys.course_type      = 2;
    nav_sys.course2_stage    = COURSE2_STAGE_START;
    nav_sys.current_cone     = 0;
    nav_sys.slalom_direction = 0;   // 初始绕桩方向(左侧)

    printf("科目二开始运行\r\n");
}

/**
 * @brief 开始运行科目三
 */
void navigation_start_course3(void)
{
    nav_sys.course_state     = COURSE_STATE_RUNNING;
    nav_sys.course_type      = 3;
    nav_sys.course3_stage    = COURSE3_STAGE_START;
    nav_sys.current_waypoint = 0;
    nav_sys.current_road     = ROAD_NORMAL;

    printf("科目三开始运行\r\n");
}

/**
 * @brief 停止当前科目
 */
void navigation_stop_course(void)
{
    nav_sys.course_state = COURSE_STATE_IDLE;
    nav_sys.course_type  = 0;

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
 * @param point_index 点位索引 (根据当前科目类型解释)
 * @param lat 纬度
 * @param lon 经度
 */
void navigation_collect_point(uint8_t point_index, double lat, double lon)
{
    if (point_index < MAX_WAYPOINTS)
    {
        // 根据当前科目类型决定存储位置
        if (nav_sys.course_type == 2 && point_index < MAX_CONES)
        {
            // 科目二: 存储锥桶位置
            nav_sys.cones[point_index][0] = lat;
            nav_sys.cones[point_index][1] = lon;
            printf("采集锥桶点位[%d]: 纬度 %.6f, 经度 %.6f\r\n", point_index, lat, lon);
        }
        else
        {
            // 科目一/三: 存储航点
            nav_sys.waypoints[point_index][0] = lat;
            nav_sys.waypoints[point_index][1] = lon;
            printf("采集航点位[%d]: 纬度 %.6f, 经度 %.6f\r\n", point_index, lat, lon);
        }
    }
}

/**
 * @brief 发送导航数据到上位机
 */
void navigation_send_data(void)
{
    seekfree_assistant_oscilloscope_data.data[0] = nav_sys.current_lat;
    seekfree_assistant_oscilloscope_data.data[1] = nav_sys.current_lon;
    seekfree_assistant_oscilloscope_data.data[2] = nav_sys.current_yaw;
    seekfree_assistant_oscilloscope_data.data[3] = nav_sys.target_steer;
    seekfree_assistant_oscilloscope_data.data[4] = nav_sys.target_speed;
    seekfree_assistant_oscilloscope_data.data[5] =
        nav_sys.course_type == 1 ? nav_sys.course1_stage
                                 : (nav_sys.course_type == 2 ? nav_sys.course2_stage : nav_sys.course3_stage);
    seekfree_assistant_oscilloscope_data.data[6]     = nav_sys.course_state;
    seekfree_assistant_oscilloscope_data.data[7]     = nav_sys.course_type;
    seekfree_assistant_oscilloscope_data.channel_num = 8;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

/*--------------------------------私有函数实现--------------------------------*/
/**
 * @brief 标准化角度到[-180, 180]范围
 */
static double normalize_angle(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

/**
 * @brief 更新科目一导航
 */
static void update_course1(void)
{
    double target_lat = 0.0, target_lon = 0.0;
    double dist_to_target = 0.0;

    // 根据当前阶段选择目标点
    switch (nav_sys.course1_stage)
    {
    case COURSE1_STAGE_START:
        // 目标是掉头起点
        target_lat = nav_sys.waypoints[1][0];
        target_lon = nav_sys.waypoints[1][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

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

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

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
    double target_azimuth = get_two_points_azimuth(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

    // 计算航向误差
    double heading_error = normalize_angle(target_azimuth - nav_sys.current_yaw);

    // 计算距离(仅在非掉头阶段使用)
    if (nav_sys.course1_stage != COURSE1_STAGE_TURN)
    {
        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);
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
}

/**
 * @brief 更新科目二导航
 */
static void update_course2(void)
{
    double target_lat = 0.0, target_lon = 0.0;
    double dist_to_target = 0.0;
    double target_azimuth = 0.0;
    double heading_error  = 0.0;

    // 根据当前阶段选择目标
    switch (nav_sys.course2_stage)
    {
    case COURSE2_STAGE_START:
        // 起点出发到第一个锥桶
        target_lat = nav_sys.cones[0][0];
        target_lon = nav_sys.cones[0][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // 如果接近第一个锥桶
        if (dist_to_target < CONE_SWITCH_DIST)
        {
            nav_sys.course2_stage = COURSE2_STAGE_FORWARD;
            nav_sys.current_cone  = 0;
            printf("到达第一个锥桶，开始正向绕桩\r\n");
        }
        break;

    case COURSE2_STAGE_FORWARD:
        // 正向绕桩段

        // 如果已经处理完所有锥桶
        if (nav_sys.current_cone >= nav_sys.total_cones - 1)
        {
            // 进入掉头阶段
            nav_sys.course2_stage = COURSE2_STAGE_TURN;
            nav_sys.initial_yaw   = nav_sys.current_yaw;
            printf("完成正向绕桩，开始掉头\r\n");
            break;
        }

        // 确定当前目标锥桶位置
        uint8_t next_cone = nav_sys.current_cone + 1;
        target_lat        = nav_sys.cones[next_cone][0];
        target_lon        = nav_sys.cones[next_cone][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // S型路线穿行逻辑：接近下一个锥桶时切换方向
        if (dist_to_target < CONE_SWITCH_DIST)
        {
            nav_sys.current_cone     = next_cone;
            nav_sys.slalom_direction = !nav_sys.slalom_direction;   // 切换绕桩方向
            printf("接近锥桶 %d，切换绕行方向\r\n", next_cone);
        }
        break;

    case COURSE2_STAGE_TURN:
        // 掉头区域
        // 计算最后一个锥桶的位置作为目标点
        target_lat = nav_sys.cones[nav_sys.total_cones - 1][0];
        target_lon = nav_sys.cones[nav_sys.total_cones - 1][1];

        // 计算航向变化
        float yaw_change = normalize_angle(nav_sys.current_yaw - nav_sys.initial_yaw);
        if (yaw_change < 0) yaw_change += 360.0;

        // 如果完成掉头(航向变化超过阈值)
        if (yaw_change > TURN_ANGLE_THRESHOLD)
        {
            nav_sys.course2_stage    = COURSE2_STAGE_BACKWARD;
            nav_sys.current_cone     = nav_sys.total_cones - 1;   // 从最后一个锥桶开始反向绕桩
            nav_sys.slalom_direction = 0;                         // 重置绕桩方向
            printf("掉头完成，开始反向绕桩\r\n");
        }
        break;

    case COURSE2_STAGE_BACKWARD:
        // 反向绕桩段

        // 如果已经回到第一个锥桶
        if (nav_sys.current_cone <= 0)
        {
            // 进入终点阶段
            nav_sys.course2_stage = COURSE2_STAGE_END;
            printf("完成反向绕桩，前往终点\r\n");
            break;
        }

        // 确定当前目标锥桶位置
        uint8_t prev_cone = nav_sys.current_cone - 1;
        target_lat        = nav_sys.cones[prev_cone][0];
        target_lon        = nav_sys.cones[prev_cone][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // S型路线穿行逻辑：接近前一个锥桶时切换方向
        if (dist_to_target < CONE_SWITCH_DIST)
        {
            nav_sys.current_cone     = prev_cone;
            nav_sys.slalom_direction = !nav_sys.slalom_direction;   // 切换绕桩方向
            printf("接近锥桶 %d，切换绕行方向\r\n", prev_cone);
        }
        break;

    case COURSE2_STAGE_END:
        // 返回终点 (可以使用第一个锥桶的镜像位置作为终点)
        target_lat = nav_sys.cones[0][0] * 2 - nav_sys.cones[1][0];   // 简单估算终点位置
        target_lon = nav_sys.cones[0][1] * 2 - nav_sys.cones[1][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // 如果到达终点
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course2_stage = COURSE2_STAGE_COMPLETE;
            nav_sys.course_state  = COURSE_STATE_COMPLETED;

            // 停止车辆
            nav_sys.target_steer = 0;
            nav_sys.target_speed = 0;

            printf("到达终点，科目二完成\r\n");
            return;
        }
        break;

    case COURSE2_STAGE_COMPLETE:
        // 已完成，保持停止状态
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        return;
    }

    // 计算到目标的方位角
    target_azimuth = get_two_points_azimuth(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

    // 计算航向误差
    heading_error = normalize_angle(target_azimuth - nav_sys.current_yaw);

    // 科目二特殊逻辑：S型绕桩需要根据当前方向和锥桶位置调整航向角
    if (nav_sys.course2_stage == COURSE2_STAGE_FORWARD || nav_sys.course2_stage == COURSE2_STAGE_BACKWARD)
    {
        // 调整航向角，使车辆从锥桶的指定侧绕行
        if (nav_sys.slalom_direction == 0)   // 左侧绕行
        {
            heading_error -= 30.0;   // 向左偏转一定角度
        }
        else   // 右侧绕行
        {
            heading_error += 30.0;   // 向右偏转一定角度
        }
    }

    // 计算转向控制量
    nav_sys.target_steer = heading_error * STEER_GAIN;

    // 限制转向量
    if (nav_sys.target_steer > MAX_STEER) nav_sys.target_steer = MAX_STEER;
    if (nav_sys.target_steer < -MAX_STEER) nav_sys.target_steer = -MAX_STEER;

    // 设置速度
    if (nav_sys.course2_stage == COURSE2_STAGE_TURN)
    {
        // 掉头阶段使用转弯速度
        nav_sys.target_speed = TURNING_SPEED;
    }
    else if (nav_sys.course2_stage == COURSE2_STAGE_FORWARD || nav_sys.course2_stage == COURSE2_STAGE_BACKWARD)
    {
        // 绕桩阶段使用绕桩速度
        nav_sys.target_speed = SLALOM_SPEED;
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
}

/**
 * @brief 检测当前路面元素类型
 * @param lat 当前纬度
 * @param lon 当前经度
 * @return 路面元素类型
 */
static uint8_t detect_road_element(double lat, double lon)
{
    // 这里可以根据实际情况实现路面元素的检测逻辑
    // 例如，可以预先定义一些坡道和颠簸路段的区域

    // 简单示例：假设特定区域是坡道或颠簸路段
    // 坡道区域
    if (lat > 31.2306 && lat < 31.2308 && lon > 121.4739 && lon < 121.4741)
    {
        return ROAD_SLOPE;
    }

    // 颠簸区域
    if (lat > 31.2310 && lat < 31.2312 && lon > 121.4743 && lon < 121.4745)
    {
        return ROAD_BUMPY;
    }

    // 默认为正常路面
    return ROAD_NORMAL;
}

/**
 * @brief 更新科目三导航
 */
static void update_course3(void)
{
    // 科目三基本逻辑与科目一相似，但增加了路面元素识别
    double target_lat = 0.0, target_lon = 0.0;
    double dist_to_target = 0.0;

    // 检测当前路面元素
    nav_sys.current_road = detect_road_element(nav_sys.current_lat, nav_sys.current_lon);

    // 根据当前阶段选择目标点
    switch (nav_sys.course3_stage)
    {
    case COURSE3_STAGE_START:
        // 目标是掉头起点
        target_lat = nav_sys.waypoints[1][0];
        target_lon = nav_sys.waypoints[1][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // 如果到达掉头起点
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course3_stage = COURSE3_STAGE_TURN;
            nav_sys.initial_yaw   = nav_sys.current_yaw;   // 记录进入掉头区的初始航向
            printf("到达掉头起点，开始掉头\r\n");
        }
        break;

    case COURSE3_STAGE_TURN:
        // 目标是掉头结束点
        target_lat = nav_sys.waypoints[2][0];
        target_lon = nav_sys.waypoints[2][1];

        // 计算航向变化
        float yaw_change = normalize_angle(nav_sys.current_yaw - nav_sys.initial_yaw);
        if (yaw_change < 0) yaw_change += 360.0;

        // 如果完成掉头(航向变化超过阈值)
        if (yaw_change > TURN_ANGLE_THRESHOLD)
        {
            nav_sys.course3_stage = COURSE3_STAGE_RETURN;
            printf("掉头完成，开始返回终点\r\n");
        }
        break;

    case COURSE3_STAGE_RETURN:
        // 目标是终点
        target_lat = nav_sys.waypoints[3][0];
        target_lon = nav_sys.waypoints[3][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // 如果到达终点
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course3_stage = COURSE3_STAGE_COMPLETE;
            nav_sys.course_state  = COURSE_STATE_COMPLETED;

            // 停止车辆
            nav_sys.target_steer = 0;
            nav_sys.target_speed = 0;

            printf("到达终点，科目三完成\r\n");
            return;
        }
        break;

    case COURSE3_STAGE_COMPLETE:
        // 已完成，保持停止状态
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        return;
    }

    // 计算到目标点的方位角
    double target_azimuth = get_two_points_azimuth(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

    // 计算航向误差
    double heading_error = normalize_angle(target_azimuth - nav_sys.current_yaw);

    // 计算距离(仅在非掉头阶段使用)
    if (nav_sys.course3_stage != COURSE3_STAGE_TURN)
    {
        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);
    }

    // 计算转向控制量
    nav_sys.target_steer = heading_error * STEER_GAIN;

    // 限制转向量
    if (nav_sys.target_steer > MAX_STEER) nav_sys.target_steer = MAX_STEER;
    if (nav_sys.target_steer < -MAX_STEER) nav_sys.target_steer = -MAX_STEER;

    // 根据当前路面元素和阶段设置速度
    if (nav_sys.current_road == ROAD_SLOPE)
    {
        // 坡道速度
        nav_sys.target_speed = SLOPE_SPEED;
        printf("检测到坡道，调整速度\r\n");
    }
    else if (nav_sys.current_road == ROAD_BUMPY)
    {
        // 颠簸路段速度
        nav_sys.target_speed = BUMPY_SPEED;
        printf("检测到颠簸路段，降低速度\r\n");
    }
    else if (nav_sys.course3_stage == COURSE3_STAGE_TURN)
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
}