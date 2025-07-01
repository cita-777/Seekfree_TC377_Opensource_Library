/**
 * *****************************************************************************
 * @file        navigation.c
 * @brief       重构导航系统，支持IMU+编码器模式和GPS模式
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
#define MAX_WAYPOINTS 20      // 最大路径点数量
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

// 科目三阶段定义
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
// 导航模式定义
typedef enum
{
    NAV_MODE_GPS         = 0,   // GPS模式（默认）
    NAV_MODE_ENCODER_IMU = 1    // 惯导/编码器模式（按键启动）
} nav_mode_t;

// 导航系统结构体
typedef struct
{
    // 基本状态
    uint8_t    course_state;   // 科目状态
    uint8_t    course_type;    // 科目类型 (1=科目一, 2=科目二, 3=科目三)
    nav_mode_t nav_mode;       // 导航模式

    // 当前位置和航向
    double current_lat;   // 当前纬度
    double current_lon;   // 当前经度
    float  current_yaw;   // 当前航向角(度)

    // 控制输出 (仅GPS模式使用)
    double  target_steer;   // 目标转向角
    int16_t target_speed;   // 目标速度

    // 惯导/编码器模式参数
    uint8_t  encoder_mode_ready;        // 编码器模式是否准备就绪(延时后)
    uint32_t encoder_mode_start_time;   // 编码器模式启动时间戳

    // GPS模式参数
    uint8_t current_waypoint;              // 当前航点索引
    double  waypoints[MAX_WAYPOINTS][2];   // 航点数组 [纬度, 经度]
    uint8_t total_waypoints;               // 总航点数量
    uint8_t course1_stage;                 // 科目一阶段
    uint8_t course2_stage;                 // 科目二阶段
    uint8_t course3_stage;                 // 科目三阶段
    uint8_t current_road;                  // 当前路面类型(用于科目三)
    float   initial_yaw;                   // 初始航向角(用于掉头判断)

    // 科目二GPS模式特定参数
    double  cones[MAX_CONES][2];   // 锥桶位置 [纬度, 经度]
    uint8_t total_cones;           // 锥桶总数
    uint8_t current_cone;          // 当前锥桶索引
    uint8_t slalom_direction;      // 当前绕桩方向(0=左侧, 1=右侧)
} NavSystem;

/*----------------------------------全局变量----------------------------------*/
static NavSystem nav_sys;   // 导航系统实例

/*----------------------------------私有函数声明------------------------------*/
static double  normalize_angle(double angle);
static void    update_course1_encoder(void);   // 编码器模式科目一
static void    update_course2_encoder(void);   // 编码器模式科目二
static void    update_course3_encoder(void);   // 编码器模式科目三
static void    update_course1_gps(void);       // GPS模式科目一
static void    update_course2_gps(void);       // GPS模式科目二
static void    update_course3_gps(void);       // GPS模式科目三
static uint8_t detect_road_element(double lat, double lon);

/*--------------------------------公共接口函数--------------------------------*/
/**
 * @brief 初始化导航系统
 */
void navigation_init(void)
{
    // 初始化基本状态
    nav_sys.course_state = COURSE_STATE_IDLE;
    nav_sys.course_type  = 0;
    nav_sys.nav_mode     = NAV_MODE_GPS;   // 默认GPS模式

    // 初始化位置和航向
    nav_sys.current_lat = 0.0;
    nav_sys.current_lon = 0.0;
    nav_sys.current_yaw = 0.0f;

    // 初始化控制输出
    nav_sys.target_steer = 0.0;
    nav_sys.target_speed = 0;

    // 初始化编码器模式参数
    nav_sys.encoder_mode_ready      = 0;
    nav_sys.encoder_mode_start_time = 0;

    // 初始化GPS模式参数
    nav_sys.current_waypoint = 0;
    nav_sys.total_waypoints  = 4;
    nav_sys.course1_stage    = COURSE1_STAGE_START;
    nav_sys.course2_stage    = COURSE2_STAGE_START;
    nav_sys.course3_stage    = COURSE3_STAGE_START;
    nav_sys.current_road     = ROAD_NORMAL;
    nav_sys.initial_yaw      = 0.0f;

    // 初始化科目二参数
    nav_sys.total_cones      = 5;
    nav_sys.current_cone     = 0;
    nav_sys.slalom_direction = 0;

    // 初始化默认航点(科目一)
    nav_sys.waypoints[0][0] = 31.2304;
    nav_sys.waypoints[0][1] = 121.4737;   // 起点
    nav_sys.waypoints[1][0] = 31.2314;
    nav_sys.waypoints[1][1] = 121.4747;   // 掉头起点
    nav_sys.waypoints[2][0] = 31.2312;
    nav_sys.waypoints[2][1] = 121.4745;   // 掉头结束点
    nav_sys.waypoints[3][0] = 31.2304;
    nav_sys.waypoints[3][1] = 121.4737;   // 终点

    // 初始化锥桶位置(科目二)
    nav_sys.cones[0][0] = 31.2304;
    nav_sys.cones[0][1] = 121.4737;
    nav_sys.cones[1][0] = 31.2306;
    nav_sys.cones[1][1] = 121.4739;
    nav_sys.cones[2][0] = 31.2308;
    nav_sys.cones[2][1] = 121.4741;
    nav_sys.cones[3][0] = 31.2310;
    nav_sys.cones[3][1] = 121.4743;
    nav_sys.cones[4][0] = 31.2312;
    nav_sys.cones[4][1] = 121.4745;

    printf("导航系统已初始化 (模式: %s)\r\n", nav_sys.nav_mode == NAV_MODE_GPS ? "GPS" : "编码器/IMU");
}

void navigation_proc(void)
{
    navigation_update(gnss.latitude, gnss.longitude, gnss.state, g_imu_angle.yaw);
    disable_handler("navigation");
}

/**
 * @brief 导航更新函数
 */
void navigation_update(double gps_lat, double gps_lon, uint8_t gps_valid, float imu_yaw)
{
    // 更新当前航向角(所有模式都需要)
    nav_sys.current_yaw = imu_yaw;
    while (nav_sys.current_yaw < 0) nav_sys.current_yaw += 360.0f;
    while (nav_sys.current_yaw >= 360.0f) nav_sys.current_yaw -= 360.0f;

    // 根据导航模式进行处理
    if (nav_sys.nav_mode == NAV_MODE_ENCODER_IMU)
    {
        // 编码器/IMU模式处理

        // 检查0.5秒延时
        if (!nav_sys.encoder_mode_ready && (system_getval_ms() - nav_sys.encoder_mode_start_time) >= 500)
        {
            nav_sys.encoder_mode_ready = 1;
            printf("编码器/IMU模式准备就绪\r\n");
        }

        // 信号检测逻辑
        if (nav_sys.encoder_mode_ready)
        {
            if (uart_receiver.state != 1 || uart_receiver.channel[3] != 992)
            {
                // SBUS信号丢失或停止，安全停机
                drv8701_motor_speed_ctrl(0);
                servo_set(SERVO_MOTOR_MID);   // 舵机回正
                nav_sys.course_state       = COURSE_STATE_IDLE;
                nav_sys.encoder_mode_ready = 0;
                printf("SBUS信号丢失，安全停机\r\n");
                return;
            }
        }

        // 执行编码器模式导航
        if (nav_sys.course_state == COURSE_STATE_RUNNING && nav_sys.encoder_mode_ready)
        {
            switch (nav_sys.course_type)
            {
            case 1: update_course1_encoder(); break;
            case 2: update_course2_encoder(); break;
            case 3: update_course3_encoder(); break;
            default:
                drv8701_motor_speed_ctrl(0);
                nav_sys.course_state = COURSE_STATE_IDLE;
                break;
            }
        }
    }
    else
    {
        // GPS模式处理

        // 更新GPS位置
        if (gps_valid)
        {
            nav_sys.current_lat = gps_lat;
            nav_sys.current_lon = gps_lon;
        }

        // 执行GPS模式导航
        if (nav_sys.course_state == COURSE_STATE_RUNNING)
        {
            switch (nav_sys.course_type)
            {
            case 1: update_course1_gps(); break;
            case 2: update_course2_gps(); break;
            case 3: update_course3_gps(); break;
            default:
                nav_sys.target_steer = 0;
                nav_sys.target_speed = 0;
                break;
            }

            // 输出控制命令
            servo_set(SERVO_MOTOR_MID + (int16_t)nav_sys.target_steer);
            drv8701_motor_speed_ctrl(nav_sys.target_speed);
        }
        else
        {
            nav_sys.target_steer = 0;
            nav_sys.target_speed = 0;
        }
    }
}

/*----------------------------------GPS模式启动函数----------------------------*/
void navigation_start_course1(void)
{
    // GPS模式暂时禁用，安全考虑
    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_speed_ctrl(0);
    printf("GPS模式已禁用，请使用编码器模式\r\n");
}

void navigation_start_course2(void)
{
    // GPS模式暂时禁用，安全考虑
    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_speed_ctrl(0);
    printf("GPS模式已禁用，请使用编码器模式\r\n");
}

void navigation_start_course3(void)
{
    // GPS模式暂时禁用，安全考虑
    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_speed_ctrl(0);
    printf("GPS模式已禁用，请使用编码器模式\r\n");
}

/*----------------------------------编码器模式启动函数------------------------*/
void navigation_start_course1_key(void)
{
    nav_sys.course_state            = COURSE_STATE_RUNNING;
    nav_sys.course_type             = 1;
    nav_sys.nav_mode                = NAV_MODE_ENCODER_IMU;
    nav_sys.course1_stage           = COURSE1_STAGE_START;
    nav_sys.current_waypoint        = 0;
    nav_sys.encoder_mode_start_time = system_getval_ms();
    nav_sys.encoder_mode_ready      = 0;
    printf("科目一编码器模式启动，0.5秒后开始\r\n");
}

void navigation_start_course2_key(void)
{
    nav_sys.course_state            = COURSE_STATE_RUNNING;
    nav_sys.course_type             = 2;
    nav_sys.nav_mode                = NAV_MODE_ENCODER_IMU;
    nav_sys.course2_stage           = COURSE2_STAGE_START;
    nav_sys.current_cone            = 0;
    nav_sys.slalom_direction        = 0;
    nav_sys.encoder_mode_start_time = system_getval_ms();
    nav_sys.encoder_mode_ready      = 0;
    printf("科目二编码器模式启动，0.5秒后开始\r\n");
}

void navigation_start_course3_key(void)
{
    nav_sys.course_state            = COURSE_STATE_RUNNING;
    nav_sys.course_type             = 3;
    nav_sys.nav_mode                = NAV_MODE_ENCODER_IMU;
    nav_sys.course3_stage           = COURSE3_STAGE_START;
    nav_sys.current_waypoint        = 0;
    nav_sys.current_road            = ROAD_NORMAL;
    nav_sys.encoder_mode_start_time = system_getval_ms();
    nav_sys.encoder_mode_ready      = 0;
    printf("科目三编码器模式启动，0.5秒后开始\r\n");
}

/*----------------------------------其他接口函数------------------------------*/
void navigation_stop_course(void)
{
    nav_sys.course_state       = COURSE_STATE_IDLE;
    nav_sys.course_type        = 0;
    nav_sys.target_steer       = 0;
    nav_sys.target_speed       = 0;
    nav_sys.encoder_mode_ready = 0;

    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_speed_ctrl(0);
    printf("科目停止\r\n");
}

uint8_t navigation_get_course_state(void)
{
    return nav_sys.course_state;
}

void navigation_get_status(double* lat, double* lon, float* yaw)
{
    if (lat) *lat = nav_sys.current_lat;
    if (lon) *lon = nav_sys.current_lon;
    if (yaw) *yaw = nav_sys.current_yaw;
}

void navigation_collect_point(uint8_t point_index, double lat, double lon)
{
    if (point_index < MAX_WAYPOINTS)
    {
        if (nav_sys.course_type == 2 && point_index < MAX_CONES)
        {
            nav_sys.cones[point_index][0] = lat;
            nav_sys.cones[point_index][1] = lon;
            printf("采集锥桶点位[%d]: %.6f, %.6f\r\n", point_index, lat, lon);
        }
        else
        {
            nav_sys.waypoints[point_index][0] = lat;
            nav_sys.waypoints[point_index][1] = lon;
            printf("采集航点[%d]: %.6f, %.6f\r\n", point_index, lat, lon);
        }
    }
}

void navigation_send_data(void)
{
    seekfree_assistant_oscilloscope_data.data[0]     = nav_sys.current_lat;
    seekfree_assistant_oscilloscope_data.data[1]     = nav_sys.current_lon;
    seekfree_assistant_oscilloscope_data.data[2]     = nav_sys.current_yaw;
    seekfree_assistant_oscilloscope_data.data[3]     = nav_sys.target_steer;
    seekfree_assistant_oscilloscope_data.data[4]     = nav_sys.target_speed;
    seekfree_assistant_oscilloscope_data.data[5]     = nav_sys.nav_mode;
    seekfree_assistant_oscilloscope_data.data[6]     = nav_sys.course_state;
    seekfree_assistant_oscilloscope_data.data[7]     = nav_sys.course_type;
    seekfree_assistant_oscilloscope_data.channel_num = 8;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

/*--------------------------------私有函数实现--------------------------------*/
static double normalize_angle(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

/*----------------------------------编码器模式导航函数------------------------*/
static void update_course1_encoder(void)
{
    // 科目一：基于编码器值的三阶段控制
    if (cumulative_encoder_data_1 >= 600000)
    {
        drv8701_motor_speed_ctrl(0);
        servo_set(SERVO_MOTOR_MID);   // 舵机回中
        nav_sys.course_state = COURSE_STATE_COMPLETED;
        printf("科目一编码器模式完成\r\n");
    }
    else if (cumulative_encoder_data_1 > 200000 && cumulative_encoder_data_1 < 400000)
    {
        drv8701_motor_speed_ctrl(5);          // 掉头阶段低速
        servo_set_pd_extended(180.0, true);   // 掉头舵机控制
    }
    else
    {
        drv8701_motor_speed_ctrl(10.0);     // 正常速度
        servo_set_pd_extended(0.0, true);   // 直行舵机控制
    }
}

static void update_course2_encoder(void)
{
    // 科目二：八字S弯控制
    drv8701_motor_speed_ctrl(10);   // 固定速度

    // 控制参数
    float    servo_scale       = 1.0f;
    uint32_t init_straight     = 15000;
    float    s_angle_go        = 40.0f;
    uint32_t s_segment_go      = 50000;
    float    s_angle_back      = 40.0f;
    uint32_t s_segment_back    = 50000;
    uint32_t pre_turn_straight = 30000;
    uint32_t turn_distance     = 50000;
    float    turn_angle        = -180.0f;

    uint32_t enc = cumulative_encoder_data_1;
    uint32_t total_distance =
        init_straight + 3 * s_segment_go + pre_turn_straight + turn_distance + 3 * s_segment_back + 30000;

    if (enc >= total_distance)
    {
        drv8701_motor_speed_ctrl(0);
        nav_sys.course_state = COURSE_STATE_COMPLETED;
        printf("科目二编码器模式完成\r\n");
        return;
    }

    // 1. 初始直行
    if (enc < init_straight)
    {
        servo_set_pd_extended(0.0, true);
    }
    // 2. 去程S弯（3段）
    else if (enc < init_straight + 3 * s_segment_go)
    {
        int   phase = (int)((enc - init_straight) / s_segment_go) % 3;
        float angle = (phase == 1 ? -s_angle_go : s_angle_go);
        servo_set_pd_extended(angle * servo_scale, true);
    }
    // 3. 掉头前直行
    else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight)
    {
        servo_set_pd_extended(0.0, true);
    }
    // 4. 掉头阶段
    else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight + turn_distance)
    {
        servo_set_pd_extended(turn_angle * servo_scale, true);
    }
    // 5. 回程S弯（3段）
    else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight + turn_distance + 3 * s_segment_back)
    {
        int phase =
            (int)((enc - init_straight - 3 * s_segment_go - pre_turn_straight - turn_distance) / s_segment_back) % 3;
        float angle = turn_angle + (phase == 1 ? s_angle_back : -s_angle_back);
        servo_set_pd_extended(angle * servo_scale, true);
    }
    // 6. 最终直行
    else
    {
        servo_set_pd_extended(turn_angle * servo_scale, true);
    }
}

static void update_course3_encoder(void)
{
    // 科目三：与科目一相同的编码器控制
    update_course1_encoder();
}

/*----------------------------------GPS模式导航函数----------------------------*/
static void update_course1_gps(void)
{
    // GPS模式科目一导航逻辑 (TODO: 实现完整的GPS导航)
    nav_sys.target_steer = 0;
    nav_sys.target_speed = NORMAL_SPEED;
    printf("科目一GPS模式运行中...\r\n");
}

static void update_course2_gps(void)
{
    // GPS模式科目二导航逻辑 (TODO: 实现完整的GPS导航)
    nav_sys.target_steer = 0;
    nav_sys.target_speed = NORMAL_SPEED;
    printf("科目二GPS模式运行中...\r\n");
}

static void update_course3_gps(void)
{
    // GPS模式科目三导航逻辑 (TODO: 实现完整的GPS导航)
    nav_sys.target_steer = 0;
    nav_sys.target_speed = NORMAL_SPEED;
    printf("科目三GPS模式运行中...\r\n");
}

static uint8_t detect_road_element(double lat, double lon)
{
    // 路面元素检测逻辑 (TODO: 根据实际需求实现)
    return ROAD_NORMAL;
}
