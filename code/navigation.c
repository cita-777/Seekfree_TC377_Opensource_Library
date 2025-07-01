/**
 * *****************************************************************************
 * @file        navigation.c
 * @brief       �ع�����ϵͳ��֧��IMU+������ģʽ��GPSģʽ
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-05-24
 * @copyright   cita
 * *****************************************************************************
 */

#include "navigation.h"
#include "zf_common_headfile.h"

/*----------------------------------�궨��-----------------------------------*/
// ����뾶���ף�
#define EARTH_RADIUS 6378137.0
// ���ȽǶ�ת��
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886

// ����ϵ����
#define MAX_WAYPOINTS 20      // ���·��������
#define WAYPOINT_RADIUS 2.0   // �����л��뾶(��)

// ��Ŀһ�׶ζ���
#define COURSE1_STAGE_START 0      // ��㵽��ͷ���
#define COURSE1_STAGE_TURN 1       // ��ͷ��
#define COURSE1_STAGE_RETURN 2     // ��ͷ�����㵽�յ�
#define COURSE1_STAGE_COMPLETE 3   // ���

// ��Ŀ���׶ζ���
#define COURSE2_STAGE_START 0      // ����������һ��׶Ͱ
#define COURSE2_STAGE_FORWARD 1    // ������׮��
#define COURSE2_STAGE_TURN 2       // ��ͷ��
#define COURSE2_STAGE_BACKWARD 3   // ������׮��
#define COURSE2_STAGE_END 4        // �����յ�
#define COURSE2_STAGE_COMPLETE 5   // ���

// ��Ŀ���׶ζ���
#define COURSE3_STAGE_START 0      // ��㵽��ͷ���
#define COURSE3_STAGE_TURN 1       // ��ͷ��
#define COURSE3_STAGE_RETURN 2     // ��ͷ�����㵽�յ�
#define COURSE3_STAGE_COMPLETE 3   // ���

// ·��Ԫ�ض���
#define ROAD_NORMAL 0   // ����·��
#define ROAD_SLOPE 1    // �µ�
#define ROAD_BUMPY 2    // ����·��

// ��������
#define TURN_ANGLE_THRESHOLD 165.0   // ��ͷ��ɵĽǶ���ֵ(��)
#define STEER_GAIN 1.0               // ת������
#define MAX_STEER 20.0               // ���ת��Ƕ�

// �ٶȲ���
#define NORMAL_SPEED 10   // �����ٶ�
#define TURNING_SPEED 6   // ת���ٶ�
#define SLOW_SPEED 40     // ����
#define SLOPE_SPEED 5     // �µ��ٶ�
#define BUMPY_SPEED 3     // ����·���ٶ�
#define SLALOM_SPEED 7    // ��׮�ٶ�

// ��Ŀ������
#define MAX_CONES 10           // ���׶Ͱ����
#define CONE_SWITCH_DIST 1.5   // ׶Ͱ�л�����(��)

/*----------------------------------���Ͷ���----------------------------------*/
// ����ģʽ����
typedef enum
{
    NAV_MODE_GPS         = 0,   // GPSģʽ��Ĭ�ϣ�
    NAV_MODE_ENCODER_IMU = 1    // �ߵ�/������ģʽ������������
} nav_mode_t;

// ����ϵͳ�ṹ��
typedef struct
{
    // ����״̬
    uint8_t    course_state;   // ��Ŀ״̬
    uint8_t    course_type;    // ��Ŀ���� (1=��Ŀһ, 2=��Ŀ��, 3=��Ŀ��)
    nav_mode_t nav_mode;       // ����ģʽ

    // ��ǰλ�úͺ���
    double current_lat;   // ��ǰγ��
    double current_lon;   // ��ǰ����
    float  current_yaw;   // ��ǰ�����(��)

    // ������� (��GPSģʽʹ��)
    double  target_steer;   // Ŀ��ת���
    int16_t target_speed;   // Ŀ���ٶ�

    // �ߵ�/������ģʽ����
    uint8_t  encoder_mode_ready;        // ������ģʽ�Ƿ�׼������(��ʱ��)
    uint32_t encoder_mode_start_time;   // ������ģʽ����ʱ���

    // GPSģʽ����
    uint8_t current_waypoint;              // ��ǰ��������
    double  waypoints[MAX_WAYPOINTS][2];   // �������� [γ��, ����]
    uint8_t total_waypoints;               // �ܺ�������
    uint8_t course1_stage;                 // ��Ŀһ�׶�
    uint8_t course2_stage;                 // ��Ŀ���׶�
    uint8_t course3_stage;                 // ��Ŀ���׶�
    uint8_t current_road;                  // ��ǰ·������(���ڿ�Ŀ��)
    float   initial_yaw;                   // ��ʼ�����(���ڵ�ͷ�ж�)

    // ��Ŀ��GPSģʽ�ض�����
    double  cones[MAX_CONES][2];   // ׶Ͱλ�� [γ��, ����]
    uint8_t total_cones;           // ׶Ͱ����
    uint8_t current_cone;          // ��ǰ׶Ͱ����
    uint8_t slalom_direction;      // ��ǰ��׮����(0=���, 1=�Ҳ�)
} NavSystem;

/*----------------------------------ȫ�ֱ���----------------------------------*/
static NavSystem nav_sys;   // ����ϵͳʵ��

/*----------------------------------˽�к�������------------------------------*/
static double  normalize_angle(double angle);
static void    update_course1_encoder(void);   // ������ģʽ��Ŀһ
static void    update_course2_encoder(void);   // ������ģʽ��Ŀ��
static void    update_course3_encoder(void);   // ������ģʽ��Ŀ��
static void    update_course1_gps(void);       // GPSģʽ��Ŀһ
static void    update_course2_gps(void);       // GPSģʽ��Ŀ��
static void    update_course3_gps(void);       // GPSģʽ��Ŀ��
static uint8_t detect_road_element(double lat, double lon);

/*--------------------------------�����ӿں���--------------------------------*/
/**
 * @brief ��ʼ������ϵͳ
 */
void navigation_init(void)
{
    // ��ʼ������״̬
    nav_sys.course_state = COURSE_STATE_IDLE;
    nav_sys.course_type  = 0;
    nav_sys.nav_mode     = NAV_MODE_GPS;   // Ĭ��GPSģʽ

    // ��ʼ��λ�úͺ���
    nav_sys.current_lat = 0.0;
    nav_sys.current_lon = 0.0;
    nav_sys.current_yaw = 0.0f;

    // ��ʼ���������
    nav_sys.target_steer = 0.0;
    nav_sys.target_speed = 0;

    // ��ʼ��������ģʽ����
    nav_sys.encoder_mode_ready      = 0;
    nav_sys.encoder_mode_start_time = 0;

    // ��ʼ��GPSģʽ����
    nav_sys.current_waypoint = 0;
    nav_sys.total_waypoints  = 4;
    nav_sys.course1_stage    = COURSE1_STAGE_START;
    nav_sys.course2_stage    = COURSE2_STAGE_START;
    nav_sys.course3_stage    = COURSE3_STAGE_START;
    nav_sys.current_road     = ROAD_NORMAL;
    nav_sys.initial_yaw      = 0.0f;

    // ��ʼ����Ŀ������
    nav_sys.total_cones      = 5;
    nav_sys.current_cone     = 0;
    nav_sys.slalom_direction = 0;

    // ��ʼ��Ĭ�Ϻ���(��Ŀһ)
    nav_sys.waypoints[0][0] = 31.2304;
    nav_sys.waypoints[0][1] = 121.4737;   // ���
    nav_sys.waypoints[1][0] = 31.2314;
    nav_sys.waypoints[1][1] = 121.4747;   // ��ͷ���
    nav_sys.waypoints[2][0] = 31.2312;
    nav_sys.waypoints[2][1] = 121.4745;   // ��ͷ������
    nav_sys.waypoints[3][0] = 31.2304;
    nav_sys.waypoints[3][1] = 121.4737;   // �յ�

    // ��ʼ��׶Ͱλ��(��Ŀ��)
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

    printf("����ϵͳ�ѳ�ʼ�� (ģʽ: %s)\r\n", nav_sys.nav_mode == NAV_MODE_GPS ? "GPS" : "������/IMU");
}

void navigation_proc(void)
{
    navigation_update(gnss.latitude, gnss.longitude, gnss.state, g_imu_angle.yaw);
    disable_handler("navigation");
}

/**
 * @brief �������º���
 */
void navigation_update(double gps_lat, double gps_lon, uint8_t gps_valid, float imu_yaw)
{
    // ���µ�ǰ�����(����ģʽ����Ҫ)
    nav_sys.current_yaw = imu_yaw;
    while (nav_sys.current_yaw < 0) nav_sys.current_yaw += 360.0f;
    while (nav_sys.current_yaw >= 360.0f) nav_sys.current_yaw -= 360.0f;

    // ���ݵ���ģʽ���д���
    if (nav_sys.nav_mode == NAV_MODE_ENCODER_IMU)
    {
        // ������/IMUģʽ����

        // ���0.5����ʱ
        if (!nav_sys.encoder_mode_ready && (system_getval_ms() - nav_sys.encoder_mode_start_time) >= 500)
        {
            nav_sys.encoder_mode_ready = 1;
            printf("������/IMUģʽ׼������\r\n");
        }

        // �źż���߼�
        if (nav_sys.encoder_mode_ready)
        {
            if (uart_receiver.state != 1 || uart_receiver.channel[3] != 992)
            {
                // SBUS�źŶ�ʧ��ֹͣ����ȫͣ��
                drv8701_motor_speed_ctrl(0);
                servo_set(SERVO_MOTOR_MID);   // �������
                nav_sys.course_state       = COURSE_STATE_IDLE;
                nav_sys.encoder_mode_ready = 0;
                printf("SBUS�źŶ�ʧ����ȫͣ��\r\n");
                return;
            }
        }

        // ִ�б�����ģʽ����
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
        // GPSģʽ����

        // ����GPSλ��
        if (gps_valid)
        {
            nav_sys.current_lat = gps_lat;
            nav_sys.current_lon = gps_lon;
        }

        // ִ��GPSģʽ����
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

            // �����������
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

/*----------------------------------GPSģʽ��������----------------------------*/
void navigation_start_course1(void)
{
    // GPSģʽ��ʱ���ã���ȫ����
    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_speed_ctrl(0);
    printf("GPSģʽ�ѽ��ã���ʹ�ñ�����ģʽ\r\n");
}

void navigation_start_course2(void)
{
    // GPSģʽ��ʱ���ã���ȫ����
    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_speed_ctrl(0);
    printf("GPSģʽ�ѽ��ã���ʹ�ñ�����ģʽ\r\n");
}

void navigation_start_course3(void)
{
    // GPSģʽ��ʱ���ã���ȫ����
    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_speed_ctrl(0);
    printf("GPSģʽ�ѽ��ã���ʹ�ñ�����ģʽ\r\n");
}

/*----------------------------------������ģʽ��������------------------------*/
void navigation_start_course1_key(void)
{
    nav_sys.course_state            = COURSE_STATE_RUNNING;
    nav_sys.course_type             = 1;
    nav_sys.nav_mode                = NAV_MODE_ENCODER_IMU;
    nav_sys.course1_stage           = COURSE1_STAGE_START;
    nav_sys.current_waypoint        = 0;
    nav_sys.encoder_mode_start_time = system_getval_ms();
    nav_sys.encoder_mode_ready      = 0;
    printf("��Ŀһ������ģʽ������0.5���ʼ\r\n");
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
    printf("��Ŀ��������ģʽ������0.5���ʼ\r\n");
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
    printf("��Ŀ��������ģʽ������0.5���ʼ\r\n");
}

/*----------------------------------�����ӿں���------------------------------*/
void navigation_stop_course(void)
{
    nav_sys.course_state       = COURSE_STATE_IDLE;
    nav_sys.course_type        = 0;
    nav_sys.target_steer       = 0;
    nav_sys.target_speed       = 0;
    nav_sys.encoder_mode_ready = 0;

    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_speed_ctrl(0);
    printf("��Ŀֹͣ\r\n");
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
            printf("�ɼ�׶Ͱ��λ[%d]: %.6f, %.6f\r\n", point_index, lat, lon);
        }
        else
        {
            nav_sys.waypoints[point_index][0] = lat;
            nav_sys.waypoints[point_index][1] = lon;
            printf("�ɼ�����[%d]: %.6f, %.6f\r\n", point_index, lat, lon);
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

/*--------------------------------˽�к���ʵ��--------------------------------*/
static double normalize_angle(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

/*----------------------------------������ģʽ��������------------------------*/
static void update_course1_encoder(void)
{
    // ��Ŀһ�����ڱ�����ֵ�����׶ο���
    if (cumulative_encoder_data_1 >= 600000)
    {
        drv8701_motor_speed_ctrl(0);
        servo_set(SERVO_MOTOR_MID);   // �������
        nav_sys.course_state = COURSE_STATE_COMPLETED;
        printf("��Ŀһ������ģʽ���\r\n");
    }
    else if (cumulative_encoder_data_1 > 200000 && cumulative_encoder_data_1 < 400000)
    {
        drv8701_motor_speed_ctrl(5);          // ��ͷ�׶ε���
        servo_set_pd_extended(180.0, true);   // ��ͷ�������
    }
    else
    {
        drv8701_motor_speed_ctrl(10.0);     // �����ٶ�
        servo_set_pd_extended(0.0, true);   // ֱ�ж������
    }
}

static void update_course2_encoder(void)
{
    // ��Ŀ��������S�����
    drv8701_motor_speed_ctrl(10);   // �̶��ٶ�

    // ���Ʋ���
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
        printf("��Ŀ��������ģʽ���\r\n");
        return;
    }

    // 1. ��ʼֱ��
    if (enc < init_straight)
    {
        servo_set_pd_extended(0.0, true);
    }
    // 2. ȥ��S�䣨3�Σ�
    else if (enc < init_straight + 3 * s_segment_go)
    {
        int   phase = (int)((enc - init_straight) / s_segment_go) % 3;
        float angle = (phase == 1 ? -s_angle_go : s_angle_go);
        servo_set_pd_extended(angle * servo_scale, true);
    }
    // 3. ��ͷǰֱ��
    else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight)
    {
        servo_set_pd_extended(0.0, true);
    }
    // 4. ��ͷ�׶�
    else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight + turn_distance)
    {
        servo_set_pd_extended(turn_angle * servo_scale, true);
    }
    // 5. �س�S�䣨3�Σ�
    else if (enc < init_straight + 3 * s_segment_go + pre_turn_straight + turn_distance + 3 * s_segment_back)
    {
        int phase =
            (int)((enc - init_straight - 3 * s_segment_go - pre_turn_straight - turn_distance) / s_segment_back) % 3;
        float angle = turn_angle + (phase == 1 ? s_angle_back : -s_angle_back);
        servo_set_pd_extended(angle * servo_scale, true);
    }
    // 6. ����ֱ��
    else
    {
        servo_set_pd_extended(turn_angle * servo_scale, true);
    }
}

static void update_course3_encoder(void)
{
    // ��Ŀ�������Ŀһ��ͬ�ı���������
    update_course1_encoder();
}

/*----------------------------------GPSģʽ��������----------------------------*/
static void update_course1_gps(void)
{
    // GPSģʽ��Ŀһ�����߼� (TODO: ʵ��������GPS����)
    nav_sys.target_steer = 0;
    nav_sys.target_speed = NORMAL_SPEED;
    printf("��ĿһGPSģʽ������...\r\n");
}

static void update_course2_gps(void)
{
    // GPSģʽ��Ŀ�������߼� (TODO: ʵ��������GPS����)
    nav_sys.target_steer = 0;
    nav_sys.target_speed = NORMAL_SPEED;
    printf("��Ŀ��GPSģʽ������...\r\n");
}

static void update_course3_gps(void)
{
    // GPSģʽ��Ŀ�������߼� (TODO: ʵ��������GPS����)
    nav_sys.target_steer = 0;
    nav_sys.target_speed = NORMAL_SPEED;
    printf("��Ŀ��GPSģʽ������...\r\n");
}

static uint8_t detect_road_element(double lat, double lon)
{
    // ·��Ԫ�ؼ���߼� (TODO: ����ʵ������ʵ��)
    return ROAD_NORMAL;
}
