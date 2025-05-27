/**
 * *****************************************************************************
 * @file        navigation.c
 * @brief       �򻯵���ϵͳʵ�֣�ʹ��GPS��λ��IMU�����
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
#define MAX_WAYPOINTS 20      // ���·�������� (������֧�ֿ�Ŀ���Ϳ�Ŀ��)
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

// ��Ŀ���׶ζ��壨���ڿ�Ŀһ�ṹ������·��Ԫ�أ�
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
// ����ϵͳ�ṹ��
typedef struct
{
    uint8_t current_waypoint;              // ��ǰ��������
    double  waypoints[MAX_WAYPOINTS][2];   // �������� [γ��, ����]
    uint8_t total_waypoints;               // �ܺ�������

    // ��ǰλ�úͺ���
    double current_lat;   // ��ǰγ��
    double current_lon;   // ��ǰ����
    float  current_yaw;   // ��ǰ�����(��)

    // �������
    double  target_steer;   // Ŀ��ת���
    int16_t target_speed;   // Ŀ���ٶ�

    // ��Ŀ��ز���
    uint8_t course_state;    // ��Ŀ״̬
    uint8_t course_type;     // ��Ŀ���� (1=��Ŀһ, 2=��Ŀ��, 3=��Ŀ��)
    uint8_t course1_stage;   // ��Ŀһ�׶�
    uint8_t course2_stage;   // ��Ŀ���׶�
    uint8_t course3_stage;   // ��Ŀ���׶�
    uint8_t current_road;    // ��ǰ·������(���ڿ�Ŀ��)

    // ��Ŀ���ض�����
    double  cones[MAX_CONES][2];   // ׶Ͱλ�� [γ��, ����]
    uint8_t total_cones;           // ׶Ͱ����
    uint8_t current_cone;          // ��ǰ׶Ͱ����
    uint8_t slalom_direction;      // ��ǰ��׮����(0=���, 1=�Ҳ�)

    float initial_yaw;   // ��ʼ�����(���ڵ�ͷ�ж�)
} NavSystem;

/*----------------------------------ȫ�ֱ���----------------------------------*/
static NavSystem nav_sys;   // ����ϵͳʵ��

/*----------------------------------˽�к���----------------------------------*/
// ������������
static double  normalize_angle(double angle);
static void    update_course1(void);
static void    update_course2(void);
static void    update_course3(void);
static uint8_t detect_road_element(double lat, double lon);

/*--------------------------------�����ӿں���--------------------------------*/
/**
 * @brief ��ʼ������ϵͳ
 */
void navigation_init(void)
{
    // ��ʼ��ϵͳ����
    nav_sys.current_waypoint = 0;
    nav_sys.target_steer     = 0.0;
    nav_sys.target_speed     = 0;
    nav_sys.total_waypoints  = 4;   // ��ĿһĬ����4������

    // ��ʼ����Ŀ��ز���
    nav_sys.course_state  = COURSE_STATE_IDLE;
    nav_sys.course_type   = 0;
    nav_sys.course1_stage = COURSE1_STAGE_START;
    nav_sys.course2_stage = COURSE2_STAGE_START;
    nav_sys.course3_stage = COURSE3_STAGE_START;
    nav_sys.current_road  = ROAD_NORMAL;
    nav_sys.initial_yaw   = 0.0f;

    // ��ʼ����Ŀ������
    nav_sys.total_cones      = 5;   // Ĭ��5��׶Ͱ
    nav_sys.current_cone     = 0;
    nav_sys.slalom_direction = 0;   // ��ʼ��׮����(���)

    // ��ʼ��Ĭ�Ͽ�Ŀ��λ(��ͨ�������ɼ�����)
    // ��Ŀһ
    nav_sys.waypoints[0][0] = 31.2304;    // ���γ��
    nav_sys.waypoints[0][1] = 121.4737;   // ��㾭��
    nav_sys.waypoints[1][0] = 31.2314;    // ��ͷ���γ��
    nav_sys.waypoints[1][1] = 121.4747;   // ��ͷ��㾭��
    nav_sys.waypoints[2][0] = 31.2312;    // ��ͷ������γ��
    nav_sys.waypoints[2][1] = 121.4745;   // ��ͷ�����㾭��
    nav_sys.waypoints[3][0] = 31.2304;    // �յ�γ��
    nav_sys.waypoints[3][1] = 121.4737;   // �յ㾭��

    // ��Ŀ����ʼ׶Ͱλ�ã�ʾ����
    nav_sys.cones[0][0] = 31.2304;    // ��1��׶Ͱγ��
    nav_sys.cones[0][1] = 121.4737;   // ��1��׶Ͱ����
    nav_sys.cones[1][0] = 31.2306;    // ��2��׶Ͱγ��
    nav_sys.cones[1][1] = 121.4739;   // ��2��׶Ͱ����
    nav_sys.cones[2][0] = 31.2308;    // ��3��׶Ͱγ��
    nav_sys.cones[2][1] = 121.4741;   // ��3��׶Ͱ����
    nav_sys.cones[3][0] = 31.2310;    // ��4��׶Ͱγ��
    nav_sys.cones[3][1] = 121.4743;   // ��4��׶Ͱ����
    nav_sys.cones[4][0] = 31.2312;    // ��5��׶Ͱγ��
    nav_sys.cones[4][1] = 121.4745;   // ��5��׶Ͱ����

    printf("����ϵͳ�ѳ�ʼ��.\r\n");
}

void navigation_proc(void)
{

    navigation_update(gnss.latitude, gnss.longitude, gnss.state, g_imu_angle.yaw);   // Ĭ�ϵ��ã�ʵ���������ⲿ����
    // timer_enable();   // ��ʾ��ǰ����״̬
    // printf("Show yaw angle: %.2f\n", g_imu_angle.yaw);
    // timer_disable();
    disable_handler("navigation");   // ���õ����������������ظ�����
}
/**
 * @brief �������º���
 * @param gps_lat ��ǰGPSγ��
 * @param gps_lon ��ǰGPS����
 * @param gps_valid GPS�����Ƿ���Ч
 * @param imu_yaw IMU����� (��)
 */
void navigation_update(double gps_lat, double gps_lon, uint8_t gps_valid, float imu_yaw)
{
    // ���µ�ǰλ��
    if (gps_valid)
    {
        nav_sys.current_lat = gps_lat;
        nav_sys.current_lon = gps_lon;
    }

    // ���µ�ǰ�����(ֻʹ��IMU��yaw��)
    nav_sys.current_yaw = imu_yaw;

    // ��׼������ǵ�[0,360)
    while (nav_sys.current_yaw < 0) nav_sys.current_yaw += 360.0f;
    while (nav_sys.current_yaw >= 360.0f) nav_sys.current_yaw -= 360.0f;

    // �����ǰû�����п�Ŀ��ֱ�ӷ���
    if (nav_sys.course_state != COURSE_STATE_RUNNING)
    {
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        return;
    }

    // ���ݿ�Ŀ���͸��µ���
    switch (nav_sys.course_type)
    {
    case 1:   // ��Ŀһ
        update_course1();
        break;

    case 2:   // ��Ŀ��
        update_course2();
        break;

    case 3:   // ��Ŀ��
        update_course3();
        break;

    default:
        // δ֪��Ŀ���ͣ�ֹͣ����
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        break;
    }

    // �����������
    servo_set(SERVO_MOTOR_MID + (int16_t)nav_sys.target_steer);
    drv8701_motor_set(nav_sys.target_speed);
}

/**
 * @brief ��ȡ��ǰ����״̬
 * @param lat γ��ָ��
 * @param lon ����ָ��
 * @param yaw �����ָ��
 */
void navigation_get_status(double* lat, double* lon, float* yaw)
{
    if (lat) *lat = nav_sys.current_lat;
    if (lon) *lon = nav_sys.current_lon;
    if (yaw) *yaw = nav_sys.current_yaw;
}

/**
 * @brief ��ʼ���п�Ŀһ
 */
void navigation_start_course1(void)
{
    nav_sys.course_state     = COURSE_STATE_RUNNING;
    nav_sys.course_type      = 1;
    nav_sys.course1_stage    = COURSE1_STAGE_START;
    nav_sys.current_waypoint = 0;

    printf("��Ŀһ��ʼ����\r\n");
}

/**
 * @brief ��ʼ���п�Ŀ��
 */
void navigation_start_course2(void)
{
    nav_sys.course_state     = COURSE_STATE_RUNNING;
    nav_sys.course_type      = 2;
    nav_sys.course2_stage    = COURSE2_STAGE_START;
    nav_sys.current_cone     = 0;
    nav_sys.slalom_direction = 0;   // ��ʼ��׮����(���)

    printf("��Ŀ����ʼ����\r\n");
}

/**
 * @brief ��ʼ���п�Ŀ��
 */
void navigation_start_course3(void)
{
    nav_sys.course_state     = COURSE_STATE_RUNNING;
    nav_sys.course_type      = 3;
    nav_sys.course3_stage    = COURSE3_STAGE_START;
    nav_sys.current_waypoint = 0;
    nav_sys.current_road     = ROAD_NORMAL;

    printf("��Ŀ����ʼ����\r\n");
}

/**
 * @brief ֹͣ��ǰ��Ŀ
 */
void navigation_stop_course(void)
{
    nav_sys.course_state = COURSE_STATE_IDLE;
    nav_sys.course_type  = 0;

    // ֹͣ����
    nav_sys.target_steer = 0;
    nav_sys.target_speed = 0;
    servo_set(SERVO_MOTOR_MID);
    drv8701_motor_set(0);

    printf("��Ŀֹͣ����\r\n");
}

/**
 * @brief ��ȡ��ǰ��Ŀ״̬
 * @return ��ǰ��Ŀ״̬
 */
uint8_t navigation_get_course_state(void)
{
    return nav_sys.course_state;
}

/**
 * @brief �ɼ�GPS��λ
 * @param point_index ��λ���� (���ݵ�ǰ��Ŀ���ͽ���)
 * @param lat γ��
 * @param lon ����
 */
void navigation_collect_point(uint8_t point_index, double lat, double lon)
{
    if (point_index < MAX_WAYPOINTS)
    {
        // ���ݵ�ǰ��Ŀ���;����洢λ��
        if (nav_sys.course_type == 2 && point_index < MAX_CONES)
        {
            // ��Ŀ��: �洢׶Ͱλ��
            nav_sys.cones[point_index][0] = lat;
            nav_sys.cones[point_index][1] = lon;
            printf("�ɼ�׶Ͱ��λ[%d]: γ�� %.6f, ���� %.6f\r\n", point_index, lat, lon);
        }
        else
        {
            // ��Ŀһ/��: �洢����
            nav_sys.waypoints[point_index][0] = lat;
            nav_sys.waypoints[point_index][1] = lon;
            printf("�ɼ�����λ[%d]: γ�� %.6f, ���� %.6f\r\n", point_index, lat, lon);
        }
    }
}

/**
 * @brief ���͵������ݵ���λ��
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

/*--------------------------------˽�к���ʵ��--------------------------------*/
/**
 * @brief ��׼���Ƕȵ�[-180, 180]��Χ
 */
static double normalize_angle(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

/**
 * @brief ���¿�Ŀһ����
 */
static void update_course1(void)
{
    double target_lat = 0.0, target_lon = 0.0;
    double dist_to_target = 0.0;

    // ���ݵ�ǰ�׶�ѡ��Ŀ���
    switch (nav_sys.course1_stage)
    {
    case COURSE1_STAGE_START:
        // Ŀ���ǵ�ͷ���
        target_lat = nav_sys.waypoints[1][0];
        target_lon = nav_sys.waypoints[1][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // ��������ͷ���
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course1_stage = COURSE1_STAGE_TURN;
            nav_sys.initial_yaw   = nav_sys.current_yaw;   // ��¼�����ͷ���ĳ�ʼ����
            printf("�����ͷ��㣬��ʼ��ͷ\r\n");
        }
        break;

    case COURSE1_STAGE_TURN:
        // Ŀ���ǵ�ͷ������
        target_lat = nav_sys.waypoints[2][0];
        target_lon = nav_sys.waypoints[2][1];

        // ���㺽��仯
        float yaw_change = normalize_angle(nav_sys.current_yaw - nav_sys.initial_yaw);
        if (yaw_change < 0) yaw_change += 360.0;

        // �����ɵ�ͷ(����仯������ֵ)
        if (yaw_change > TURN_ANGLE_THRESHOLD)
        {
            nav_sys.course1_stage = COURSE1_STAGE_RETURN;
            printf("��ͷ��ɣ���ʼ�����յ�\r\n");
        }
        break;

    case COURSE1_STAGE_RETURN:
        // Ŀ�����յ�
        target_lat = nav_sys.waypoints[3][0];
        target_lon = nav_sys.waypoints[3][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // ��������յ�
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course1_stage = COURSE1_STAGE_COMPLETE;
            nav_sys.course_state  = COURSE_STATE_COMPLETED;

            // ֹͣ����
            nav_sys.target_steer = 0;
            nav_sys.target_speed = 0;

            printf("�����յ㣬��Ŀһ���\r\n");
            return;
        }
        break;

    case COURSE1_STAGE_COMPLETE:
        // ����ɣ�����ֹͣ״̬
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        return;
    }

    // ���㵽Ŀ���ķ�λ��
    double target_azimuth = get_two_points_azimuth(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

    // ���㺽�����
    double heading_error = normalize_angle(target_azimuth - nav_sys.current_yaw);

    // �������(���ڷǵ�ͷ�׶�ʹ��)
    if (nav_sys.course1_stage != COURSE1_STAGE_TURN)
    {
        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);
    }

    // ����ת�������
    nav_sys.target_steer = heading_error * STEER_GAIN;

    // ����ת����
    if (nav_sys.target_steer > MAX_STEER) nav_sys.target_steer = MAX_STEER;
    if (nav_sys.target_steer < -MAX_STEER) nav_sys.target_steer = -MAX_STEER;

    // �����ٶ�
    if (nav_sys.course1_stage == COURSE1_STAGE_TURN)
    {
        // ��ͷ�׶�ʹ��ת���ٶ�
        nav_sys.target_speed = TURNING_SPEED;
    }
    else if (fabs(heading_error) > 45.0)
    {
        // ��Ƕ�ת�򣬽����ٶ�
        nav_sys.target_speed = TURNING_SPEED;
    }
    else if (dist_to_target < 5.0)
    {
        // �ӽ�Ŀ��㣬�����ٶ�
        nav_sys.target_speed = SLOW_SPEED;
    }
    else
    {
        // �����ٶ�
        nav_sys.target_speed = NORMAL_SPEED;
    }
}

/**
 * @brief ���¿�Ŀ������
 */
static void update_course2(void)
{
    double target_lat = 0.0, target_lon = 0.0;
    double dist_to_target = 0.0;
    double target_azimuth = 0.0;
    double heading_error  = 0.0;

    // ���ݵ�ǰ�׶�ѡ��Ŀ��
    switch (nav_sys.course2_stage)
    {
    case COURSE2_STAGE_START:
        // ����������һ��׶Ͱ
        target_lat = nav_sys.cones[0][0];
        target_lon = nav_sys.cones[0][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // ����ӽ���һ��׶Ͱ
        if (dist_to_target < CONE_SWITCH_DIST)
        {
            nav_sys.course2_stage = COURSE2_STAGE_FORWARD;
            nav_sys.current_cone  = 0;
            printf("�����һ��׶Ͱ����ʼ������׮\r\n");
        }
        break;

    case COURSE2_STAGE_FORWARD:
        // ������׮��

        // ����Ѿ�����������׶Ͱ
        if (nav_sys.current_cone >= nav_sys.total_cones - 1)
        {
            // �����ͷ�׶�
            nav_sys.course2_stage = COURSE2_STAGE_TURN;
            nav_sys.initial_yaw   = nav_sys.current_yaw;
            printf("���������׮����ʼ��ͷ\r\n");
            break;
        }

        // ȷ����ǰĿ��׶Ͱλ��
        uint8_t next_cone = nav_sys.current_cone + 1;
        target_lat        = nav_sys.cones[next_cone][0];
        target_lon        = nav_sys.cones[next_cone][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // S��·�ߴ����߼����ӽ���һ��׶Ͱʱ�л�����
        if (dist_to_target < CONE_SWITCH_DIST)
        {
            nav_sys.current_cone     = next_cone;
            nav_sys.slalom_direction = !nav_sys.slalom_direction;   // �л���׮����
            printf("�ӽ�׶Ͱ %d���л����з���\r\n", next_cone);
        }
        break;

    case COURSE2_STAGE_TURN:
        // ��ͷ����
        // �������һ��׶Ͱ��λ����ΪĿ���
        target_lat = nav_sys.cones[nav_sys.total_cones - 1][0];
        target_lon = nav_sys.cones[nav_sys.total_cones - 1][1];

        // ���㺽��仯
        float yaw_change = normalize_angle(nav_sys.current_yaw - nav_sys.initial_yaw);
        if (yaw_change < 0) yaw_change += 360.0;

        // �����ɵ�ͷ(����仯������ֵ)
        if (yaw_change > TURN_ANGLE_THRESHOLD)
        {
            nav_sys.course2_stage    = COURSE2_STAGE_BACKWARD;
            nav_sys.current_cone     = nav_sys.total_cones - 1;   // �����һ��׶Ͱ��ʼ������׮
            nav_sys.slalom_direction = 0;                         // ������׮����
            printf("��ͷ��ɣ���ʼ������׮\r\n");
        }
        break;

    case COURSE2_STAGE_BACKWARD:
        // ������׮��

        // ����Ѿ��ص���һ��׶Ͱ
        if (nav_sys.current_cone <= 0)
        {
            // �����յ�׶�
            nav_sys.course2_stage = COURSE2_STAGE_END;
            printf("��ɷ�����׮��ǰ���յ�\r\n");
            break;
        }

        // ȷ����ǰĿ��׶Ͱλ��
        uint8_t prev_cone = nav_sys.current_cone - 1;
        target_lat        = nav_sys.cones[prev_cone][0];
        target_lon        = nav_sys.cones[prev_cone][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // S��·�ߴ����߼����ӽ�ǰһ��׶Ͱʱ�л�����
        if (dist_to_target < CONE_SWITCH_DIST)
        {
            nav_sys.current_cone     = prev_cone;
            nav_sys.slalom_direction = !nav_sys.slalom_direction;   // �л���׮����
            printf("�ӽ�׶Ͱ %d���л����з���\r\n", prev_cone);
        }
        break;

    case COURSE2_STAGE_END:
        // �����յ� (����ʹ�õ�һ��׶Ͱ�ľ���λ����Ϊ�յ�)
        target_lat = nav_sys.cones[0][0] * 2 - nav_sys.cones[1][0];   // �򵥹����յ�λ��
        target_lon = nav_sys.cones[0][1] * 2 - nav_sys.cones[1][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // ��������յ�
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course2_stage = COURSE2_STAGE_COMPLETE;
            nav_sys.course_state  = COURSE_STATE_COMPLETED;

            // ֹͣ����
            nav_sys.target_steer = 0;
            nav_sys.target_speed = 0;

            printf("�����յ㣬��Ŀ�����\r\n");
            return;
        }
        break;

    case COURSE2_STAGE_COMPLETE:
        // ����ɣ�����ֹͣ״̬
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        return;
    }

    // ���㵽Ŀ��ķ�λ��
    target_azimuth = get_two_points_azimuth(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

    // ���㺽�����
    heading_error = normalize_angle(target_azimuth - nav_sys.current_yaw);

    // ��Ŀ�������߼���S����׮��Ҫ���ݵ�ǰ�����׶Ͱλ�õ��������
    if (nav_sys.course2_stage == COURSE2_STAGE_FORWARD || nav_sys.course2_stage == COURSE2_STAGE_BACKWARD)
    {
        // ��������ǣ�ʹ������׶Ͱ��ָ��������
        if (nav_sys.slalom_direction == 0)   // �������
        {
            heading_error -= 30.0;   // ����ƫתһ���Ƕ�
        }
        else   // �Ҳ�����
        {
            heading_error += 30.0;   // ����ƫתһ���Ƕ�
        }
    }

    // ����ת�������
    nav_sys.target_steer = heading_error * STEER_GAIN;

    // ����ת����
    if (nav_sys.target_steer > MAX_STEER) nav_sys.target_steer = MAX_STEER;
    if (nav_sys.target_steer < -MAX_STEER) nav_sys.target_steer = -MAX_STEER;

    // �����ٶ�
    if (nav_sys.course2_stage == COURSE2_STAGE_TURN)
    {
        // ��ͷ�׶�ʹ��ת���ٶ�
        nav_sys.target_speed = TURNING_SPEED;
    }
    else if (nav_sys.course2_stage == COURSE2_STAGE_FORWARD || nav_sys.course2_stage == COURSE2_STAGE_BACKWARD)
    {
        // ��׮�׶�ʹ����׮�ٶ�
        nav_sys.target_speed = SLALOM_SPEED;
    }
    else if (fabs(heading_error) > 45.0)
    {
        // ��Ƕ�ת�򣬽����ٶ�
        nav_sys.target_speed = TURNING_SPEED;
    }
    else if (dist_to_target < 5.0)
    {
        // �ӽ�Ŀ��㣬�����ٶ�
        nav_sys.target_speed = SLOW_SPEED;
    }
    else
    {
        // �����ٶ�
        nav_sys.target_speed = NORMAL_SPEED;
    }
}

/**
 * @brief ��⵱ǰ·��Ԫ������
 * @param lat ��ǰγ��
 * @param lon ��ǰ����
 * @return ·��Ԫ������
 */
static uint8_t detect_road_element(double lat, double lon)
{
    // ������Ը���ʵ�����ʵ��·��Ԫ�صļ���߼�
    // ���磬����Ԥ�ȶ���һЩ�µ��͵���·�ε�����

    // ��ʾ���������ض��������µ������·��
    // �µ�����
    if (lat > 31.2306 && lat < 31.2308 && lon > 121.4739 && lon < 121.4741)
    {
        return ROAD_SLOPE;
    }

    // ��������
    if (lat > 31.2310 && lat < 31.2312 && lon > 121.4743 && lon < 121.4745)
    {
        return ROAD_BUMPY;
    }

    // Ĭ��Ϊ����·��
    return ROAD_NORMAL;
}

/**
 * @brief ���¿�Ŀ������
 */
static void update_course3(void)
{
    // ��Ŀ�������߼����Ŀһ���ƣ���������·��Ԫ��ʶ��
    double target_lat = 0.0, target_lon = 0.0;
    double dist_to_target = 0.0;

    // ��⵱ǰ·��Ԫ��
    nav_sys.current_road = detect_road_element(nav_sys.current_lat, nav_sys.current_lon);

    // ���ݵ�ǰ�׶�ѡ��Ŀ���
    switch (nav_sys.course3_stage)
    {
    case COURSE3_STAGE_START:
        // Ŀ���ǵ�ͷ���
        target_lat = nav_sys.waypoints[1][0];
        target_lon = nav_sys.waypoints[1][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // ��������ͷ���
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course3_stage = COURSE3_STAGE_TURN;
            nav_sys.initial_yaw   = nav_sys.current_yaw;   // ��¼�����ͷ���ĳ�ʼ����
            printf("�����ͷ��㣬��ʼ��ͷ\r\n");
        }
        break;

    case COURSE3_STAGE_TURN:
        // Ŀ���ǵ�ͷ������
        target_lat = nav_sys.waypoints[2][0];
        target_lon = nav_sys.waypoints[2][1];

        // ���㺽��仯
        float yaw_change = normalize_angle(nav_sys.current_yaw - nav_sys.initial_yaw);
        if (yaw_change < 0) yaw_change += 360.0;

        // �����ɵ�ͷ(����仯������ֵ)
        if (yaw_change > TURN_ANGLE_THRESHOLD)
        {
            nav_sys.course3_stage = COURSE3_STAGE_RETURN;
            printf("��ͷ��ɣ���ʼ�����յ�\r\n");
        }
        break;

    case COURSE3_STAGE_RETURN:
        // Ŀ�����յ�
        target_lat = nav_sys.waypoints[3][0];
        target_lon = nav_sys.waypoints[3][1];

        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

        // ��������յ�
        if (dist_to_target < WAYPOINT_RADIUS)
        {
            nav_sys.course3_stage = COURSE3_STAGE_COMPLETE;
            nav_sys.course_state  = COURSE_STATE_COMPLETED;

            // ֹͣ����
            nav_sys.target_steer = 0;
            nav_sys.target_speed = 0;

            printf("�����յ㣬��Ŀ�����\r\n");
            return;
        }
        break;

    case COURSE3_STAGE_COMPLETE:
        // ����ɣ�����ֹͣ״̬
        nav_sys.target_steer = 0;
        nav_sys.target_speed = 0;
        return;
    }

    // ���㵽Ŀ���ķ�λ��
    double target_azimuth = get_two_points_azimuth(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

    // ���㺽�����
    double heading_error = normalize_angle(target_azimuth - nav_sys.current_yaw);

    // �������(���ڷǵ�ͷ�׶�ʹ��)
    if (nav_sys.course3_stage != COURSE3_STAGE_TURN)
    {
        dist_to_target = get_two_points_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);
    }

    // ����ת�������
    nav_sys.target_steer = heading_error * STEER_GAIN;

    // ����ת����
    if (nav_sys.target_steer > MAX_STEER) nav_sys.target_steer = MAX_STEER;
    if (nav_sys.target_steer < -MAX_STEER) nav_sys.target_steer = -MAX_STEER;

    // ���ݵ�ǰ·��Ԫ�غͽ׶������ٶ�
    if (nav_sys.current_road == ROAD_SLOPE)
    {
        // �µ��ٶ�
        nav_sys.target_speed = SLOPE_SPEED;
        printf("��⵽�µ��������ٶ�\r\n");
    }
    else if (nav_sys.current_road == ROAD_BUMPY)
    {
        // ����·���ٶ�
        nav_sys.target_speed = BUMPY_SPEED;
        printf("��⵽����·�Σ������ٶ�\r\n");
    }
    else if (nav_sys.course3_stage == COURSE3_STAGE_TURN)
    {
        // ��ͷ�׶�ʹ��ת���ٶ�
        nav_sys.target_speed = TURNING_SPEED;
    }
    else if (fabs(heading_error) > 45.0)
    {
        // ��Ƕ�ת�򣬽����ٶ�
        nav_sys.target_speed = TURNING_SPEED;
    }
    else if (dist_to_target < 5.0)
    {
        // �ӽ�Ŀ��㣬�����ٶ�
        nav_sys.target_speed = SLOW_SPEED;
    }
    else
    {
        // �����ٶ�
        nav_sys.target_speed = NORMAL_SPEED;
    }
}