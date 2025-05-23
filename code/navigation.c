/**
 * *****************************************************************************
 * @file        navigation.c
 * @brief       ����ϵͳʵ�֣��򻯰汾����Ҫ�������ܳ���Ŀ����
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-04-04
 * @copyright   cita
 * *****************************************************************************
 */

#include "navigation.h"
#include "zf_common_headfile.h"

/*----------------------------------�궨��-----------------------------------*/
// ����ģʽ����
#define NAV_MODE_GPS_ONLY 0   // ��ʹ��GPSģʽ
#define NAV_MODE_IMU_ONLY 1   // ��ʹ��IMUģʽ
#define NAV_MODE_MANUAL 2     // �ֶ�ģʽ

// ����뾶���ף�
#define EARTH_RADIUS 6378137.0
// ���ȽǶ�ת��
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886

// ����ϵ����
#define MAX_WAYPOINTS 4       // ���·��������
#define WAYPOINT_RADIUS 2.0   // �����л��뾶(��)

// ��Ŀ״̬����
#define COURSE_STATE_IDLE 0        // δ����
#define COURSE_STATE_RUNNING 1     // ��������
#define COURSE_STATE_COMPLETED 2   // �����

// ��Ŀһ�׶ζ���
#define COURSE1_STAGE_START 0      // ��㵽��ͷ���
#define COURSE1_STAGE_TURN 1       // ��ͷ��
#define COURSE1_STAGE_RETURN 2     // ��ͷ�����㵽�յ�
#define COURSE1_STAGE_COMPLETE 3   // ���

// ��������
#define TURN_ANGLE_THRESHOLD 165.0   // ��ͷ��ɵĽǶ���ֵ(��)
#define STEER_GAIN 1.0               // ת������
#define MAX_STEER 35.0               // ���ת��Ƕ�
#define NORMAL_SPEED 100             // �����ٶ�
#define TURNING_SPEED 60             // ת���ٶ�
#define SLOW_SPEED 40                // ����

/*----------------------------------���Ͷ���----------------------------------*/
// ����ϵͳ�ṹ��
typedef struct
{
    uint8_t mode;                          // ����ģʽ
    uint8_t current_waypoint;              // ��ǰ��������
    double  waypoints[MAX_WAYPOINTS][2];   // �������� [γ��, ����]

    // ��ǰλ�úͺ���
    double current_lat;   // ��ǰγ��
    double current_lon;   // ��ǰ����
    float  current_yaw;   // ��ǰ�����(��)

    // �������
    double  target_steer;   // Ŀ��ת���
    int16_t target_speed;   // Ŀ���ٶ�

    // ��Ŀ��ز���
    uint8_t course_state;    // ��Ŀ״̬
    uint8_t course1_stage;   // ��Ŀһ�׶�
    float   initial_yaw;     // ��ʼ�����(���ڵ�ͷ�ж�)
} NavSystem;

/*----------------------------------ȫ�ֱ���----------------------------------*/
static NavSystem nav_sys;   // ����ϵͳʵ��

/*----------------------------------˽�к���----------------------------------*/
// ������������
static double calculate_distance(double lat1, double lon1, double lat2, double lon2);
static double calculate_azimuth(double lat1, double lon1, double lat2, double lon2);
static double normalize_angle(double angle);

/*--------------------------------�����ӿں���--------------------------------*/
/**
 * @brief ��ʼ������ϵͳ
 */
void navigation_init(void)
{
    // ��ʼ��ϵͳ����
    nav_sys.mode             = NAV_MODE_GPS_ONLY;
    nav_sys.current_waypoint = 0;
    nav_sys.target_steer     = 0.0;
    nav_sys.target_speed     = 0;

    // ��ʼ����Ŀ��ز���
    nav_sys.course_state  = COURSE_STATE_IDLE;
    nav_sys.course1_stage = COURSE1_STAGE_START;
    nav_sys.initial_yaw   = 0.0f;

    // ��ʼ��Ĭ�Ͽ�Ŀ��λ(��ͨ�������ɼ�����)
    // Ĭ�Ϻ��� - ���ڲɼ�ʱ����
    nav_sys.waypoints[0][0] = 31.2304;    // ���γ��
    nav_sys.waypoints[0][1] = 121.4737;   // ��㾭��
    nav_sys.waypoints[1][0] = 31.2314;    // ��ͷ���γ��
    nav_sys.waypoints[1][1] = 121.4747;   // ��ͷ��㾭��
    nav_sys.waypoints[2][0] = 31.2312;    // ��ͷ������γ��
    nav_sys.waypoints[2][1] = 121.4745;   // ��ͷ�����㾭��
    nav_sys.waypoints[3][0] = 31.2304;    // �յ�γ��
    nav_sys.waypoints[3][1] = 121.4737;   // �յ㾭��

    printf("Navigation system initialized.\r\n");
}

/**
 * @brief �򻯵������º���
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

    // ��Ŀһ�����߼�
    double target_lat = 0.0, target_lon = 0.0;
    double dist_to_target = 0.0;

    // ���ݵ�ǰ�׶�ѡ��Ŀ���
    switch (nav_sys.course1_stage)
    {
    case COURSE1_STAGE_START:
        // Ŀ���ǵ�ͷ���
        target_lat = nav_sys.waypoints[1][0];
        target_lon = nav_sys.waypoints[1][1];

        dist_to_target = calculate_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

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

        dist_to_target = calculate_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

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
    double target_azimuth = calculate_azimuth(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);

    // ���㺽�����
    double heading_error = normalize_angle(target_azimuth - nav_sys.current_yaw);

    // �������(���ڷǵ�ͷ�׶�ʹ��)
    if (nav_sys.course1_stage != COURSE1_STAGE_TURN)
    {
        dist_to_target = calculate_distance(nav_sys.current_lat, nav_sys.current_lon, target_lat, target_lon);
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

    // �����������
    if (nav_sys.mode != NAV_MODE_MANUAL)
    {
        servo_set(SERVO_MOTOR_MID + (int16_t)nav_sys.target_steer);
        drv8701_motor_set(nav_sys.target_speed);
    }
}

/**
 * @brief �л�����ģʽ
 * @param mode Ŀ��ģʽ
 */
void navigation_set_mode(uint8_t mode)
{
    if (mode <= NAV_MODE_MANUAL)
    {
        nav_sys.mode = mode;
        printf("����ģʽ�л�Ϊ: %d\r\n", mode);
    }
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
    nav_sys.course1_stage    = COURSE1_STAGE_START;
    nav_sys.mode             = NAV_MODE_GPS_ONLY;
    nav_sys.current_waypoint = 0;

    printf("��Ŀһ��ʼ����\r\n");
}

/**
 * @brief ֹͣ��ǰ��Ŀ
 */
void navigation_stop_course(void)
{
    nav_sys.course_state = COURSE_STATE_IDLE;

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
 * @param point_index ��λ���� (0-���, 1-��ͷ���, 2-��ͷ������, 3-�յ�)
 * @param lat γ��
 * @param lon ����
 */
void navigation_collect_point(uint8_t point_index, double lat, double lon)
{
    if (point_index < MAX_WAYPOINTS)
    {
        nav_sys.waypoints[point_index][0] = lat;
        nav_sys.waypoints[point_index][1] = lon;

        printf("�ɼ���λ[%d]: γ�� %.6f, ���� %.6f\r\n", point_index, lat, lon);
    }
}

/**
 * @brief ���͵������ݵ���λ��
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

/*--------------------------------˽�к���ʵ��--------------------------------*/
/**
 * @brief ������������
 */
static double calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
    // ����γ��ת��Ϊ����
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lon1_rad = lon1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double lon2_rad = lon2 * DEG_TO_RAD;

    // ����ʸ��ʽ�������
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;
    double a    = sin(dlat / 2) * sin(dlat / 2) + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2) * sin(dlon / 2);
    double c    = 2 * atan2(sqrt(a), sqrt(1 - a));

    return EARTH_RADIUS * c;
}

/**
 * @brief ����ӵ�1����2�ķ�λ��
 */
static double calculate_azimuth(double lat1, double lon1, double lat2, double lon2)
{
    // ����γ��ת��Ϊ����
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lon1_rad = lon1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double lon2_rad = lon2 * DEG_TO_RAD;

    // ���㷽λ��
    double y           = sin(lon2_rad - lon1_rad) * cos(lat2_rad);
    double x           = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(lon2_rad - lon1_rad);
    double azimuth_rad = atan2(y, x);

    // ת��Ϊ��������ȷ����[0,360)��Χ��
    double azimuth_deg = azimuth_rad * RAD_TO_DEG;
    return (azimuth_deg < 0) ? (azimuth_deg + 360.0) : azimuth_deg;
}

/**
 * @brief ��׼���Ƕȵ�[-180, 180]��Χ
 */
static double normalize_angle(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}