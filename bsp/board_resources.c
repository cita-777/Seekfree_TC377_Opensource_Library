/**
 * *****************************************************************************
 * @file        board_resources.c
 * @brief       ������Դ
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
uint8 key1_state = 1;   // ��������״̬
uint8 key2_state = 1;   // ��������״̬
uint8 key3_state = 1;   // ��������״̬
uint8 key4_state = 1;   // ��������״̬

uint8 switch1_state = 0;   // ���뿪�ض���״̬
uint8 switch2_state = 0;   // ���뿪�ض���״̬

uint8 key1_state_last = 0;   // ��һ�ΰ�������״̬
uint8 key2_state_last = 0;   // ��һ�ΰ�������״̬
uint8 key3_state_last = 0;   // ��һ�ΰ�������״̬
uint8 key4_state_last = 0;   // ��һ�ΰ�������״̬

uint8 switch1_state_last = 0;   // ��һ�β��뿪�ض���״̬
uint8 switch2_state_last = 0;   // ��һ�β��뿪�ض���״̬

uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;

uint8 key_val;


uint8 switch1_flag;
uint8 switch2_flag;

uint16 key1_count = -1;
uint16 key2_count = -1;
uint16 key3_count = -1;
uint16 key4_count = -1;

uint16 switch1_count = -1;
uint16 switch2_count = -1;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
//===================================================������===================================================
void Buzzer_init()   // ��������ʼ��
{
    gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);
}

void Buzzer_check(int time1, int time2)   // ���������Լ캯��
{
    gpio_set_level(BUZZER_PIN, 1);
    system_delay_ms(time1);
    gpio_set_level(BUZZER_PIN, 0);
    system_delay_ms(time2);
}
//===================================================������LED===================================================


void Key_init()   // ������LED��ʼ��
{

    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ
    gpio_init(LED2, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // ��ʼ�� LED2 ��� Ĭ�ϸߵ�ƽ �������ģʽ
    gpio_init(LED3, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // ��ʼ�� LED3 ��� Ĭ�ϸߵ�ƽ �������ģʽ
    gpio_init(LED4, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // ��ʼ�� LED4 ��� Ĭ�ϸߵ�ƽ �������ģʽ

    gpio_init(KEY1, GPI, GPIO_LOW, GPI_PULL_UP);    // ��ʼ�� KEY1 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);   // ��ʼ�� KEY2 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);   // ��ʼ�� KEY3 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);   // ��ʼ�� KEY4 ���� Ĭ�ϸߵ�ƽ ��������

    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_FLOATING_IN);   // ��ʼ�� SWITCH1 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_FLOATING_IN);   // ��ʼ�� SWITCH2 ���� Ĭ�ϸߵ�ƽ ��������
}

void key_scan()   // ����ɨ��
{
    // ʹ�ô˷����ŵ����ڣ�����Ҫʹ��while(1) �ȴ������⴦������Դ�˷�

    // ���水��״̬
    key1_state_last = key1_state;
    key2_state_last = key2_state;
    key3_state_last = key3_state;
    key4_state_last = key4_state;

    // ��ȡ��ǰ����״̬
    key1_state = gpio_get_level(KEY1);
    key2_state = gpio_get_level(KEY2);
    key3_state = gpio_get_level(KEY3);
    key4_state = gpio_get_level(KEY4);


    // ��⵽��������֮��  ���ſ���λ��־λ
    if (key1_state && !key1_state_last)
    {
        key1_flag = 1;
    }
    if (key2_state && !key2_state_last)
    {
        key2_flag = 1;
    }
    if (key3_state && !key3_state_last)
    {
        key3_flag = 1;
    }
    if (key4_state && !key4_state_last)
    {
        key4_flag = 1;
    }

    // ��־λ��λ֮�󣬿���ʹ�ñ�־λִ���Լ���Ҫ�����¼�

    //        system_delay_ms(10);//��ʱ����������Ӧ�ñ�֤����ʱ�䲻С��10ms
}
/*------------------------------------test------------------------------------*/
