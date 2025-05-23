/**
 * *****************************************************************************
 * @file        board_resources.c
 * @brief       板载资源
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-26
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "zf_common_headfile.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
uint8 key1_state = 1;   // 按键动作状态
uint8 key2_state = 1;   // 按键动作状态
uint8 key3_state = 1;   // 按键动作状态
uint8 key4_state = 1;   // 按键动作状态

uint8 switch1_state = 0;   // 拨码开关动作状态
uint8 switch2_state = 0;   // 拨码开关动作状态

uint8 key1_state_last = 0;   // 上一次按键动作状态
uint8 key2_state_last = 0;   // 上一次按键动作状态
uint8 key3_state_last = 0;   // 上一次按键动作状态
uint8 key4_state_last = 0;   // 上一次按键动作状态

uint8 switch1_state_last = 0;   // 上一次拨码开关动作状态
uint8 switch2_state_last = 0;   // 上一次拨码开关动作状态

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
//===================================================蜂鸣器===================================================
void Buzzer_init()   // 蜂鸣器初始化
{
    gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);
}

void Buzzer_check(int time1, int time2)   // 蜂鸣器的自检函数
{
    gpio_set_level(BUZZER_PIN, 1);
    system_delay_ms(time1);
    gpio_set_level(BUZZER_PIN, 0);
    system_delay_ms(time2);
}
//===================================================按键与LED===================================================


void Key_init()   // 按键与LED初始化
{

    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // 初始化 LED1 输出 默认高电平 推挽输出模式
    gpio_init(LED2, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // 初始化 LED2 输出 默认高电平 推挽输出模式
    gpio_init(LED3, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // 初始化 LED3 输出 默认高电平 推挽输出模式
    gpio_init(LED4, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // 初始化 LED4 输出 默认高电平 推挽输出模式

    gpio_init(KEY1, GPI, GPIO_LOW, GPI_PULL_UP);    // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);   // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);   // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);   // 初始化 KEY4 输入 默认高电平 上拉输入

    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_FLOATING_IN);   // 初始化 SWITCH1 输入 默认高电平 浮空输入
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_FLOATING_IN);   // 初始化 SWITCH2 输入 默认高电平 浮空输入
}

void key_scan()   // 按键扫描
{
    // 使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费

    // 保存按键状态
    key1_state_last = key1_state;
    key2_state_last = key2_state;
    key3_state_last = key3_state;
    key4_state_last = key4_state;

    // 读取当前按键状态
    key1_state = gpio_get_level(KEY1);
    key2_state = gpio_get_level(KEY2);
    key3_state = gpio_get_level(KEY3);
    key4_state = gpio_get_level(KEY4);


    // 检测到按键按下之后  并放开置位标志位
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

    // 标志位置位之后，可以使用标志位执行自己想要做的事件

    //        system_delay_ms(10);//延时，按键程序应该保证调用时间不小于10ms
}
/*------------------------------------test------------------------------------*/
