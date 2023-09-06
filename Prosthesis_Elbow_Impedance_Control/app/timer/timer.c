#include "timer.h"

volatile uint32_t g_tickcount = 0; // SysTick计数器

// 开始计时
void start_timer()
{
    // 配置SysTick定时器
    SysTick_Config(SystemCoreClock / 1000); // 1ms中断一次
}

// 停止计时，返回运行时间（单位：毫秒）
uint32_t stop_timer()
{
    // 关闭SysTick定时器
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    // 获取计数器的值
    uint32_t ticks = g_tickcount;

    // 计算运行时间
    uint32_t time_ms = ticks;

    return time_ms;
}