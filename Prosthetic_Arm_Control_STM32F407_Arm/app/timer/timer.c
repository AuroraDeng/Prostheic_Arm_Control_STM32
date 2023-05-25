#include "timer.h"

volatile uint32_t g_tickcount = 0; // SysTick������

// ��ʼ��ʱ
void start_timer()
{
    // ����SysTick��ʱ��
    SysTick_Config(SystemCoreClock / 1000); // 1ms�ж�һ��
}

// ֹͣ��ʱ����������ʱ�䣨��λ�����룩
uint32_t stop_timer()
{
    // �ر�SysTick��ʱ��
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    // ��ȡ��������ֵ
    uint32_t ticks = g_tickcount;

    // ��������ʱ��
    uint32_t time_ms = ticks;

    return time_ms;
}