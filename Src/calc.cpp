
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include "../DSP/Include/arm_math.h"

#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef hlpuart1;

float i = 0;
float m_pi = 3.141592653589793f;
extern RNG_HandleTypeDef hrng;

static const size_t buff_size = 1000; // calculate continous

static volatile float rslt_buff[buff_size];
static volatile float ix_buff[buff_size];

extern int get_time_ms();
extern uint64_t get_usec();

char send[128];

static void output_elapse(const char name[], const float start_usec, const size_t times)
{
    auto now = get_usec();
    auto elapse = now - start_usec;
    auto usec = (float)elapse / static_cast<float>(times);
    snprintf(send, sizeof(send), "%s : %.2fusec  (%.2fmsec)\n", name, usec, elapse / 1000.f);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)send, strlen(send), 500);
}

void calc()
{

    // == output compiler name ==============================================
#ifdef __GNUC__
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)"GCC : ===\n", 10, 500);
#else
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)"IAR : ===\n", 10, 500);
#endif /* __GNUC__ */

    // == check elapse time ==============================================
    auto start = get_usec();
    HAL_Delay(500);
    output_elapse("check 500msec", start, 1);

    for (size_t i = 0; i < buff_size; i++)
    {
        uint32_t ix;
        HAL_RNG_GenerateRandomNumber(&hrng, &ix);
        ix_buff[i] = static_cast<float>(ix) / static_cast<float>(0x80000000);
    }

    // == sin =============================================================
    start = get_usec();
    for (size_t i = 0; i < buff_size; i++)
    {
        rslt_buff[i] = sinf(ix_buff[i]);
    }
    output_elapse("sin", start, buff_size);

    // == CMSIS sin =============================================================
    start = get_usec();
    for (size_t i = 0; i < buff_size; i++)
    {
        rslt_buff[i] = arm_sin_f32(ix_buff[i]);
    }
    output_elapse("cmsis sin", start, buff_size);
}
