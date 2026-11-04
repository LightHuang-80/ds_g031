/*
 * motor_control.h
 *
 *  Created on: 2026年9月15日
 *      Author: Huang
 */

#ifndef SRC_MOTOR_MOTOR_CONTROL_H_
#define SRC_MOTOR_MOTOR_CONTROL_H_

#include "main.h"
#include <stdint.h>

#define PWM_ARR                 1599.0f

#define PWM_DUTY_MAX            0.998f
#define PWM_DUTY_MIN            0.1f

#define CURRENT_PI_MAX          1.0f
#define CURRENT_PI_MIN          0.001f

#define PWM_FREQ_HZ             20000.0f
#define CURRENT_LOOP_DIV        4
#define CURRENT_LOOP_DT         (CURRENT_LOOP_DIV / PWM_FREQ_HZ)   // 0.0002f

#define ADC_FULL_SCALE          4095.0f
#define ADC_VREF                3.3f

/* IPROPI */
#define IPROPI_MIRROR           0.001575f     // uA/A
#define IPROPI_R                1500.0f       // ohm

/* ------------------------------------------------------------------ */
/*  ADC -> mA                                                          */
/* ------------------------------------------------------------------ */
#define IPROPI_ADC_VREF_V     3.3f
#define IPROPI_ADC_FS         4096.0f
#define IPROPI_V_PER_A        2.3625f      /* IPROPI_MIRROR * IPROPI_R */

#define IPROPI_MA_PER_LSB   (IPROPI_ADC_VREF_V * 1000.0f / IPROPI_ADC_FS / IPROPI_V_PER_A)
#define IPROPI_MA_Q16       ((int32_t)(IPROPI_MA_PER_LSB * 65536.0f + 0.5f))   /* 编译期常量 */

#define CURRENT_PI_Kp         0.15f
#define CURRENT_PI_Ki         15.0f

typedef struct CurrentPI_
{
    float kp;
    float ki;

    float integral;

    float output_min;
    float output_max;

} CurrentPI_t;

extern CurrentPI_t current_pi;


/* ------------------------------------------------------------------ */
/*  定点 PI                                                            */
/* ------------------------------------------------------------------ */
typedef struct {
    int32_t kp_q32;          /* (FS / mA) * 2^32           */
    int32_t ki_dt_q36;       /* (FS / mA / step) * 2^36    */
    int64_t integ_q36;       /* 积分状态, FS = 2^36        */
    int64_t integ_max_q36;   /* 积分限幅                   */
} cur_pi_q_t;


/* VBus divider
 * 这里一定需要根据你实际的电阻分压修改
 */
#define VBUS_R_TOP              47000.0f
#define VBUS_R_BOTTOM           10000.0f

void IPROPI_Init();
uint8_t IPROPI_Calibrate();

float IPROPI_To_Current(uint16_t adc);
float ADC_To_VBus(uint16_t adc);

float Current_PI_Update(float current_target, float current_actual);

void Motor_Output_Disable(void);
void Motor_SetPWM_Duty(float duty);
void Motor_SetPWM_Q16(int32_t duty_q16);

/*
 * flt_q4     : 滤波后的 ADC 值 (Q4)
 * offset_q4  : 零电流偏置 (Q4)
 * 返回       : 电流幅值 mA (>= 0)
 * 溢出检查   : 65520 * 22349 = 1.46e9 < 2^31
 */
static inline int32_t cur_adc_to_mA(int32_t flt_q4, int32_t offset_q4)
{
    int32_t d = flt_q4 - offset_q4;
    if (d < 0) d = 0;
    return (d * IPROPI_MA_Q16 + (1 << 19)) >> 20;
}

#endif /* SRC_MOTOR_MOTOR_CONTROL_H_ */
