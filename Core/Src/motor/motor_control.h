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

typedef struct CurrentPI_
{
    float kp;
    float ki;

    float integral;

    float output_min;
    float output_max;

} CurrentPI_t;

extern CurrentPI_t current_pi;

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

#endif /* SRC_MOTOR_MOTOR_CONTROL_H_ */
