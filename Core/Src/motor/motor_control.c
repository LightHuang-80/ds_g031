#include <stdio.h>

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "adc.h"
#include "tim.h"
#include "servo.h"
#include "motor_control.h"

CurrentPI_t current_pi = {
		0.15f,
		15.0f,
		0.0f,
		-1.4f,
        1.4f
};

static float ipropi_offset = 0.0f;
uint8_t  ipropi_calibrating = 0;
float    ipropi_sum;
uint16_t ipropi_count = 0;

void IPROPI_Init()
{
	ipropi_sum = 0.0f;
	ipropi_count = 0;
}

uint8_t IPROPI_Calibrate()
{
	if (ipropi_calibrating){
        ipropi_sum += (float)adc_dma_buffer[1];
        ipropi_count ++;

	    if (ipropi_count >= 500){
	        ipropi_offset = ipropi_sum / (float)ipropi_count;
	        ipropi_calibrating = 0;
	    }

	    return 1;
	}

	return 0;
}

float IPROPI_To_Current(uint16_t adc)
{
    float adc_corrected = adc;
    adc_corrected -= ipropi_offset;

    if (adc_corrected < 0)
        adc_corrected = 0.0f;

    return (float)adc_corrected * (0.0008056640625f / 2.3625); // ADC_VREF / ADC_FULL_SCALE
}

float ADC_To_VBus(uint16_t adc)
{
	// ((VBUS_R_TOP + VBUS_R_BOTTOM) / VBUS_R_BOTTOM);
    return (float)((float)(adc) * 0.004592f); // ADC_VREF) /  ADC_FULL_SCALE;
}

/* ======================================================================== *
 * =====================  Servo 多环控制 ==================================
 * ======================================================================== */

float Current_PI_Update(float current_target, float current_actual)
{
    float error;
    float output;

    error = current_target - current_actual;

    // 积分
    current_pi.integral +=
        current_pi.ki *
        error *
        CURRENT_LOOP_DT;

    // 积分限幅
    if (current_pi.integral > CURRENT_PI_MAX)
    	current_pi.integral = CURRENT_PI_MAX;

    if (current_pi.integral < -CURRENT_PI_MAX)
    	current_pi.integral = -CURRENT_PI_MAX;

    // PI
    output =
    		current_pi.kp * error +
			current_pi.integral;

    // 输出限幅
    if (output > PWM_DUTY_MAX)
        output = PWM_DUTY_MAX;

    if (output < -PWM_DUTY_MAX)
        output = -PWM_DUTY_MAX;

    return output;
}

void Motor_Output_Disable(void)
{
    __HAL_TIM_SET_COMPARE(&htim1,
                          TIM_CHANNEL_1,
                          0);

    __HAL_TIM_SET_COMPARE(&htim1,
                          TIM_CHANNEL_4,
                          0);
}

void Motor_SetPWM_Duty(float duty)
{
    uint32_t ccr;

    /*
     * 1. 限制输入范围
     *
     * duty:
     * +1.0  = 正方向100%
     *  0.0  = 停止
     * -1.0  = 反方向100%
     */
    if (duty > PWM_DUTY_MAX){
        duty = PWM_DUTY_MAX;
    }else if (duty < -PWM_DUTY_MAX){
        duty = -PWM_DUTY_MAX;
    }


    /*
     * 2. 判断停止
     */
    if (fabsf(duty) < 0.0001f){
    	Motor_Output_Disable();

        return;
    }


    /*
     * 3. 计算CCR
     *
     * 中心对齐：
     *
     * ARR = 1599
     *
     * duty = 0.5
     *
     * CCR = 800左右
     */
    ccr = (uint32_t)(fabsf(duty) * (float)PWM_ARR);


    /*
     * 4. 防止超过ARR
     */
    if (ccr > PWM_ARR){
        ccr = (uint32_t)PWM_ARR;
    }


    /*
     * 5. 判断方向
     */
    if (servo_rt.drive_mode){
    	if (duty > 0.0f) {
    		/* 正方向：IN1 常高，IN2 用反相 PWM（高占空比 = 多刹车 = 低平均电压） */
    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(PWM_ARR));          // IN1 = 100%
    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(PWM_ARR - ccr));    // IN2 = 反相 PWM
    	} else {
    		/* 反方向：IN2 常高，IN1 用反相 PWM */
    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(PWM_ARR - ccr));    // IN1 = 反相 PWM
    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(PWM_ARR));          // IN2 = 100%
    	}
    }else{
    	if (duty > 0.0f) {
    		/* 正方向：IN1 常高，IN2 用反相 PWM（高占空比 = 多刹车 = 低平均电压） */
    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(PWM_ARR));          // IN1 = 100%
    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(PWM_ARR - ccr));    // IN2 = 反相 PWM
    	} else {
    		/* 反方向：IN2 常高，IN1 用反相 PWM */
    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(PWM_ARR - ccr));    // IN1 = 反相 PWM
    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(PWM_ARR));          // IN2 = 100%
    	}
    }
}
