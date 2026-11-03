/*
 * servo.c
 *
 *  Created on: 2026年9月15日
 *      Author: Huang
 */

#include <math.h>
#include "adc.h"
#include "tim.h"
#include "dma.h"

#include "as5600.h"
#include "angle.h"
#include "dynamixel.h"
#include "motor_control.h"
#include "config.h"
#include "motion.h"
#include "servo.h"

typedef struct {
    float    current;      // A,带符号
    float    speed;        // rad/s

    uint16_t raw_angle;    // 0~4095,已扣零点偏移
    uint16_t vbus_raw;
} servo_snap_t;

ServoState_t   servo_state;
volatile ServoRuntime_t servo_rt;
extern DMA_HandleTypeDef hdma_adc1;

volatile float   goal_current;

static void servo_snapshot(servo_snap_t *s)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();                     // 只拷几个字,不到 1µs
    s->current   = servo_rt.current;
    s->speed     = servo_rt.speed;

    s->raw_angle = (uint16_t)servo_rt.raw_angle;   // 需要 TIM2 回调里把 raw_angle 存进 servo_rt
    s->vbus_raw  = servo_rt.vm_bus_raw;    // 电流环回调里只存原始值
    __set_PRIMASK(pm);
}

void Servo_Init()
{
#ifndef DEBUG
	servo_state = SERVO_IPROPI_CAL;

	/* 电机输出关闭 */
	Motor_Output_Disable();

	/* 清零 */
	IPROPI_Init();

	/* Motion 参数更新*/
	Motion_Refresh();

	motion_profile_ready = 1;

	/* 开启 ADC DMA */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_DMA_LENGTH);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);

	/* 开启 TIM1 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	/* 计数器开始 */
	__HAL_TIM_ENABLE(&htim1);

	/*读取当前角度*/
	uint16_t raw_angle;
	float angle = 0.0f;
	if (!ANGLE_getRawAngle(&raw_angle)){
		angle = DXL_Position_To_Rad(raw_angle);
	}

  	servo_rt.angle = angle;
  	servo_rt.speed = 0.0f;

  	servo_rt.current = 0.0f;
  	servo_rt.state = 0;
#endif

}

void Servo_Current_PI_Update()
{
#ifndef DEBUG

	servo_rt.pwmset = Current_PI_Update(goal_current, servo_rt.current);
	if (servo_rt.pwmset > 0) {
		servo_rt.cursign = 1;
	}else{
		servo_rt.cursign = -1;
	}

	Motor_SetPWM_Duty(servo_rt.pwmset);

#endif
}

void Servo_Velocity_PI_Update()
{
#ifndef DEBUG
	/* 目标速度 */
	int32_t goalVelocity = ControlTable_ReadInt32(104);

	/* 目标速度转换成 rad/s */
	float fgoalVel = goalVelocity * DYNA_VEL_UNIT;

	goal_current = Velocity_PI_Update(fgoalVel, servo_rt.speed);

	// set the goal current
	int16_t goalCur = (int16_t)(goal_current * 1000.0f);

	// update the RAM table
	DXL_RAMTable[102] = GET_INT16_LSB(goalCur);
	DXL_RAMTable[103] = GET_INT16_MSB(goalCur);

#endif
}

void Servo_UpdateToTable(void)          // 主循环里调用
{
    static uint32_t last;
    static float vbus_f;

    uint32_t now = HAL_GetTick();
    if ((now - last) < 1) return;        // 1kHz 足够
    last = now;

    servo_snap_t s;
    servo_snapshot(&s);

    int16_t cur = (int16_t)lroundf(s.current * 1000.0f);
    DXL_RAMTable[126] = GET_INT16_LSB(cur);
    DXL_RAMTable[127] = GET_INT16_MSB(cur);

    ControlTable_WriteUInt32(s.raw_angle, 132);
    ControlTable_WriteInt32((int32_t)lroundf(s.speed / DYNA_VEL_UNIT), 128);

    vbus_f += 0.1f * (ADC_To_VBus(s.vbus_raw) - vbus_f);     // 一阶滤波,代替原来的平均
    uint16_t uvol = (uint16_t)(vbus_f * 10.0f);
    DXL_RAMTable[144] = GET_INT16_LSB(uvol);
    DXL_RAMTable[145] = GET_INT16_MSB(uvol);

	/* fpwm : 0% ~ 100% */
	uint16_t prezPWM = (int16_t)lroundf(servo_rt.pwmset * 100.0f);

	// Update the DXL_RAMTable
	DXL_RAMTable[124] = GET_INT16_LSB(prezPWM);
	DXL_RAMTable[125] = GET_INT16_MSB(prezPWM);

    if (!DXL_RAMTable[TORQUE_ENABLE_ADDR])
        ControlTable_WriteUInt32(s.raw_angle, 116);          // 力矩关闭时目标位置跟随
}

// ISR ADC sample complete
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
#ifndef DEBUG
	static uint8_t div;
	static float   last_current;

	if (hadc->Instance != ADC1) return;

	if (servo_state == SERVO_IPROPI_CAL){
    	if (!IPROPI_Calibrate()){
    	    // start current loop
    	    servo_state = SERVO_READY;
    	    return;
    	}
    }

    if (servo_state != SERVO_READY && servo_state != SERVO_RUN){
    	return;
    }

    // 计算电机电流
    float current = IPROPI_To_Current(adc_dma_buffer[1]);
    last_current  += 0.1f * (current - last_current);

    // Save the vm bus voltage
    servo_rt.vm_bus_raw = adc_dma_buffer[0];

	/* ADC 采集频率20KHz, 5KHz 电流环， 1KHz 电压采集*/
	/* 电流环采集频率 1/4 */
	if (div == CURRENT_LOOP_DIV){

		//5K Hz//
		servo_rt.current = (servo_rt.cursign < 0) ? -last_current : last_current;

		// 64	1	Torque Enable
		if (DXL_RAMTable[TORQUE_ENABLE_ADDR]){
			Servo_Current_PI_Update();
		}
	}

	if (++div > CURRENT_LOOP_DIV) div = 0;

#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t cnt = 0;
	uint16_t raw_angle;

    if (htim->Instance == TIM2){
    	float angle = 0.0f;

    	as5600_error_t err = ANGLE_getRawAngle(&raw_angle);
    	if (!err){
    		// 做零点偏移
    		if (raw_angle < servo_rt.home_offset){
    			raw_angle += 4096;
    		}

    		raw_angle -= servo_rt.home_offset;
    		if (raw_angle >= 4096){
    			raw_angle -= 4096;
    		}

    		// 获取角度, 单位: rad //
    		angle = DXL_Position_To_Rad(raw_angle);

    		// 换算成rad/s //
    		float delta_angle = (angle - servo_rt.angle);

    		if (delta_angle > DSQ_PI)
    		    delta_angle -= DSQ_2PI;

    		if (delta_angle < -DSQ_PI)
    		    delta_angle += DSQ_2PI;

    		// delta angle / 1ms，
    		// 1°/s = PI/180 rad/s = 0.01745329252,
    		// delta angle / 0.001 * 0.01745329252,
    		// servo_rt.speed 为真实速度 //
    		servo_rt.speed = delta_angle * 1000.0f;
    		servo_rt.angle = angle;
    		servo_rt.raw_angle = raw_angle;

    		/*
    		// 当前角度，度数转换成(0 ~ 4095) //
    		ControlTable_WriteUInt32(raw_angle, 132);

    		// 对于Torque disable //
            // 64	1	Torque Enable
            if (!DXL_RAMTable[TORQUE_ENABLE_ADDR]){
            	// Update goal position with present position
            	ControlTable_WriteUInt32(raw_angle, 116);
            }


    		// 当前速度, 转换成DXL单位 //
    		int32_t prezVelocity = (int32_t)roundf(servo_rt.speed / DYNA_VEL_UNIT);
    		ControlTable_WriteInt32(prezVelocity, 128);
    		*/

        	Servo_Velocity_PI_Update();
    	}

        /* 位置环分频 1/2 */
        if (++cnt >= 2){
        	cnt = 0;

        	// Position PID cycle
        	Position_Update();
        }
    }
}

