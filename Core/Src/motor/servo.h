/*
 * servo.h
 *
 *  Created on: 2026年9月15日
 *      Author: Huang
 */

#ifndef SRC_MOTOR_SERVO_H_
#define SRC_MOTOR_SERVO_H_

typedef enum
{
    SERVO_INIT = 0,
    SERVO_IPROPI_CAL,
    SERVO_READY,
    SERVO_RUN
} ServoState_t;

void Servo_Init();

typedef struct ServoRuntime_ {
	float    angle;
	float    speed;
	float    current;
	float    vm_vol;
	int32_t  pwmset;

	uint8_t  state;
	int8_t   cursign;
	uint8_t  drive_mode;
	uint8_t  operation_mode;

	int32_t  current_mA;
	int32_t  home_offset;

	int32_t  min_pos;
	int32_t  max_pos;

	uint16_t raw_angle;
	uint16_t vm_bus_raw;

	int32_t vel_q8;
}ServoRuntime_t;

#define DYNA_VEL_UNIT 0.0239691227f

void Servo_Current_PI_Update();
void Servo_Velocity_PI_Update();

extern volatile ServoRuntime_t servo_rt;

extern volatile uint8_t current_ready;
extern volatile float   goal_current;

void Servo_MainLoopSync(void);

void Servo_Position_limit_Update();
#endif /* SRC_MOTOR_SERVO_H_ */
