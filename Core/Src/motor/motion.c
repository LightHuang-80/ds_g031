/*
 * motion.c
 *
 *  Created on: 2026年9月22日
 *      Author: Huang
 */
#include <math.h>
#include <string.h>
#include "dynamixel.h"
#include "config.h"
#include "motor_control.h"
#include "servo.h"
#include "motion.h"
#include "velocity_loop_q.h"
#include "position_loop_q.h"

uint8_t position_ready;
uint8_t motion_profile_ready;

volatile  vel_pi_q_t      vel_pi;
volatile  pos_pid_q_t     pos_pid;

volatile  PositionPID_t   position_pid;
volatile  VelocityPI_t    velocity_pi;

MotionProfile_t motion_profile;
PositionFF_t    position_ff;

/* ---------------- Velocity ---------------- */
/*
 * XL330:
 *
 * 1 unit = 0.229 rev/min
 *
 * 1 rev = 2*pi rad
 * 1 min = 60 sec
 *
 * 0.229 * 2*pi / 60
 * = 0.023980? rad/s
 *
 * 但你当前采用的是 XL330 的实际协议换算值：
 * 0.0239691227 rad/s
 */
/* ---------------- Acceleration ---------------- */

/*
 * XL330:
 *
 * Profile Acceleration:
 * 1 unit = 214.577 rev/min^2
 *
 * 转换：
 *
 * 214.577 rev/min^2
 * × 2*pi rad/rev
 * ÷ 60^2 sec^2/min^2
 *
 * = rad/s^2
 */

float DXL_Position_To_Rad(uint32_t value)
{
    return ((float)value - DSQ_POSITION_ZERO)
           * DSQ_POSITION_SCALE_RAD;
}

uint32_t Rad_To_DXL_Position(float rad)
{
    float value;

    /* 限制到 Dynamixel 的位置范围 */
    if (rad <= -DSQ_PI)
        return 0;

    if (rad >= DSQ_PI)
        return 4095;

    value =
        rad * (2048.0f / DSQ_PI)
        + DSQ_POSITION_ZERO;

    /* 四舍五入 */
    if (value < 0.0f)
        value = 0.0f;

    if (value > 4095.0f)
        value = 4095.0f;

    return (uint32_t)(value + 0.5f);
}

float DXL_Velocity_To_RadS(int32_t value)
{
    return (float)value *
           DSQ_VELOCITY_UNIT_RAD_S;
}

int32_t RadS_To_DXL_Velocity(float rad_s)
{
    float value;

    value =
        rad_s / DSQ_VELOCITY_UNIT_RAD_S;

    /*
     * 四舍五入到最近的 Dynamixel unit
     */
    if (value >= 0.0f)
        return (int32_t)(value + 0.5f);
    else
        return (int32_t)(value - 0.5f);
}

float DXL_Acceleration_To_RadS2(uint32_t value)
{
    return (float)value *
           0.3745f;
}

uint32_t RadS2_To_DXL_Acceleration(float acceleration_rad_s2)
{
    float value;

    value =
        acceleration_rad_s2 /
        DXL_ACCEL_UNIT_RAD_S2;

    if (value <= 0.0f)
        return 0;

    return (uint32_t)(value + 0.5f);
}

void Motion_Vel_PI_Update()
{
	uint16_t vkp;
	// 78	2	Velocity P Gain
	MAKE_UINT16(vkp, DXL_RAMTable[78], DXL_RAMTable[79]);
	velocity_pi.kp = (float)(vkp) / 100.0f;

	uint16_t vki;
	// 76	2	Velocity I Gain
	MAKE_UINT16(vki, DXL_RAMTable[76], DXL_RAMTable[77]);
	velocity_pi.ki = (float)(vkp) / 100.0f;

	velocity_pi.integral = 0.0f;

	vel_pi_q_set_gains((vel_pi_q_t *)&vel_pi, velocity_pi.kp, velocity_pi.ki);
}

void Motion_Pos_PID_Update()
{
	// 80	2	Position D Gain
	uint16_t pkd;
	MAKE_UINT16(pkd, DXL_RAMTable[80], DXL_RAMTable[81]);
	position_pid.kd = (float)(pkd) / 100.0f;

	uint16_t pki;
	// 82	2	Position I Gain
	MAKE_UINT16(pki, DXL_RAMTable[82], DXL_RAMTable[83]);
	position_pid.ki = (float)(pki) / 100.0f;

	uint16_t pkp;
	// 84	2	Position I Gain
	MAKE_UINT16(pkp, DXL_RAMTable[84], DXL_RAMTable[85]);
	position_pid.kp = (float)(pkp) / 100.0f;

	position_pid.integral = 0.0f;

	pos_pid_q_set_gains((pos_pid_q_t *)&pos_pid, position_pid.kp, position_pid.ki, position_pid.kd,
			position_ff.ff1, position_ff.ff2, motion_profile.max_acceleration);
}

void Motion_Profile_Update()
{
#ifndef DEBUG
	// Feedforward 1st Gain
	int16_t  fgain1;
	MAKE_UINT16(fgain1, DXL_RAMTable[90], DXL_RAMTable[91]);

	// Feedforward 2nd Gain
	int16_t  fgain2;
	MAKE_UINT16(fgain2, DXL_RAMTable[88], DXL_RAMTable[89]);

	position_ff.ff1 = (float)(fgain1) / 100.0f;
	position_ff.ff2 = (float)(fgain2) / 100.0f;

	// 112	4	Profile Velocity
	float prof_acce = (float)ControlTable_ReadUInt32(108);
	prof_acce /= 100.0f;
	motion_profile.max_acceleration = DXL_Acceleration_To_RadS2(prof_acce);

	// 108	4	Profile Acceleration
	float prof_vel = (float)ControlTable_ReadUInt32(112);
	prof_vel /= 100.0f;
	motion_profile.max_velocity = DXL_Velocity_To_RadS(prof_vel);

#endif
}

void Motion_goPosition()
{

	uint32_t goal_position;

	// get the goal position, UNIT: rad
	float prez_position;
	prez_position = servo_rt.angle;

    goal_position    = ControlTable_ReadUInt32(116);
    if (goal_position < servo_rt.min_pos){
    	goal_position = servo_rt.min_pos;
    }else if (goal_position > servo_rt.max_pos){
    	goal_position = servo_rt.max_pos;
    }

#ifndef DEBUG
	motion_profile.position = prez_position;
	motion_profile.target_position  = DXL_Position_To_Rad(goal_position);

	motion_profile.velocity = servo_rt.speed;
	motion_profile.acceleration = 0.0f;
#endif
}

