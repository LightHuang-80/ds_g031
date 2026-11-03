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

uint8_t position_ready;
uint8_t motion_profile_ready;

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
#define DXL_ACCEL_UNIT_REV_MIN2       214.577f

#define DXL_ACCEL_UNIT_RAD_S2         \
    (DXL_ACCEL_UNIT_REV_MIN2 * 2.0f * DSQ_PI / 3600.0f)

#define VELOCITY_LOOP_DT      0.001f      // 1ms
#define CURRENT_LIMIT         1.2f       // 最大目标电流，按你的实际需求修改

#define POSITION_DEADBAND  0.002f
#define MAX_VELOCITY       7.2f

MotionProfile_t motion_profile;
PositionPID_t   position_pid;
PositionFF_t    position_ff;

VelocityPI_t    velocity_pi;

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

void Motion_Refresh()
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

	uint16_t vkp;
	// 78	2	Velocity P Gain
	MAKE_UINT16(vkp, DXL_RAMTable[78], DXL_RAMTable[79]);
	velocity_pi.kp = (float)(vkp) / 100.0f;

	uint16_t vki;
	// 76	2	Velocity I Gain
	MAKE_UINT16(vki, DXL_RAMTable[76], DXL_RAMTable[77]);
	velocity_pi.ki = (float)(vkp) / 100.0f;

	// 112	4	Profile Velocity
	float prof_acce = (float)ControlTable_ReadUInt32(108);
	prof_acce /= 100.0f;
	motion_profile.max_acceleration = DXL_Acceleration_To_RadS2(prof_acce);

	// 108	4	Profile Acceleration
	float prof_vel = (float)ControlTable_ReadUInt32(112);
	prof_vel /= 100.0f;
	motion_profile.max_velocity = DXL_Velocity_To_RadS(prof_vel);

	velocity_pi.integral = 0.0f;
	position_pid.integral = 0.0f;

#endif
}

float Velocity_PI_Update(float target_velocity_rad_s,
                         float actual_velocity_rad_s)
{
    float error;
    float output;

#ifndef DEBUG
    /* 速度误差 */
    error = target_velocity_rad_s - actual_velocity_rad_s;

    /* 积分 */
    velocity_pi.integral +=
        velocity_pi.ki *
        error *
        VELOCITY_LOOP_DT;

    /* 积分限幅 */
    if (velocity_pi.integral > CURRENT_LIMIT)
    {
        velocity_pi.integral = CURRENT_LIMIT;
    }
    else if (velocity_pi.integral < -CURRENT_LIMIT)
    {
        velocity_pi.integral = -CURRENT_LIMIT;
    }

    /* PI */
    output =
        velocity_pi.kp * error +
        velocity_pi.integral;

    /* 输出限幅 */
    if (output > CURRENT_LIMIT)
    {
        output = CURRENT_LIMIT;
    }
    else if (output < -CURRENT_LIMIT)
    {
        output = -CURRENT_LIMIT;
    }
#endif
    return output;
}

static float Angle_Error(float target, float actual)
{
    float error = target - actual;

    if (error > DSQ_PI)
        error -= 2.0f * DSQ_PI;

    else if (error < -DSQ_PI)
        error += 2.0f * DSQ_PI;

    return error;
}

float Position_Controller_Update(float target_position, float actual_position, float actual_velocity)
{

    float error;
    float p;
    float i;
    float d;
    float ff1;
    float ff2;
    float velocity_cmd;

    /* Position update frequency 2ms*/
    const float dt = 0.002f;

    error = Angle_Error(target_position , actual_position);

    // P
    p =
        position_pid.kp *
        error;

    // I
    position_pid.integral +=
        position_pid.ki *
        error *
        dt;

    // I 限制
    if (position_pid.integral > 20.0f)
        position_pid.integral = 20.0f;

    if (position_pid.integral < -20.0f)
        position_pid.integral = -20.0f;

    i =
        position_pid.integral;

    // D：使用实际速度
    d =
        -position_pid.kd *
        actual_velocity;

    // 一阶前馈：速度
    ff1 =
        position_ff.ff1 *
        motion_profile.velocity;

    // 二阶前馈：加速度
    ff2 =
        position_ff.ff2 *
        motion_profile.acceleration;

    velocity_cmd =
        p +
        i +
        d +
        ff1 +
        ff2;

    // 最大速度限制
    if (velocity_cmd >
        motion_profile.max_velocity)
    {
        velocity_cmd =
            motion_profile.max_velocity;
    }

    if (velocity_cmd <
        -motion_profile.max_velocity)
    {
        velocity_cmd =
            -motion_profile.max_velocity;
    }

    return velocity_cmd;
}

float Position_Control_Update(float target_position, float actual_position, float actual_velocity)
{
    float position_error;
    float velocity_cmd;
    float dt = 0.002f;
    static float prev_vel_cmd = 0.0f;

    position_error =
        Angle_Error(
            target_position,
            actual_position
        );

    // I
    position_pid.integral +=
        position_pid.ki *
		position_error *
        dt;

    // I 限制
    if (position_pid.integral > 20.0f)
        position_pid.integral = 20.0f;

    if (position_pid.integral < -20.0f)
        position_pid.integral = -20.0f;

    float i =
        position_pid.integral;

    velocity_cmd =
    		position_pid.kp * position_error
        - position_pid.kd * actual_velocity
		+ i
		+ position_ff.ff1 * prev_vel_cmd
		+ position_ff.ff2 * motion_profile.max_acceleration;

    prev_vel_cmd = velocity_cmd;

    // velocity limit
    if (velocity_cmd > MAX_VELOCITY)
        velocity_cmd = MAX_VELOCITY;

    if (velocity_cmd < -MAX_VELOCITY)
        velocity_cmd = -MAX_VELOCITY;

    return velocity_cmd;
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

void Position_Update()
{
#ifndef DEBUG
	// 真实角度，UNIT: rad //
	float acut_position = servo_rt.angle;

	// 真实速度，UNIT: rad/s //
	float acut_velocity = servo_rt.speed;

	float vel_cmd = Position_Control_Update(motion_profile.target_position,
						acut_position,
						acut_velocity);

	// real time update the control table
	// 104	4	Goal Velocity
	int32_t goal_vel = RadS_To_DXL_Velocity(vel_cmd);
	ControlTable_WriteInt32(goal_vel, 104);
#endif
}
