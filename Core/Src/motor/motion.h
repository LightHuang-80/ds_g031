/*
 * motion.h
 *
 *  Created on: 2026年9月22日
 *      Author: Huang
 */

#ifndef SRC_MOTOR_MOTION_H_
#define SRC_MOTOR_MOTION_H_

#define DXL_ACCEL_UNIT_REV_MIN2       214.577f

#define DXL_ACCEL_UNIT_RAD_S2         \
    (DXL_ACCEL_UNIT_REV_MIN2 * 2.0f * DSQ_PI / 3600.0f)

#define VELOCITY_LOOP_DT      0.001f      // 1ms
#define CURRENT_LIMIT         1.4f       // 最大目标电流，按你的实际需求修改

#define POSITION_DEADBAND  0.002f
#define MAX_VELOCITY       7.2f

typedef struct VelocityPI_
{
    float kp;
    float ki;
    float integral;
} VelocityPI_t;

extern volatile VelocityPI_t  velocity_pi;

typedef struct PositionPID_
{
    float kp;
    float ki;
    float kd;

    float integral;

} PositionPID_t;

extern volatile PositionPID_t  position_pid;

typedef struct PositionFF_
{
    float ff1;
    float ff2;
} PositionFF_t;

typedef struct MotionProfile_
{
    float position;
    float velocity;
    float acceleration;

    float target_position;

    float max_velocity;
    float max_acceleration;

} MotionProfile_t;

extern uint8_t position_ready;
extern uint8_t motion_profile_ready;

#define DSQ_VELOCITY_UNIT_RAD_S    0.0239691227f
#define DSQ_PI                     3.14159265f
#define DSQ_2PI                    6.2831853f
#define DSQ_POSITION_ZERO          2048.0f
#define DSQ_POSITION_SCALE_RAD     (DSQ_PI / 2048.0f)

/*==============   单位换算   =========================*/
float    DXL_Position_To_Rad(uint32_t value);
uint32_t Rad_To_DXL_Position(float rad);
float    DXL_Velocity_To_RadS(int32_t value);
int32_t  RadS_To_DXL_Velocity(float rad_s);
float    DXL_Acceleration_To_RadS2(uint32_t value);
uint32_t RadS2_To_DXL_Acceleration(float acceleration_rad_s2);
/* =====================================================*/

float Velocity_PI_Update(float target_velocity_rad_s,
                         float actual_velocity_rad_s);

void Motion_goPosition();

void Motion_Pos_PID_Update();
void Motion_Vel_PI_Update();
void Motion_Profile_Update();

#endif /* SRC_MOTOR_MOTION_H_ */
