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
#include "velocity_loop_q.h"
#include "position_loop_q.h"

#if defined(__ARM_ARCH)
  #define CUR_IRQ_SAVE()     uint32_t cur_pm_ = __get_PRIMASK(); __disable_irq()
  #define CUR_IRQ_RESTORE()  __set_PRIMASK(cur_pm_)
#else                                   /* PC 上测试用 */
  #define CUR_IRQ_SAVE()     (void)0
  #define CUR_IRQ_RESTORE()  (void)0
#endif

typedef struct {
    float    current;      // A,带符号
    float    speed;        // rad/s

    uint16_t raw_angle;    // 0~4095,已扣零点偏移
    uint16_t vbus_raw;
} servo_snap_t;

ServoState_t   servo_state;
volatile ServoRuntime_t servo_rt;
extern   DMA_HandleTypeDef hdma_adc1;
static   cur_pi_q_t cur_pi;

volatile float    goal_current;
volatile int32_t  goal_mA;              /* TIM2 速度环里: goal_mA = (int32_t)(goal_current * 1000.0f); */
extern float      ipropi_offset;

volatile int32_t     pos_target_q8;     /* 目标位置, 计数 x 256, 只由主循环写 (32 位对齐, ISR 读取是原子的) */
volatile int32_t     goal_vel_q8;       /* 位置环输出, 速度环直接读取 (取代控制表 104) */

static int32_t    cur_flt_q4;           /* ADC 一阶滤波值 Q4                */
static int32_t    ipropi_offset_q4;     /* 零电流偏置 Q4                    */

extern volatile  vel_pi_q_t      vel_pi;
extern volatile  pos_pid_q_t     pos_pid;

#define PWM_REPORT_SCALE  100     // 100 = 1%(和原来一致);1000 = 0.1%
#define PWM_REPORT_K  ((int32_t)(PWM_DUTY_MAX * (float)PWM_REPORT_SCALE * 256.0f + 0.5f))  // 编译期常量

void Position_Update(void);

static inline int16_t pwm_q16_to_report(int32_t d)
{
    int32_t m = (d < 0) ? -d : d;                                   // <= 65536
    int32_t v = (int32_t)(((int64_t)m * PWM_REPORT_K + (1 << 23)) >> 24);
    return (int16_t)((d < 0) ? -v : v);
}

/* float -> 定点,并饱和,防止上位机写入过大增益导致整数溢出 */
static inline int32_t cur_f2q_sat(float x, float scale)
{
    float v = x * scale;
    if (v < 0.0f)     v = 0.0f;
    if (v > 1.0e9f)   v = 1.0e9f;
    return (int32_t)(v + 0.5f);
}

static inline void cur_pi_q_reset(cur_pi_q_t *p)
{
    CUR_IRQ_SAVE();
    p->integ_q36 = 0;
    CUR_IRQ_RESTORE();
}

/*
 * 设置增益(单位与原 float PI 完全一致):
 *     kp : 占空比 / A
 *     ki : 占空比 / (A*s)
 * 在主循环或 Proc_control_table_Update() 里调用,不要在 ISR 里调用。
 */
static inline void cur_pi_q_set_gains(cur_pi_q_t *p, float kp, float ki)
{
    int32_t kp_q  = cur_f2q_sat(kp / PWM_DUTY_MAX / 1000.0f, 4294967296.0f);            /* 2^32 */
    int32_t ki_q  = cur_f2q_sat(ki / PWM_DUTY_MAX * CURRENT_LOOP_DT / 1000.0f,
                                68719476736.0f);                                        /* 2^36 */

    float   imx   = CURRENT_PI_MAX / PWM_DUTY_MAX;
    if (imx > 1.0f) imx = 1.0f;
    int64_t imax  = (int64_t)(imx * 68719476736.0f);

    CUR_IRQ_SAVE();
    p->kp_q32        = kp_q;
    p->ki_dt_q36     = ki_q;
    p->integ_max_q36 = imax;
    if (p->integ_q36 >  imax) p->integ_q36 =  imax;
    if (p->integ_q36 < -imax) p->integ_q36 = -imax;
    CUR_IRQ_RESTORE();
}

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

void Servo_Position_limit_Update()
{
	servo_rt.max_pos = ControlTable_ReadInt32(48);
	if (servo_rt.max_pos > 4095 || servo_rt.max_pos <= 0){
		servo_rt.max_pos = 4095;
		ControlTable_WriteInt32(servo_rt.max_pos, MAX_POSITION_LIMIT_ADDR);
	}

	servo_rt.min_pos = ControlTable_ReadInt32(52);
	if (servo_rt.min_pos < 0 || servo_rt.min_pos >= 4095){
		servo_rt.min_pos = 0;
		ControlTable_WriteInt32(servo_rt.min_pos, MIN_POSITION_LIMIT_ADDR);
	}

	if (servo_rt.min_pos > servo_rt.max_pos){
		/* swap pos*/
		int32_t tmp = servo_rt.min_pos;
		servo_rt.min_pos = servo_rt.max_pos;
		servo_rt.max_pos = tmp;

		ControlTable_WriteInt32(servo_rt.min_pos, MIN_POSITION_LIMIT_ADDR);
		ControlTable_WriteInt32(servo_rt.max_pos, MAX_POSITION_LIMIT_ADDR);
	}
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
	Motion_Profile_Update();

	Motion_Vel_PI_Update();
	Motion_Pos_PID_Update();

	motion_profile_ready = 1;

	/* 位置限制更新 */
	Servo_Position_limit_Update();

	/* 设置电流环PI*/
	cur_pi_q_set_gains(&cur_pi, CURRENT_PI_Kp, CURRENT_PI_Ki);

	/* 设置速度环PI*/
	vel_pi_q_set_gains((vel_pi_q_t*)&vel_pi, velocity_pi.kp, velocity_pi.ki);

	/* 设置上电角度 */
	int32_t home_offset = ControlTable_ReadInt32(HOME_OFFSET_ADDR);
	if (home_offset > 2048) {
		home_offset = 2048;
		ControlTable_WriteInt32(home_offset, HOME_OFFSET_ADDR);
	}else if (home_offset < -2047){
		home_offset = -2047;
		ControlTable_WriteInt32(home_offset, HOME_OFFSET_ADDR);
	}

	uint32_t home_pos = 2048 - home_offset;
	ControlTable_WriteUInt32(home_pos, 116);

	/* 开启 ADC DMA */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_DMA_LENGTH);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);

	/* 开启 TIM1 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	/* 计数器开始 */
	__HAL_TIM_ENABLE(&htim1);

  	servo_rt.speed = 0.0f;

  	servo_rt.current = 0.0f;
  	servo_rt.state = 0;
#endif

}

/*
 * 一步 PI。
 *     err_mA : 目标电流 - 实际电流 (mA, 带符号)
 *     返回   : Q16 归一化占空比, +-65536 = +-PWM_DUTY_MAX
 * 内部用 int64 做乘加,任何增益设置下都不会溢出回绕。
 */
static inline int32_t cur_pi_q_step(cur_pi_q_t *p, int32_t err_mA)
{
    int64_t integ = p->integ_q36 + (int64_t)p->ki_dt_q36 * err_mA;
    if (integ >  p->integ_max_q36) integ =  p->integ_max_q36;
    if (integ < -p->integ_max_q36) integ = -p->integ_max_q36;
    p->integ_q36 = integ;

    /* kp: Q32,积分: Q36 -> 先右移 4 位对齐到 Q32,再整体右移 16 位得到 Q16 */
    int64_t out = ((int64_t)p->kp_q32 * err_mA + (integ >> 4)) >> 16;
    if (out >  65536) out =  65536;
    if (out < -65536) out = -65536;
    return (int32_t)out;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
#ifndef DEBUG
    static uint8_t div;

    if (hadc->Instance != ADC1) return;

    uint16_t raw_v = adc_dma_buffer[0];     /* 先拷出,DMA 是循环模式 */
    uint16_t raw_i = adc_dma_buffer[1];

    if (servo_state == SERVO_IPROPI_CAL) {
        if (!IPROPI_Calibrate()) {
            ipropi_offset_q4 = (int32_t)(ipropi_offset * 16.0f + 0.5f);   /* 只执行一次 */
            cur_flt_q4       = ipropi_offset_q4;
            cur_pi_q_reset(&cur_pi);
            servo_state      = SERVO_READY;
        }
        return;
    }

    if (servo_state != SERVO_READY && servo_state != SERVO_RUN) return;

    servo_rt.vm_bus_raw = raw_v;

    /* 20 kHz 一阶低通, alpha = 1/8 (原来是 0.1) */
    cur_flt_q4 += (((int32_t)raw_i << 4) - cur_flt_q4) >> 3;

    if (++div < CURRENT_LOOP_DIV) return;   /* 周期正好是 CURRENT_LOOP_DIV 次 */
    div = 0;

    int32_t i_mA = cur_adc_to_mA(cur_flt_q4, ipropi_offset_q4);
    if (servo_rt.cursign < 0) i_mA = -i_mA;
    servo_rt.current_mA = i_mA;             /* 控制表 126~127 直接用它,不需要 float */

    if (DXL_RAMTable[TORQUE_ENABLE_ADDR]) {
        servo_rt.pwmset = cur_pi_q_step(&cur_pi, goal_mA - i_mA);
        servo_rt.cursign = (servo_rt.pwmset > 0) ? 1 : -1;

        Motor_SetPWM_Q16(servo_rt.pwmset);
    } else {
        cur_pi_q_reset(&cur_pi);            /* 力矩关闭时清积分,避免重新使能时跳变 */
    }
#endif
}

#if 1
void Servo_Velocity_PI_Update(void)
{
    if (!DXL_RAMTable[TORQUE_ENABLE_ADDR]) {          /* 力矩关闭: 清积分,目标电流 0 */
        vel_pi_q_reset((vel_pi_q_t*)&vel_pi);
        goal_mA = 0;
        return;
    }

    int32_t g = goal_vel_q8;                          /* DXL 速度单位 x 256 */
    if (g >  VEL_GOAL_CLAMP * 256) g =  VEL_GOAL_CLAMP * 256;
    if (g < -VEL_GOAL_CLAMP * 256) g = -VEL_GOAL_CLAMP * 256;

    goal_mA = vel_pi_q_step((vel_pi_q_t*)&vel_pi, g, servo_rt.vel_q8);
}


void Position_Update(void)               /* TIM2 回调里, 500 Hz, 紧跟在 Servo_Velocity_PI_Update 之后 */
{
    if (!DXL_RAMTable[TORQUE_ENABLE_ADDR]) {          /* 力矩关闭: 清积分和 prev, 目标速度 0 */
        pos_pid_q_reset((pos_pid_q_t *)&pos_pid);
        goal_vel_q8 = 0;
        return;
    }
    goal_vel_q8 = pos_pid_q_step((pos_pid_q_t *)&pos_pid, pos_target_q8, servo_rt.raw_angle, servo_rt.vel_q8);
}

/* 主循环里, 放在 COMM_Process() 之后。
 * 目标位置直接取控制表 116 的整数值 (单位与 raw_angle 相同, 0..4095), 不经过 rad, 没有 float */
void Servo_PositionTargetSync(void)
{
    if (!DXL_RAMTable[TORQUE_ENABLE_ADDR])
        ControlTable_WriteUInt32(servo_rt.raw_angle, 116);    /* 力矩关闭: 目标位置跟随当前位置 */

    uint32_t goal = ControlTable_ReadUInt32(116);             /* WRITE / SYNC WRITE 之后表里的值 */
    if (goal > 4095) goal = 4095;                             /* 也可以按 min/max position limit 限幅 */

    if (goal < servo_rt.min_pos) goal = servo_rt.min_pos;
    if (goal > servo_rt.max_pos) goal = servo_rt.max_pos;

    pos_target_q8 = (int32_t)(goal << 8);
}

/* ---- 主循环里 (和 COMM_Process 放在一起) ---- */
void Servo_MainLoopSync(void)
{
    static uint32_t last;
    static float vbus_f;
    static uint8_t prev_mode = 0xFF;

    uint32_t now = HAL_GetTick();
    if ((now - last) < 1) return;        // 1kHz 足够
    last = now;

    uint8_t mode = DXL_RAMTable[OPERATION_MODE_ADDR];
    if (mode != prev_mode) {                       /* 模式切换: 清两个环的状态 */
        prev_mode = mode;
        pos_pid_q_reset((pos_pid_q_t*)&pos_pid);
        vel_pi_q_reset((vel_pi_q_t*)&vel_pi);
        goal_vel_q8 = 0;
    }

    if (mode == VELOCITY_MODE){
        /* 目标速度: 在主循环里读控制表, 与主机的 WRITE 在同一上下文, 不会读到一半 */
        int32_t g = ControlTable_ReadInt32(104);
        if (g >  VEL_GOAL_CLAMP) g =  VEL_GOAL_CLAMP;
        if (g < -VEL_GOAL_CLAMP) g = -VEL_GOAL_CLAMP;
        goal_vel_q8 = g * 256;
    }else{
    	/* 更新目标位置 */
    	Servo_PositionTargetSync();
    }

    /* 更新目标电流 */
    int32_t gm = goal_mA;
    DXL_RAMTable[102] = GET_INT16_LSB(gm);
    DXL_RAMTable[103] = GET_INT16_MSB(gm);

    /* 发布给上位机 */
    servo_snap_t s;
    servo_snapshot(&s);

    vbus_f += 0.1f * (ADC_To_VBus(s.vbus_raw) - vbus_f);     // 一阶滤波,代替原来的平均
    uint16_t uvol = (uint16_t)(vbus_f * 10.0f);
    DXL_RAMTable[144] = GET_INT16_LSB(uvol);
    DXL_RAMTable[145] = GET_INT16_MSB(uvol);

	/* prezPWM : 0% ~ 100% */
    int32_t d = servo_rt.pwmset;
    int16_t prezPWM = pwm_q16_to_report(d);

	// Update the DXL_RAMTable
	DXL_RAMTable[124] = GET_INT16_LSB(prezPWM);
	DXL_RAMTable[125] = GET_INT16_MSB(prezPWM);

    /* 当前电流, 2Bytes signed */
    ControlTable_WriteInt16(servo_rt.current_mA, 126);

    /* 当前速度 */
    int32_t pv = (servo_rt.vel_q8 + 128) >> 8;                     /* DXL 速度单位, 四舍五入 */
    ControlTable_WriteInt32(pv, 128);

    /* 当前位置 */
    ControlTable_WriteUInt32(s.raw_angle, 132);
}
#endif


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#ifndef DEBUG
    static uint8_t  cnt, have_prev;
    static uint16_t prev_raw;
    static int32_t  vel_q8;
    uint16_t raw;

    if (htim->Instance != TIM2) return;

    ANGLE_getRawAngle(&raw);

    raw = (uint16_t)((raw - servo_rt.home_offset) & 0x0FFF);   /* 零点偏移, 模 4096 */
    servo_rt.raw_angle = raw;

    if (have_prev) {                                           /* 读角度后的第一拍只做同步 */
        int32_t inst_q8 = enc_delta_counts(raw, prev_raw) * VEL_Q8_PER_COUNT;
        vel_q8 += (inst_q8 - vel_q8) >> SPEED_FILT_SHIFT;
        servo_rt.vel_q8 = vel_q8;
        Servo_Velocity_PI_Update();
    }

    prev_raw  = raw;
    have_prev = 1;

    if (++cnt >= 2) {                                              /* 位置环 500 Hz */
        cnt = 0;
        // 11	1	Operating Mode
        uint8_t operation_mode = DXL_RAMTable[OPERATION_MODE_ADDR];
        if (operation_mode == POSITION_MODE){
        	Position_Update();
        }
    }

    /* restart read as5600 */
    ANGLE_StartRead();

#else
    uint16_t raw;
    ANGLE_getRawAngle(&raw);
#endif
}

