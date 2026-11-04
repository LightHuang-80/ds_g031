/*
 * position_loop_q.h  --  位置环定点化 (Cortex-M0+, 无 FPU)
 *
 * 使用前必须已经定义 (都要带 f 后缀):
 *     DSQ_POSITION_ZERO          编码器零位对应的计数 (2048.0f)
 *     DSQ_POSITION_SCALE_RAD     1 个编码器计数 = 多少 rad (pi/2048)
 *     DYNA_VEL_UNIT              1 个 DXL 速度单位 = 多少 rad/s
 *     VELOCITY_LOOP_DT           速度环周期 1ms
 *     MAX_VELOCITY               位置环输出限幅 (rad/s)
 *
 * 数据格式
 *     位置          编码器计数 x 256 (Q8, 0 .. 4095*256)
 *     速度          DXL 速度单位 x 256 (Q8)  与 velocity_loop_q.h 一致
 *     误差          计数 x 256, 折叠到 +-2048 计数
 *     kp / kd / ff1 int32 Q16
 *     ki*dt         int32 Q32 (DXL单位 / 计数 / 步)
 *     积分状态      int64 Q40 (DXL 速度单位 x 2^40)
 *     输出          int32 Q8 DXL 速度单位, 直接作为速度环的目标速度 goal_vel_q8
 *
 * 增益单位与原 PositionPID_t / PositionFF_t 完全一致:
 *     kp [1/s]   ki [1/s^2]   kd [无量纲]   ff1 [无量纲]   ff2 [s]
 * float -> 整数 的换算只在 pos_pid_q_set_gains() 里做(改参数时调用, 不要在 ISR 里调用)。
 */
#ifndef POSITION_LOOP_Q_H
#define POSITION_LOOP_Q_H

#include <stdint.h>
#include "motion.h"

#define POS_LOOP_DT           (2.0f * VELOCITY_LOOP_DT)     /* 位置环 = 速度环 2 分频 */
#define POS_VEL_MAX_Q8        ((int32_t)(MAX_VELOCITY / DYNA_VEL_UNIT * 256.0f + 0.5f))

/* 积分限幅 (rad/s)。原代码是 20.0f, 比 MAX_VELOCITY(7.2) 大, 建议改成 MAX_VELOCITY */
#ifndef POS_INTEG_LIMIT
#define POS_INTEG_LIMIT       20.0f
#endif
#define POS_INTEG_MAX_Q40     ((int64_t)(POS_INTEG_LIMIT / DYNA_VEL_UNIT * 1099511627776.0f))

#if defined(__ARM_ARCH)
  #define POS_IRQ_SAVE()     uint32_t pos_pm_ = __get_PRIMASK(); __disable_irq()
  #define POS_IRQ_RESTORE()  __set_PRIMASK(pos_pm_)
#else                                   /* PC 上测试用 */
  #define POS_IRQ_SAVE()     (void)0
  #define POS_IRQ_RESTORE()  (void)0
#endif

typedef struct {
    int32_t kp_q16;          /* (DXL单位 / 计数) * 2^16        */
    int32_t kd_q16;          /* 无量纲 * 2^16                  */
    int32_t ki_dt_q32;       /* (DXL单位 / 计数 / 步) * 2^32   */
    int32_t ff1_q16;         /* 无量纲 * 2^16                  */
    int32_t ff2_const_q8;    /* ff2 * max_acceleration, 换成 DXL单位 x 256 (常数) */
    int32_t prev_q8;         /* 上一次(未限幅的)输出, 与原代码 prev_vel_cmd 一致     */
    int64_t integ_q40;
} pos_pid_q_t;

/* 有符号饱和: float -> int32 (+-1e9) */
static inline int32_t pos_f2q_sat(float x)
{
    if (x >  1.0e9f) x =  1.0e9f;
    if (x < -1.0e9f) x = -1.0e9f;
    return (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}

static inline void pos_pid_q_reset(pos_pid_q_t *p)
{
    POS_IRQ_SAVE();
    p->integ_q40 = 0;
    p->prev_q8   = 0;
    POS_IRQ_RESTORE();
}

/*
 * 在主循环 / 参数更新时调用 (motion_profile.max_acceleration 变化时也要重新调用)。
 *     max_accel: 与原代码里 motion_profile.max_acceleration 同单位
 */
static inline void pos_pid_q_set_gains(pos_pid_q_t *p,
                                       float kp, float ki, float kd,
                                       float ff1, float ff2, float max_accel)
{
    const float cnt_to_unit = DSQ_POSITION_SCALE_RAD / DYNA_VEL_UNIT;   /* DXL单位 / 计数 per (1/s) */

    int32_t kp_q  = pos_f2q_sat(kp  * cnt_to_unit * 65536.0f);
    int32_t kd_q  = pos_f2q_sat(kd  * 65536.0f);
    int32_t ki_q  = pos_f2q_sat(ki  * POS_LOOP_DT * cnt_to_unit * 4294967296.0f);
    int32_t ff1_q = pos_f2q_sat(ff1 * 65536.0f);
    int32_t ff2_q = pos_f2q_sat(ff2 * max_accel / DYNA_VEL_UNIT * 256.0f);

    POS_IRQ_SAVE();
    p->kp_q16 = kp_q;  p->kd_q16 = kd_q;  p->ki_dt_q32 = ki_q;
    p->ff1_q16 = ff1_q; p->ff2_const_q8 = ff2_q;
    POS_IRQ_RESTORE();
}

/* rad -> 计数 x 256。在低频上下文(目标位置更新时)调用, 内部有 float。 */
static inline int32_t pos_rad_to_q8(float rad)
{
    float c = rad / DSQ_POSITION_SCALE_RAD + DSQ_POSITION_ZERO;
    if (c < 0.0f)        c = 0.0f;
    if (c > 4095.99f)    c = 4095.99f;
    return (int32_t)(c * 256.0f + 0.5f);
}

/*
 * target_q8 : 目标位置, 计数 x 256
 * raw       : 当前编码器计数 (0..4095, 已扣零点偏移)
 * vel_q8    : 当前速度, DXL 速度单位 x 256
 * 返回      : 目标速度, DXL 速度单位 x 256 (已限幅到 +-MAX_VELOCITY)
 */
static inline int32_t pos_pid_q_step(pos_pid_q_t *p, int32_t target_q8, uint16_t raw, int32_t vel_q8)
{
    /* 位置误差, 折叠到 +-2048 计数 (等价于原来的 Angle_Error, 最短路径) */
    int32_t err = target_q8 - ((int32_t)raw << 8);
    if (err >  (2048 << 8)) err -= (4096 << 8);
    if (err < -(2048 << 8)) err += (4096 << 8);

    /* 积分 + 限幅 */
    int64_t integ = p->integ_q40 + (int64_t)p->ki_dt_q32 * err;        /* Q32 x Q8 = Q40 */
    if (integ >  POS_INTEG_MAX_Q40) integ =  POS_INTEG_MAX_Q40;
    if (integ < -POS_INTEG_MAX_Q40) integ = -POS_INTEG_MAX_Q40;
    p->integ_q40 = integ;

    /* 各项合成, 单位 DXL 速度单位 x 256 */
    int64_t out = (((int64_t)p->kp_q16  * err)       >> 16)
                - (((int64_t)p->kd_q16  * vel_q8)    >> 16)
                +  (integ >> 32)
                + (((int64_t)p->ff1_q16 * p->prev_q8) >> 16)
                +  p->ff2_const_q8;

    /* 与原代码一致: prev 保存限幅之前的值 (仅为防溢出限在 +-2^30) */
    int64_t pv = out;
    if (pv >  (1 << 30)) pv =  (1 << 30);
    if (pv < -(1 << 30)) pv = -(1 << 30);
    p->prev_q8 = (int32_t)pv;

    if (out >  POS_VEL_MAX_Q8) out =  POS_VEL_MAX_Q8;
    if (out < -POS_VEL_MAX_Q8) out = -POS_VEL_MAX_Q8;
    return (int32_t)out;
}

/* ------------------------------------------------------------------ */
/*  使用示例                                                            */
/* ------------------------------------------------------------------ */
#if 0
static pos_pid_q_t   pos_pid;
volatile int32_t     pos_target_q8;     /* 目标位置, 计数 x 256, 只由主循环写 (32 位对齐, ISR 读取是原子的) */
volatile int32_t     goal_vel_q8;       /* 位置环输出, 速度环直接读取 (取代控制表 104) */

void Position_Update(void)               /* TIM2 回调里, 500 Hz, 紧跟在 Servo_Velocity_PI_Update 之后 */
{
    if (!DXL_RAMTable[TORQUE_ENABLE_ADDR]) {          /* 力矩关闭: 清积分和 prev, 目标速度 0 */
        pos_pid_q_reset(&pos_pid);
        goal_vel_q8 = 0;
        return;
    }
    goal_vel_q8 = pos_pid_q_step(&pos_pid, pos_target_q8, servo_rt.raw_angle, servo_rt.vel_q8);
}

/* 主循环里, 放在 COMM_Process() 之后。
 * 目标位置直接取控制表 116 的整数值 (单位与 raw_angle 相同, 0..4095), 不经过 rad, 没有 float */
void Servo_PositionTargetSync(void)
{
    if (!DXL_RAMTable[TORQUE_ENABLE_ADDR])
        ControlTable_WriteUInt32(servo_rt.raw_angle, 116);    /* 力矩关闭: 目标位置跟随当前位置 */

    uint32_t goal = ControlTable_ReadUInt32(116);             /* WRITE / SYNC WRITE 之后表里的值 */
    if (goal > 4095) goal = 4095;                             /* 也可以按 min/max position limit 限幅 */
    pos_target_q8 = (int32_t)(goal << 8);
}
/* 如果 motion_profile.target_position 还有别的来源/处理, 再用 pos_rad_to_q8() 转换 */
#endif

#endif /* POSITION_LOOP_Q_H */
