/*
 * velocity_loop_q.h  --  速度测量 + 速度环定点化 (Cortex-M0+, 无 FPU)
 *
 * 使用前必须已经定义 (config.h / dynamixel 头文件, 都要带 f 后缀):
 *     DYNA_VEL_UNIT              1 个 DXL 速度单位 = 多少 rad/s
 *     DSQ_POSITION_SCALE_RAD     1 个编码器计数 = 多少 rad (2*pi/4096)
 *     VELOCITY_LOOP_DT           速度环周期 (s), 1ms
 *     CURRENT_LIMIT              速度环输出限幅 (A)
 *
 * 数据格式
 *     编码器          uint16, 0..4095 (已扣零点偏移)
 *     速度            int32,  Q8 的 DXL 速度单位 (单位值 x 256)
 *     目标速度        int32,  DXL 速度单位 (控制表 104 的原始值)
 *     kp              int32,  Q16 ( mA / DXL速度单位 )
 *     ki*dt           int32,  Q28 ( mA / DXL速度单位 / 步 )
 *     积分状态        int64,  Q36 ( mA x 2^36 )
 *     输出            int32,  mA  (直接作为电流环的目标电流 goal_mA)
 *
 * 增益 float -> 整数 的换算只在 vel_pi_q_set_gains() 里做(改参数时调用,不要在 ISR 里调用)。
 * 增益单位与原 VelocityPI_t 完全一致: kp [A/(rad/s)], ki [A/(rad/s*s)]。
 */
#ifndef VELOCITY_LOOP_Q_H
#define VELOCITY_LOOP_Q_H

#include <stdint.h>
#include "motion.h"

/* 1 个编码器计数/ms 对应多少 "DXL 速度单位 x 256"  (约 63.97 * 256 = 16377), 编译期常量 */
#define VEL_Q8_PER_COUNT \
    ((int32_t)(DSQ_POSITION_SCALE_RAD * (1.0f / VELOCITY_LOOP_DT) / DYNA_VEL_UNIT * 256.0f + 0.5f))

/* 速度输出限幅 (mA) */
#define VEL_CUR_LIMIT_MA   ((int32_t)(CURRENT_LIMIT * 1000.0f + 0.5f))

/* 目标速度钳位 (DXL 单位),防止异常值溢出 */
#define VEL_GOAL_CLAMP     32767

/* 速度一阶滤波: 0 = 不滤波(与原代码一致), 2 = alpha 1/4 (推荐尝试) */
#ifndef SPEED_FILT_SHIFT
#define SPEED_FILT_SHIFT   0
#endif

#if defined(__ARM_ARCH)
  #define VEL_IRQ_SAVE()     uint32_t vel_pm_ = __get_PRIMASK(); __disable_irq()
  #define VEL_IRQ_RESTORE()  __set_PRIMASK(vel_pm_)
#else                                   /* PC 上测试用 */
  #define VEL_IRQ_SAVE()     (void)0
  #define VEL_IRQ_RESTORE()  (void)0
#endif

/* 两次采样之间的编码器增量,折叠到 [-2048, 2048] (等价于原来 ±pi 的折叠) */
static inline int32_t enc_delta_counts(uint16_t raw, uint16_t prev)
{
    int32_t d = (int32_t)raw - (int32_t)prev;
    if (d >  2048) d -= 4096;
    if (d < -2048) d += 4096;
    return d;
}

/* ------------------------------------------------------------------ */
typedef struct {
    int32_t kp_q16;
    int32_t ki_dt_q28;
    int64_t integ_q36;
    int64_t integ_max_q36;
} vel_pi_q_t;

static inline int32_t vel_f2q_sat(float x)
{
    if (x < 0.0f)   x = 0.0f;
    if (x > 1.0e9f) x = 1.0e9f;
    return (int32_t)(x + 0.5f);
}

static inline void vel_pi_q_reset(vel_pi_q_t *p)
{
    VEL_IRQ_SAVE();
    p->integ_q36 = 0;
    VEL_IRQ_RESTORE();
}

/* kp: A/(rad/s), ki: A/(rad/s*s)。在主循环 / 参数更新时调用。 */
static inline void vel_pi_q_set_gains(vel_pi_q_t *p, float kp, float ki)
{
    const float k_unit = 1000.0f * DYNA_VEL_UNIT;       /* (mA/单位) per (A/(rad/s)) */
    int32_t kp_q = vel_f2q_sat(kp * k_unit * 65536.0f);                           /* 2^16 */
    int32_t ki_q = vel_f2q_sat(ki * k_unit * VELOCITY_LOOP_DT * 268435456.0f);    /* 2^28 */
    int64_t imax = (int64_t)((float)VEL_CUR_LIMIT_MA * 68719476736.0f);           /* 2^36 */

    VEL_IRQ_SAVE();
    p->kp_q16        = kp_q;
    p->ki_dt_q28     = ki_q;
    p->integ_max_q36 = imax;
    if (p->integ_q36 >  imax) p->integ_q36 =  imax;
    if (p->integ_q36 < -imax) p->integ_q36 = -imax;
    VEL_IRQ_RESTORE();
}

/*
 * goal_q8 / meas_q8 : DXL 速度单位 x 256
 * 返回              : 目标电流 mA (已限幅到 +-VEL_CUR_LIMIT_MA)
 * int64 乘加,任何增益设置下都不会回绕。
 */
static inline int32_t vel_pi_q_step(vel_pi_q_t *p, int32_t goal_q8, int32_t meas_q8)
{
    int32_t err_q8 = goal_q8 - meas_q8;

    int64_t integ = p->integ_q36 + (int64_t)p->ki_dt_q28 * err_q8;     /* Q28 x Q8 = Q36 */
    if (integ >  p->integ_max_q36) integ =  p->integ_max_q36;
    if (integ < -p->integ_max_q36) integ = -p->integ_max_q36;
    p->integ_q36 = integ;

    /* kp: Q16 x Q8 = Q24;积分 Q36 >> 12 = Q24;相加后 >> 24 得到 mA */
    int64_t out = ((int64_t)p->kp_q16 * err_q8 + (integ >> 12)) >> 24;
    if (out >  VEL_CUR_LIMIT_MA) out =  VEL_CUR_LIMIT_MA;
    if (out < -VEL_CUR_LIMIT_MA) out = -VEL_CUR_LIMIT_MA;
    return (int32_t)out;
}

/* ------------------------------------------------------------------ */
/*  使用示例                                                            */
/* ------------------------------------------------------------------ */
#if 0
/* servo_rt 里新增: volatile uint16_t raw_angle; volatile int32_t vel_q8;
 * goal_vel_q8 在 position_loop_q.h 的示例里定义: 位置环输出,速度环直接读取 (取代控制表 104) */
static vel_pi_q_t       vel_pi;
extern volatile int32_t goal_vel_q8;

void Servo_Velocity_PI_Update(void)
{
    if (!DXL_RAMTable[TORQUE_ENABLE_ADDR]) {          /* 力矩关闭: 清积分,目标电流 0 */
        vel_pi_q_reset(&vel_pi);
        goal_mA = 0;
        return;
    }
    int32_t g = goal_vel_q8;                          /* DXL 速度单位 x 256 */
    if (g >  VEL_GOAL_CLAMP * 256) g =  VEL_GOAL_CLAMP * 256;
    if (g < -VEL_GOAL_CLAMP * 256) g = -VEL_GOAL_CLAMP * 256;

    goal_mA = vel_pi_q_step(&vel_pi, g, servo_rt.vel_q8);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t  cnt, have_prev;
    static uint16_t prev_raw;
    static int32_t  vel_q8;
    uint16_t raw;
    uint8_t  valid = 0;

    if (htim->Instance != TIM2) return;

    if (!ANGLE_getRawAngle(&raw)) {
        raw = (uint16_t)((raw - servo_rt.home_offset) & 0x0FFF);   /* 零点偏移, 模 4096 */
        servo_rt.raw_angle = raw;

        if (have_prev) {                                           /* 读失败后的第一拍只做同步 */
            int32_t inst_q8 = enc_delta_counts(raw, prev_raw) * VEL_Q8_PER_COUNT;
            vel_q8 += (inst_q8 - vel_q8) >> SPEED_FILT_SHIFT;
            servo_rt.vel_q8 = vel_q8;
            Servo_Velocity_PI_Update();
            valid = 1;
        }
        prev_raw  = raw;
        have_prev = 1;
    } else {
        have_prev = 0;
    }

    if (++cnt >= 2) {                                              /* 位置环 500 Hz */
        cnt = 0;
        if (valid) Position_Update();                              /* 传感器无效时不跑位置环, 积分不会在旧数据上累积 */
    }
}

/* ---- 主循环里 (和 COMM_Process 放在一起) ---- */
void Servo_MainLoopSync(void)
{
    /* 发布给上位机 (控制表 104 只是显示, 不再参与控制) */
    ControlTable_WriteInt32((goal_vel_q8 + 128) >> 8, 104);

    int32_t gm = goal_mA;
    DXL_RAMTable[102] = GET_INT16_LSB(gm);
    DXL_RAMTable[103] = GET_INT16_MSB(gm);

    int32_t pv = (servo_rt.vel_q8 + 128) >> 8;                     /* DXL 速度单位, 四舍五入 */
    ControlTable_WriteInt32(pv, 128);
}
#endif

#endif /* VELOCITY_LOOP_Q_H */
