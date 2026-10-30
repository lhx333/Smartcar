/* ==================================================================================
 *  speed_control.c —— 速度输出层（电机 + 底盘模式分支）
 *
 *  电机编号→引脚映射表、左右轮 PWM/DIR 输出、点动自检、舵机/差速分支。
 *
 *  ★ 占空比一律先 fabsf() 再转 uint32，符号只走 DIR 引脚；负值直接进 pwm_set_duty
 *    的 uint32 参数会变成接近满量程的占空比，表现是「想减速反而全速冲」。
 * ================================================================================== */

#include "speed_control.h"
#include "servo_control.h"

#include <math.h>

/* ==================== 硬件映射表 ==================== */

static const gpio_pin_enum    motor_dir_pin[MOTOR_COUNT] =
{
    MOTOR1_DIR, MOTOR2_DIR,
};

static const pwm_channel_enum motor_pwm_pin[MOTOR_COUNT] =
{
    MOTOR1_PWM, MOTOR2_PWM,
};

/* 某一路电机属于左侧(+1)/右侧(-1)/未选中(0)。每次由 MOTOR_LEFT/RIGHT 现算，不存表。 */
static int8 motor_side_of(uint8 index)
{
    if (index == (uint8)(MOTOR_LEFT  - 1)) return (MOTOR_LEFT_SIGN  >= 0) ? +1 : -1;
    if (index == (uint8)(MOTOR_RIGHT - 1)) return (MOTOR_RIGHT_SIGN >= 0) ? +1 : -1;
    return 0;
}

/* 转向指令的当前值（已限速）。 */
static float turn_slew = 0.0f;
static float applied_servo_offset;
float speed_applied_servo_offset(void) { return applied_servo_offset; }

/* ==================== 底层输出 ==================== */

/* 原始驱动：不查左右，直接落 DIR + PWM。只给点动自检用（见文件末尾）。 */
static void motor_set_raw(uint8 index, float cmd, float pwm_scale_percent)
{
    float mag;

    if (index >= MOTOR_COUNT) return;

    mag = fabsf(cmd);
    if (mag > 1.0f) mag = 1.0f;

    gpio_set_level(motor_dir_pin[index], (cmd >= 0.0f) ? GPIO_HIGH : GPIO_LOW);

    /* pwm_scale_percent is the full-scale duty for this call. Normal control
     * uses DUTY_CMD_MAX; module 1 alone uses 40. pwm_set_duty takes the
     * PWM_DUTY_MAX raw scale, so convert from physical percent here.
     * mag 上面已取绝对值并钳到 1.0，结果必在 [0, PWM_DUTY_MAX]，不会是负数。 */
    pwm_set_duty(motor_pwm_pin[index],
                 (uint32)(mag * pwm_scale_percent * (float)PWM_DUTY_MAX / 100.0f));
}

/* 按侧驱动一路电机。
 * cmd ∈ [-1, 1]，正 = 该轮向前。符号在这里翻译成 DIR 电平，幅值进 pwm_set_duty。 */
static void motor_set(uint8 index, float cmd, float pwm_scale_percent)
{
    int8 side = motor_side_of(index);

    if (side == 0) return;                          /* 没被 MOTOR_LEFT/RIGHT 选中的路，不驱动 */

    motor_set_raw(index, cmd * (float)side, pwm_scale_percent);
}

/* 按侧驱动。duty ∈ [0,1] 归一化占空比。 */
static void motor_set_left(float duty, float pwm_scale_percent)
{
    motor_set((uint8)(MOTOR_LEFT  - 1), duty, pwm_scale_percent);
}

static void motor_set_right(float duty, float pwm_scale_percent)
{
    motor_set((uint8)(MOTOR_RIGHT - 1), duty, pwm_scale_percent);
}

static void motor_all_stop(void)
{
    uint8 i;
    for (i = 0; i < MOTOR_COUNT; i++)
    {
        pwm_set_duty(motor_pwm_pin[i], 0);
    }
}

/* ==================== 对外接口 ==================== */

void speed_init(void)
{
    turn_slew = 0.0f;
    applied_servo_offset = 0.0f;

    motor_all_stop();
    servo_set(0.0f);
}

static void speed_apply_internal(float turn, float duty_l, float duty_r,
                                 uint8 debug_mode, float pwm_scale_percent)
{
    float d;
    (void)debug_mode;

    /* ---------- 指令限幅 ---------- */
    if (turn >  1.0f) turn =  1.0f;
    if (turn < -1.0f) turn = -1.0f;

    if (duty_l > 1.0f) duty_l = 1.0f;
    if (duty_l < 0.0f) duty_l = 0.0f;
    if (duty_r > 1.0f) duty_r = 1.0f;
    if (duty_r < 0.0f) duty_r = 0.0f;

    /* 方向统一在这里翻：舵机和差速共用 CHASSIS_TURN_SIGN。 */
    turn *= (float)CHASSIS_TURN_SIGN;

    /* ---------- 转向指令逐拍限速 ---------- */
    d = turn - turn_slew;
    if (d >  CHASSIS_TURN_SLEW_MAX) d =  CHASSIS_TURN_SLEW_MAX;
    if (d < -CHASSIS_TURN_SLEW_MAX) d = -CHASSIS_TURN_SLEW_MAX;
    turn_slew += d;
    turn = turn_slew;

    /* ---------- 落到硬件 ---------- */
#if (CHASSIS_MODE == CHASSIS_STEERING_SERVO)

    /* Servo steering; each rear motor receives its encoder speed-loop output. */
    applied_servo_offset = turn * (debug_mode ? DEBUG_TURN_MAX_DEG : CHASSIS_TURN_MAX_DEG);
    if (debug_mode) servo_set_debug(turn);
    else servo_set(turn);
    motor_set_left(duty_l, pwm_scale_percent);
    motor_set_right(duty_r, pwm_scale_percent);

#elif (CHASSIS_MODE == CHASSIS_DIFFERENTIAL)

    /* 差速车：转向靠左右轮不同速，走出弧线。
     *
     * 两个速度环输出的平均值作为基础驱动，转向叠加左右差速。
     *
     * !! 必须走弧线，不能让内侧轮反转（原地转）：内侧轮被地面反拖后编码器里程与真实
     *    航向脱钩，位置推算失效。下面把差速量限制在「内侧轮最慢到 0」以内。
     * 限幅限制差速量 d 而不是分别钳两侧，是为了保持左右之和不变、平均速度不受影响。 */
    {
        float left, right;
        float duty, dmax;

        duty = 0.5f * (duty_l + duty_r);        /* 两轮平均 */

        d = CHASSIS_DIFF_GAIN * turn + CHASSIS_DIFF_TRIM;

        /* 限 |d| ≤ min(duty, 1 − duty)，保证 left/right 都落在 [0,1]。 */
        dmax = (duty < (1.0f - duty)) ? duty : (1.0f - duty);
        if (d >  dmax) d =  dmax;
        if (d < -dmax) d = -dmax;

        left  = duty - d;                          /* 左转（turn>0 → d>0）时左侧慢 */
        right = duty + d;                          /* 左转时右侧快 */

        motor_set_left(left, pwm_scale_percent);
        motor_set_right(right, pwm_scale_percent);
    }

#else
    #error "CHASSIS_MODE 取值非法，请在 my_common.h 里选 CHASSIS_STEERING_SERVO 或 CHASSIS_DIFFERENTIAL"
#endif
}

void speed_apply(float turn, float duty_l, float duty_r)
{
    speed_apply_internal(turn, duty_l, duty_r, 0, DUTY_CMD_MAX);
}

void speed_apply_pwm_test(float turn, float left_percent, float right_percent)
{
    if (left_percent < 0.0f) left_percent = 0.0f;
    if (right_percent < 0.0f) right_percent = 0.0f;
    if (left_percent > 40.0f) left_percent = 40.0f;
    if (right_percent > 40.0f) right_percent = 40.0f;
    speed_apply_internal(turn, left_percent / 40.0f, right_percent / 40.0f, 0, 40.0f);
}

void speed_apply_debug(float turn, float duty_l, float duty_r)
{
    speed_apply_internal(turn, duty_l, duty_r, 1, DUTY_CMD_MAX);
}

void speed_stop(void)
{
    motor_all_stop();
    servo_set(0.0f);
    turn_slew = 0.0f;
    applied_servo_offset = 0.0f;
}

/* ==================== 电机点动自检 ==================== */

#if CHASSIS_BRINGUP_MODE

static uint16 bringup_tick = 0;

uint8 speed_bringup_task(void)
{
    uint8 i;
    uint8 active = (uint8)(bringup_tick / CHASSIS_BRINGUP_TICKS);

    if (active >= MOTOR_COUNT) active = MOTOR_COUNT - 1;

    /* 用 motor_set_raw 而非 motor_set：自检就是要绕过 MOTOR_LEFT / RIGHT 的选择，
     * 否则没被选中的两路永远不转，也查不出它们对应哪只轮子。 */
    for (i = 0; i < MOTOR_COUNT; i++)
        motor_set_raw(i, (i == active) ? CHASSIS_BRINGUP_DUTY : 0.0f, DUTY_CMD_MAX);

    if (++bringup_tick >= (uint32)CHASSIS_BRINGUP_TICKS * MOTOR_COUNT)
        bringup_tick = 0;

    return (uint8)(active + 1);         /* 返回 1..MOTOR_COUNT，方便直接显示 */
}

#else

uint8 speed_bringup_task(void)
{
    return 0;                            /* 未启用 */
}

#endif
