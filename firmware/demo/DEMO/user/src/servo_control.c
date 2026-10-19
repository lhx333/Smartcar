/* ==================================================================================
 *  servo_control.c —— 舵机输出层
 *
 *  角度换算 + 限幅 + 一路舵机 PWM（C30）。
 *
 *  只在舵机底盘下有实体代码，差速底盘下 servo_set() 是空函数 ——
 *  这样 speed_control.c 可以无条件调它，不用在外面套 #if。
 *
 *  台架自检（CHASSIS_BRINGUP_MODE = 1）下多一个入口 servo_bringup_task()：
 *  它不经过 servo_set()，直接在 L_MAX..R_MAX 之间来回扫。
 * ================================================================================== */

#include "servo_control.h"

#if (CHASSIS_MODE == CHASSIS_STEERING_SERVO)

/* 角度 → C30 舵机 PWM。角度由上层的调用方算好，这里不再限幅。 */
static void servo_write_deg(float deg)
{
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(deg));
}

#endif

void servo_set(float turn)
{
#if (CHASSIS_MODE == CHASSIS_STEERING_SERVO)

    /* 实车方向已确认：舵机指令角增大时前轮左转。
     * turn=+1 表示左转，因此角度取中位加摆角；实际前轮角度由连杆几何决定。 */
    float deg = (float)SERVO_MOTOR_M + turn * (float)CHASSIS_TURN_MAX_DEG;

    /* 活动范围限幅。中位在窗口正中、CHASSIS_TURN_MAX_DEG 又不超过两侧行程时，
     * 这里两边都碰不到 —— my_common.h 里那条 #if 就是保证这一点的，
     * 真被钳住说明那里的条件没满足，表现是某个方向的 turn 全都没反应。 */
    if (deg > (float)SERVO_MOTOR_R_MAX) deg = (float)SERVO_MOTOR_R_MAX;
    if (deg < (float)SERVO_MOTOR_L_MAX) deg = (float)SERVO_MOTOR_L_MAX;

    /* 单路舵机输出到 C30。
     * 角度 → 占空比见 my_common.h 的 SERVO_MOTOR_DUTY。 */
    servo_write_deg(deg);

#else

    /* 差速底盘不用舵机，空实现供调用方无条件调用。
     * 转向方向自检仍要做，差速车靠的是 CHASSIS_DIFF_GAIN 的符号。 */
    (void)turn;

#endif
}

void servo_set_debug(float turn)
{
#if (CHASSIS_MODE == CHASSIS_STEERING_SERVO)
    /* DEBUG manual steering has its own ±20° limit; normal path steering remains
     * limited by CHASSIS_TURN_MAX_DEG. */
    float deg = (float)SERVO_MOTOR_M + turn * (float)DEBUG_TURN_MAX_DEG;
    if (deg > (float)SERVO_MOTOR_R_MAX) deg = (float)SERVO_MOTOR_R_MAX;
    if (deg < (float)SERVO_MOTOR_L_MAX) deg = (float)SERVO_MOTOR_L_MAX;
    servo_write_deg(deg);
#else
    (void)turn;
#endif
}

/* ==================== 舵机自检 ==================== */

#if CHASSIS_BRINGUP_MODE

/* 在正常转向范围 ±CHASSIS_TURN_MAX_DEG 之间来回扫，单程 CHASSIS_BRINGUP_SERVO_TICKS 拍。
 * 返回值是当前角度（度），供主循环显示。没有走限幅，两端就是 L_MAX / R_MAX。 */
float servo_bringup_task(void)
{
#if (CHASSIS_MODE == CHASSIS_STEERING_SERVO)

    static uint32 bringup_servo_tick = 0;

    float half = (float)CHASSIS_BRINGUP_SERVO_TICKS;
    float t    = (float)bringup_servo_tick / half;              /* 0..2 的三角波 */
    float deg;

    if (t > 1.0f) t = 2.0f - t;
    deg = (float)SERVO_MOTOR_M - (float)CHASSIS_TURN_MAX_DEG
        + t * (2.0f * (float)CHASSIS_TURN_MAX_DEG);

    servo_write_deg(deg);

    if (++bringup_servo_tick >= (uint32)CHASSIS_BRINGUP_SERVO_TICKS * 2u)
        bringup_servo_tick = 0;

    return deg;

#else

    return 0.0f;                            /* 差速底盘不驱动舵机 */

#endif
}

#else

float servo_bringup_task(void)
{
    return 0.0f;                            /* 未启用 */
}

#endif
