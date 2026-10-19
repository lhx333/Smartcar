#ifndef _speed_control_h_
#define _speed_control_h_

/* ==================================================================================
 *  speed_control.h —— 速度输出层（底盘指令的落地点）
 *
 * 入口 speed_apply()/speed_apply_debug()：{ 转向, 左轮占空比, 右轮占空比 } 三个归一化量，
 *  翻译成舵机角度 + 两路电机 PWM，并负责限幅、转向限速、方向映射。
 *  本层不做闭环，不看编码器和 IMU ——「实际转了多少、跑多快」由 state_machine 负责。
 *
 *  ★ 舵机车和差速车的控制律在上层完全一样，差别只在这三个量怎么执行，
 *    state_machine.c 一行 #if 都不用写，换底盘只改 my_common.h 的 CHASSIS_MODE。
 * ================================================================================== */

#include "my_common.h"

/* 每个控制周期调用一次：把指令落到硬件上。
 *
 *   turn   ∈ [-1, 1]，正 = 左转（逆时针，yaw 增大）。舵机车 → 前轮转角，1.0 对应
 *            CHASSIS_TURN_MAX_DEG；差速车 → 左右轮速度差，1.0 对应 CHASSIS_DIFF_GAIN。
 *   duty_l /
 *   duty_r ∈ [0, 1]，左右后轮各自的归一化占空比，1.0 = MAX_PWM_DUTY 定义的满占空比。
 *            名字是 duty 而不是 speed：本层没有任何测速，进来的就是开环占空比，
 *            真正的速度闭环在 state_machine.c（两轮各自测速 → 各自 PI → 各自占空比）。
 *
 * 内部依次做：三个量的限幅 → 转向方向（CHASSIS_TURN_SIGN）→ 转向逐拍限速
 * → 按 CHASSIS_MODE 落到硬件。
 *
 * ★ 后轮是两个独立电机，同一个占空比转速并不相同。左右两个速度环各自给出自己的
 *   占空比，舵机车下这里原样落到电机上，两轮的差就是自动的电子差速锁。
 *   差速车不能用两个值（那两个环会把转向所需的轮速差抹掉），先合成平均值，
 *   详见 speed_control.c 里的说明。 */
void speed_apply(float turn, float duty_l, float duty_r);
/* DEBUG 手动遥控使用独立的舵机 ±20° 行程，普通路径仍用 CHASSIS_TURN_MAX_DEG。 */
void speed_apply_debug(float turn, float duty_l, float duty_r);

/* 上电初始化：电机停转 + 舵机回中。必须在 Init_All() 之后调用（要求 PWM/GPIO 已初始化）。 */
void speed_init(void);
float speed_applied_servo_offset(void);

/* 全部电机停转 + 舵机回中。用于路径完成、故障停车和蓝牙 STOP 命令。 */
void speed_stop(void);

/* CHASSIS_BRINGUP_MODE 下的点动自检：依次只驱动一路电机，用来核对「电机编号 → 实际轮位」。
 *
 * 返回当前正在点动的电机编号 +1（1..MOTOR_COUNT），0 表示没启用。
 * 启用时由状态中断提交自检输出，需要先发送 START；STOP 仍可立即停车。 */
uint8 speed_bringup_task(void);

#endif
