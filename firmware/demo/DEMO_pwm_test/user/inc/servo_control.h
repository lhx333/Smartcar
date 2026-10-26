#ifndef _servo_control_h_
#define _servo_control_h_

/* ==================================================================================
 *  servo_control.h —— 舵机输出层
 *
 *  只做一件事：把归一化的转向指令 turn ∈ [-1, 1] 翻译成舵机角度，限幅之后写到
 *  一路舵机 PWM（C30）上。到这里就到硬件了。
 *  本层不做闭环，也不看编码器和 IMU ——「实际转了多少」由 state_machine.c 的航向环负责。
 *
 *  ★ 只有舵机车（CHASSIS_MODE = CHASSIS_STEERING_SERVO）用得上。
 *    差速车下 servo_set() 是空实现，调用方不需要加 #if。
 * ================================================================================== */

#include "my_common.h"

/* 把转向指令落到舵机上。每个控制周期调用一次。
 *
 *   turn ∈ [-1, 1]，正 = 左转（逆时针，yaw 增大），1.0 对应 CHASSIS_TURN_MAX_DEG。
 *
 * 内部依次做：中位 + turn 摆角 → 活动范围限幅（SERVO_MOTOR_L_MAX / R_MAX）
 * → 折算成 PWM 占空比 → 写三路。
 *
 * ★ 角度全靠 my_common.h 里那三个数（M / L_SPAN / R_SPAN）推出来，
 *   中位落在窗口正中，所以只要摆角不超过两侧行程，限幅就不会生效 ——
 *   那条约束由 my_common.h 的 #if 在编译期保证。
 *   它一旦不成立，那个方向的所有 turn 都会被钳成同一个常数，
 *   舵机在某一段行程里一动不动，看起来像硬件坏了，而且不报任何错。 */
void servo_set(float turn);
/* DEBUG uses the same JSON-configured maximum steering offset as AUTO. */
void servo_set_debug(float turn);

/* CHASSIS_BRINGUP_MODE 下的舵机自检：在中位 ±CHASSIS_TURN_MAX_DEG 之间来回扫。
 *
 * 返回当前角度（度），供主循环显示；未启用或差速底盘时恒返回 0。
 * 它直接调用经过最终角度限幅的 PWM 输出。启用时在 pit_handler 里调用它，
 * 不要再调 servo_set()。 */
float servo_bringup_task(void);

#endif
