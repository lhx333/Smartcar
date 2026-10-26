#ifndef _mycommon_h
#define _mycommon_h

#include "zf_common_headfile.h"
#include "vehicle_params.h"       /* fixed PWM test settings */

/* ==================================================================================
 *  惯导组例程 —— 全部可调参数集中在本文件
 *
 *  任务：上位机 START 后保持舵机中位，固定 30% PWM 输出 1 秒，再清零。
 *
 *  上车前按顺序核对，每一条错了都表现为「车乱跑」而不是报错：
 *    IMU_MODEL / PIT_D / CHASSIS_MODE / MOTOR_LEFT·RIGHT·_SIGN /
 *    ENCODER_CAL_DISTANCE_M·ENCODER_CAL_COUNTS / ENC_L·R_SIGN / CHASSIS_TURN_SIGN /
 *    UART1 蓝牙命令 / gyro_z 符号（俯视把车逆时针转 90°，yaw 必须增大）
 * ================================================================================== */

/* ==================== 系统节拍 ==================== */
 // 使用的中断编号 如果修改 需要同步对应修改中断编号 isr.c 中的调用
#define PIT_CH                      (PIT_CH0 )

/* 控制周期 ms。必须是整数毫秒：imu.c 里有 `3000/PIT_D`（整数除法求拍数），
 * 写成 (5/1000.0f) 会算出 600000 并被 uint16 静默截断。 */
#define PIT_D                       (5)

/* 控制周期 s。由 PIT_D 推出来。 */
#define DT                          ((float)PIT_D / 1000.0f)

/* All configurable interrupts use preemption priorities (group 3).
 * UART1 RX and PendSV share priority zero (highest): RX captures commands
 * promptly; PendSV runs immediately after RX returns to apply state changes. */
#define IRQ_PRIORITY_STATE          (0)
#define IRQ_PRIORITY_COMMAND        (0)
#define IRQ_PRIORITY_TICK           (2)
#define IRQ_PRIORITY_DMA            (3)
#define IRQ_PRIORITY_BACKGROUND     (4)
#define CONTROL_WATCHDOG_MS         (20)
#define UART_TX_TIMEOUT_MS          (100)
#define TELEMETRY_HZ                (200)

#if (PIT_D <= 0) || ((TELEMETRY_HZ * PIT_D) > 1000)
    #error "PIT_D / TELEMETRY_HZ out of range"
#endif

#define LED1 (B9)

/* ==================== 通用数学常量 ==================== */
#define PI_F                        (3.14159265f)
#define RAD2DEG                     (57.2957795131f)
#define DEG2RAD                     (0.01745329251f)

/* ==================== IMU 型号选择 ====================
 * RA/RB/RC 的 SPI 接线：SCK C23 / MOSI C22 / MISO C21 / CS C20。
 * RC 使用原始加速度/角速度轮询模式，不启用四元数，不需要 INT2 -> D1。
 * 各型号使用自己的驱动和量程因子；RC ±2000dps 为 14.2857 LSB/(°/s)。 */
#define IMU_MODEL_RA                (1)
#define IMU_MODEL_RB                (2)
#define IMU_MODEL_RC                (3)
#define IMU_MODEL                   (IMU_MODEL_RC)

/* ==================== 底盘形式选择 ==================== */
#define CHASSIS_STEERING_SERVO      (1)     /* 舵机打前轮转向 */
#define CHASSIS_DIFFERENTIAL        (2)     /* 左右轮差速转向 */
#define CHASSIS_MODE                (CHASSIS_STEERING_SERVO)

/* ==================== 占空比限幅 ==================== */
#define MAX_PWM_DUTY            (VEHICLE_PWM_MAX_PERCENT)

/* 电机占空比上限（百分比）。 */

/* ==================== 电机 ==================== */
#define MOTOR_COUNT                 (2)

#define MOTOR1_DIR               (C7)                         /* DIR1 */
#define MOTOR1_PWM               (PWM2_MODULE0_CHA_C6)       /* PWM1 */

#define MOTOR2_DIR               (C9)                         /* DIR2 */
#define MOTOR2_PWM               (PWM2_MODULE1_CHA_C8)       /* PWM2 */

/* 左右后轮各接在第几路电机上，用点动自检（CHASSIS_BRINGUP_MODE）看实物填。
 * 没被选中的路一律不驱动。 */
#define MOTOR_LEFT                  (1)     /* ← 左轮接的是第几路电机 */
#define MOTOR_RIGHT                 (2)     /* ← 右轮接的是第几路电机 */

/* 点动时给正占空比，轮子必须往前转，往后转就把这一侧改成 -1。
 * ★ 左右填反在**差速底盘**下致命：turn 整体反号 → yaw 环变正反馈 → 指数发散，
 *   且它的表征和「PID 没调好」一模一样。 */
#define MOTOR_LEFT_SIGN             (+1)
#define MOTOR_RIGHT_SIGN            (+1)

#if (MOTOR_LEFT < 1) || (MOTOR_LEFT > MOTOR_COUNT) || (MOTOR_RIGHT < 1) || (MOTOR_RIGHT > MOTOR_COUNT)
    #error "MOTOR_LEFT / MOTOR_RIGHT 必须是 1..MOTOR_COUNT 的编号（就是点动自检时看到的那个数字）"
#endif
#if (MOTOR_LEFT == MOTOR_RIGHT)
    #error "MOTOR_LEFT 和 MOTOR_RIGHT 填了同一个编号：同一路电机不能既当左轮又当右轮"
#endif

/* ==================== 编码器：左轮 C0/C1，右轮 C2/C24 ==================== */
#define ENCODER_COUNT               (2)

#define ENCODER_L                   (QTIMER1_ENCODER1)          /* 左轮 */
#define ENCODER_L_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_L_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_R                   (QTIMER1_ENCODER2)          /* 右轮 */
#define ENCODER_R_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_R_B                 (QTIMER1_ENCODER2_CH2_C24)

/* 里程换算：以实际推行距离直接标定，包含轮胎有效周长与编码器解码倍率。
 * 本次实测推行 0.90m，编码器增加 9503 count；之后测速和路径里程都使用该比例。
 * 更换轮胎、编码器/减速比或解码配置后，应重新实测更新这两个标定值。 */
#define ENCODER_CAL_DISTANCE_M      (0.90f)
#define ENCODER_CAL_COUNTS          (9503.0f)
#define METERS_PER_COUNT            (ENCODER_CAL_DISTANCE_M / ENCODER_CAL_COUNTS)

/* 手推车向前走时两路 count 都必须为正，哪路为负就改成 -1。
 * 台架上也可以标：CHASSIS_BRINGUP_MODE = 1 点动，观察串口 RAW 读数、
 * 目视该轮向前转时为正就该填 +1。
 * ★ 单路反了不报错，只把里程拉偏（平均后偏一半），看起来像轮径没标对。
 * 航迹推算取两路平均，速度环分别用两路，见「速度环参数」。 */
#define ENC_L_SIGN                  (-1)
#define ENC_R_SIGN                  (+1)

/* ==================== 电感信号 ADC（本组不使用） ==================== */
#define USE_ADC                     (0)

#define ADC_CHANNEL_NUMBER          (4) //使用的电感个数

#define ADC_CHANNEL1            (ADC1_CH3_B14)
#define ADC_CHANNEL2            (ADC1_CH4_B15)
#define ADC_CHANNEL3            (ADC1_CH10_B21)
#define ADC_CHANNEL4            (ADC1_CH12_B23)

/* ==================== 舵机 ==================== */
#define SERVO_MOTOR1_PWM             (PWM4_MODULE2_CHA_C30)                         // 舵机 PWM 输出，仅使用 C30

#define SERVO_MOTOR_FREQ            (50 )                                           // 舵机接上舵机的频率  注意范围 50-300

/* ★ 只有下面三个数要动，其余全部由它们推出来：
 *      中位 = SERVO_MOTOR_M                    校准中位只改这一个
 *      行程 = [M − L_SPAN, M + R_SPAN]          改行程只改这两个（左右可以不一样宽）
 *   单位就是舵机角度（0..180）。M 填「前轮正的时候舵机该在的角度」，
 *   舵机臂装偏多少已经含在这个数里，不再另外补零偏。
 * ★ 怎么量 M：CHASSIS_BRINGUP_MODE = 1 让前轮扫起来，等前轮正的那一瞬间
 *   观察串口中的舵机角度后填入，再编译一次微调（眼睛大约差 1~2°）。 */
#define SERVO_MOTOR_M               (99.0f)

#define SERVO_MOTOR_L_SPAN          (20)                                            // 中位往左（角度更小那一侧）还能走几度
#define SERVO_MOTOR_R_SPAN          (20)                                            // 中位往右（角度更大那一侧）还能走几度

/* 由上面三个推出来的活动范围。写代码只用这两个名字，不要去直接改。 */
#define SERVO_MOTOR_L_MAX           (SERVO_MOTOR_M - (float)SERVO_MOTOR_L_SPAN)
#define SERVO_MOTOR_R_MAX           (SERVO_MOTOR_M + (float)SERVO_MOTOR_R_SPAN)

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif

/* ==================== 底盘输出参数 ==================== */

/* turn=±1 时舵机指令角相对中位的偏移；实测指令角增大对应车轮左转。 */
#define CHASSIS_TURN_MAX_DEG        (VEHICLE_SERVO_MAX_DEG)
#define DEBUG_TURN_MAX_DEG          (VEHICLE_SERVO_MAX_DEG)

/* 摆角必须放得进行程里，否则那一侧的 turn 会被限幅钳成同一个常数、舵机不动。 */
#if (VEHICLE_SERVO_MAX_MILLIDEG > SERVO_MOTOR_L_SPAN * 1000) || (VEHICLE_SERVO_MAX_MILLIDEG > SERVO_MOTOR_R_SPAN * 1000)
    #error "Configured servo limit exceeds calibrated travel"
#endif

/* turn=+1 时车应向左转（逆时针，yaw 增大）。反了改为 -1。 */
#define CHASSIS_TURN_SIGN           (+1)

/* 每拍（5ms）允许的 turn 变化量，防止舵机齿轮被阶跃指令打坏。
 * 0.04 约等于满行程 250ms。 */
#define CHASSIS_TURN_SLEW_MAX       (0.04f)

/* ---------- 差速转向 ---------- */
/* CHASSIS_DIFF_GAIN * turn = 差速量。台架标定：令 turn=1、speed=1，测左右轮转速比反推。
 * !! R=1m 这种大半径弯必须走弧线（两轮同向、速度不同），不能原地转：
 *    几何关系 (vR-vL)/(vR+vL) = 轮距/(2R)，轮距 0.18m 时只需 9% 速度差。
 *    原地转会让内侧轮被反向拖动，里程与航向脱钩，弧长判据直接失效。 */
#define CHASSIS_DIFF_GAIN           (0.60f)
#define CHASSIS_DIFF_TRIM           (0.0f)

/* ---------- 台架自检（不上赛道就能动电机和舵机的入口）----------
 * 置 1 后依次只驱动 MOTOR1→2→3→4，串口状态字报告当前编号，据此确认哪个轮子在转。
 * 同时舵机在中位 ±CHASSIS_TURN_MAX_DEG 之间来回扫，串口状态帧提供当前舵机角度，
 * 两路编码器在这 2s 内的累计增量（原始 / 乘完 ENC_L/R_SIGN）也可从编码器遥测查看。
 * 用来确认「电机编号 → 实际轮位」的映射、编码器的接线与符号、舵机的接线方向行程；
 * 这一步没确认就往下做，后面所有标定（编码器符号、左右映射、转向方向）都建在错误的基础上。
 * 顺带也是标 DUTY_START_MIN 的工具。!! 车是真的会走，先架起来，标完改回 0。 */
#define CHASSIS_BRINGUP_MODE        (0)
#define CHASSIS_BRINGUP_TICKS       (400)   /* 每路持续拍数，400×5ms = 2s */

/* 归一化占空比（0..1），**不是** DUTY_START_MIN 那个百分比：
 *      点动自检实际占空比 = CHASSIS_BRINGUP_DUTY × MAX_PWM_DUTY
 * 固定自检为 7.5%，并受配置的 MAX_PWM_DUTY 上限约束。 */
#define CHASSIS_BRINGUP_DUTY        (7.5f / (float)MAX_PWM_DUTY)

/* 舵机自检单程时长（拍）。400×5ms = 2s，从中位减最大配置摆角扫到加最大摆角。
 * 自检也受 servo_write_deg 的最终配置角度限幅。 */
#define CHASSIS_BRINGUP_SERVO_TICKS (400)

/* ==================== 路径（Pure Pursuit 前视跟踪）====================
 * 路径、速度和前视参数来自自动生成的 robust_trajectory.h。
 * PP_LOOKAHEAD_M 仅为旧接口保留，新跟踪器使用配置及轨迹表的动态前视。 */

/* 前视距离 L（m），Pure Pursuit 唯一的形状参数：
 * 投影到当前路径段 → 沿路径弧长向前 L 米插值 → 目标航向指向插值点。
 * ★ 调它的现象：弯里切内道、出弯往外偏 → 调小；直线上方向小幅来回摆 → 调大。
 * ★ 和 KP_YAW 别一起调：L 决定往哪打，KP_YAW 决定打多少。 */
#define PP_LOOKAHEAD_M              (0.40f)

/* ==================== 导航 ==================== */
/* 到路径终点的到达半径/终点线横向容差，须推进至路径末段才判定。
 * ★ 路径的起点和终点不能离得比它更近，否则一发车就判「到了」。 */
#define NAV_ARRIVE_R_M              (0.10f)

/* ==================== 航向零偏跟踪（imu.c 的 imu_bias_track）====================
 * 时间常数 = DT/alpha = 5s。门限 1.0°/s 很宽：真实零偏是 0.1°/s 量级。 */
#define YAW_BIAS_ALPHA              (0.001f)    /* 静止残余零偏更新率，时间常数 5s */
#define YAW_BIAS_GATE_DPS           (1.0f)      /* 残余零偏更新门限，超过就认为车在动 */

/* PWM 测试使用 pwm_test_config.h 的固定占空比，无速度闭环或起步爬升。 */
#define DUTY_START_MIN              (VEHICLE_PWM_START_PERCENT)
#define V_CRUISE_MPS                (1.00f)
#define DEBUG_MAX_SPEED_MPS         (2.00f)
#define DEBUG_ACCEL_MPS2            (0.50f)
#define DEBUG_DECEL_MPS2            (0.80f)
#define DEBUG_SPEED_KI              (4.0f)
#define DEBUG_LINK_TIMEOUT_MS       (300U)
#define SPEED_KI                    (1.0f)
#define SPEED_DUTY_DIFF_MAX         (6.0f)
#define DUTY_CMD_MAX                ((float)MAX_PWM_DUTY)
#define SPEED_LPF_A                 (0.30f)
#define V_DEADBAND_MPS              (0.03f)

#if VEHICLE_PWM_START_MILLI_PERCENT > VEHICLE_PWM_MAX_MILLI_PERCENT
    #error "Configured launch PWM exceeds maximum PWM"
#endif

/* Wheel/servo/control defaults are accepted; gyro bias is measured on host request.
 * Set the following flag to 1 only to explicitly use compiled gyro biases. */
#define IMU_USE_PRECALIBRATED_DEFAULTS (0)
#define IMU_CALIB_STD_MAX_DPS        (0.5f)
#define IMU_CALIB_BIAS_MAX_DPS       (5.0f)
#define IMU_SAMPLE_TIMEOUT_MS       (100)
#define MOTOR_STALL_TIMEOUT_MS      (2000)
#define RUN_TIMEOUT_MS              (120000)

/* ==================== 航向环参数（P + D，一套增益，直行弯道共用）====================
 * 只有 P 和 D。加 I 会变成三阶，车身以 1~2s 周期缓慢画龙；稳态机械零偏交给舵机中位补。
 * D 取陀螺仪角速度，不取 e_yaw 的差分（差分会把位置噪声放大再喂进来）。
 * ★ 调参：弯里切内道 → 加大 KP_YAW；直线上 1~2s 周期画龙 → 加大 KD_YAW。 */
#define KP_YAW                      (0.030f)
#define KD_YAW                      (0.004f)

/* Runtime checks stop on stale IMU data, prolonged low/reverse wheel speed,
 * or task timeout. STOP clears the fault; START never clears it. */

/* ==================== UART 控制协议 ====================
 * 上位机向 UART1 发送 ASCII 命令并以 CR/LF 结束：
 *   START：仅在 ST_ARMED 时归零导航并发车
 *   STOP ：立即停车
 *   CALIBRATE：在非 RUN 状态开始陀螺仪零偏标定
 *   DEBUG：进入上位机按钮遥控模式（先完成 IMU 标定）
 *   JOY,u,d,t：遥控心跳；u=上键加速，d=下键减速，t=转向(-1/0/+1)
 * UART1 为 B12(TX)/B13(RX)，E35 模块 TX 接 B13，RX 接 B12，共地。 */

/* 零偏只在上位机发送 CALIBRATE 后采集；上电不自动标定。
 * 2000 个 5ms 样本约 10 秒，标定时车辆必须静止。 */

/* ==================== 陀螺零偏的 Flash 持久化（当前禁用）==================== */
/* 本工程当前不持久化标定值，每次上电需由上位机重新发 CALIBRATE。
 * 若以后启用 Flash 持久化，需遵守三条硬约束：
 *   1. 扇区 0 是 boot header，写下去程序直接不启动。
 *   2. 擦写会 interrupt_global_disable() 几十毫秒 —— 绝不能在控制中断里做，
 *      不可放在控制中断。
 *   3. 存的值必须带 magic + 量程校验。读到半写坏的数据还当有效零偏用，
 *      表现是航向持续单向漂、且不报错。
 * 另：flash_init() 在库里被注释掉了（zf_common_clock.c:86），存取前要自己调一次。 */
#define IMU_BIAS_FLASH_ENABLE       (0)

/* 存到哪个扇区。RT1064 是 4MB = 128 个 32K 扇区。默认 127 是本库约定，离镜像最远。
 * **千万别用 0** —— 那是 boot header，写下去程序直接不启动。 */
#define IMU_FLASH_SECTOR            (127)

#endif
