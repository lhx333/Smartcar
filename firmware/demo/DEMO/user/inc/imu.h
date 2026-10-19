#ifndef _imu_h_
#define _imu_h_

/* ==================================================================================
 *  imu.h —— IMU 驱动层 + 航向解算
 *
 *  陀螺 z 轴 → °/s → 去零偏 + 刻度校正 → 积分成 yaw（弧度，0 = 发车朝向，逆时针为正）。
 *  全工程只有 imu.c 碰陀螺；navigation.c 拿到的 yaw 已经是解算好的。
 *  单位：gyro_z 是 °/s，yaw 系列是弧度。
 * ================================================================================== */

#include "my_common.h"

/* RA/RB/RC 的 SPI 同引脚（SCK C23 / MOSI C22 / MISO C21 / CS C20）。
 * RC 使用原始传感器数据轮询；四元数/欧拉角关闭，不需要 INT2 -> D1 连线。
 * 两者的 *_init() 都是「成功返回 0、失败返回非 0」。 */
#if (IMU_MODEL == IMU_MODEL_RA)

    #define IMU_INIT()          imu660ra_init()
    #define IMU_GET_GYRO()      imu660ra_get_gyro()

    #define IMU_GYRO_Z          imu660ra_gyro_z

    #define IMU_FACTOR_GYRO     imu660ra_transition_factor[1]

#elif (IMU_MODEL == IMU_MODEL_RB)

    #define IMU_INIT()          imu660rb_init()
    #define IMU_GET_GYRO()      imu660rb_get_gyro()

    #define IMU_GYRO_Z          imu660rb_gyro_z

    #define IMU_FACTOR_GYRO     imu660rb_transition_factor[1]

#elif (IMU_MODEL == IMU_MODEL_RC)

    #define IMU_INIT()          imu660rc_init(IMU660RC_QUARTERNION_DISABLE)
    /* 四元数关闭后驱动允许直接轮询读取；每个主循环控制拍读取一次。 */
    #define IMU_GET_GYRO()      imu660rc_get_gyro()
    #define IMU_GYRO_Z          imu660rc_gyro_z
    #define IMU_FACTOR_GYRO     imu660rc_transition_factor[1]

    extern volatile int16 imu_rc_acc[3];
    extern volatile int16 imu_rc_gyro[3];
    void imu_rc_poll_sample(void);

#else
    #error "IMU_MODEL must be IMU_MODEL_RA, IMU_MODEL_RB or IMU_MODEL_RC"
#endif

/* 当前驱动型号。 */
#if (IMU_MODEL == IMU_MODEL_RA)
    #define IMU_MODEL_STR       "RA"
#elif (IMU_MODEL == IMU_MODEL_RB)
    #define IMU_MODEL_STR       "RB"
#else
    #define IMU_MODEL_STR       "RC"
#endif

/* 用驱动自己的因子，不硬编码数值：它会随型号和量程变（RA 2000dps 是 16.4、
 * RB/RC 约为 14.2857，改 1000dps 再翻倍），硬编码就会在换型号/改量程时漏改 ——
 * 后果是 yaw 整体缩放，180° 的 U 型弯走成 206° 或 90°，且不报任何错。 */
#define GYRO_CONV_FACTOR    (1.0f / IMU_FACTOR_GYRO)         /* raw → °/s  */

/* Vehicle convention: positive Z rate = counterclockwise viewed from above.
 * RC mounting verified by a physical CCW 90-degree rotation on 2026-10-19.
 * Calibration and runtime MUST use this same conversion before bias removal.
 * RA/RB retain their original mounting convention. Recheck after remounting. */
#if (IMU_MODEL == IMU_MODEL_RC)
    #define IMU_GYRO_Z_SIGN     (1.0f)
#else
    #define IMU_GYRO_Z_SIGN     (-1.0f)
#endif
#define IMU_Z_TO_DPS(raw) ((float)(raw) * IMU_GYRO_Z_SIGN * GYRO_CONV_FACTOR)

/* 零偏标定采样拍数：本工程控制/轮询周期 5ms，2000 拍约 10s。
 * 标定期间车必须完全静止：拍数越少均值噪声越大，越多越容易被风吹/被踩污染。 */
#define CALIBRATION_SAMPLES (2000)

/* 静止判据：三轴角速度方差（(°/s)^2）都要小于它。100 对应标准差 10°/s，
 * 很宽松 —— 只用来拒绝「标定时人还在碰车」，不负责精度。 */
#define VARIANCE_THRESHOLD  (100.0f)

/* 航向刻度校正系数，默认 1.0。
 * 标定：地面画参考线，把车手动精确转过 180°（或 90°），记下 串口遥测中的航向字段右边的
 *       C（yaw_cont）实测值，YAW_SCALE_CAL = 180.0 / Δyaw实测。
 * 乘在去偏之后、积分之前的 gz 上（见 imu.c），别处不要再补一次。 */
#define YAW_SCALE_CAL       (1.0f)

/* Updated by imu_update(); false after the RC chip fails its periodic presence check. */
bool imu_data_valid(void);

/* 陀螺 z 轴（°/s）。已转换到车体系、已去零偏、已乘 YAW_SCALE_CAL。本层只导出这一个传感器量。 */
extern float gyro_z;

/* 陀螺零偏（°/s）。由上位机请求的标定计算得到。 */
extern float gyro_bias_x, gyro_bias_y, gyro_bias_z;

/* 标定残差标准差（°/s）。 */
extern float gyro_std_x, gyro_std_y, gyro_std_z;

/* 弧度。控制层用度，转换只发生在 state_machine.c 读它们的那一行。 */
extern float yaw;           /* 折到 [-pi, pi]。0 = 发车朝向，逆时针为正 */
extern float yaw_cont;      /* 不折的连续角。验收看它：U 型弯出弯应读到 +pi（180°） */
extern float yaw_rate;      /* rad/s，本拍角速度。调试观测用，控制层不用它 */
extern float yaw_mid;       /* 本拍中点航向，navigation.c 的位置递推直接读它 */
extern float yaw_bias;      /* °/s，残余零偏。积分前扣掉的那个（串口遥测的 BZ） */

extern uint8 is_calibrated;
extern uint8 is_stable;

/* 单次驱动初始化尝试；不自动重试、不自动标定。RC 不依赖 D1/外部中断。 */
void  imu_init(void);

/* 每个控制周期轮询陀螺数据并算出本拍航向（gyro_z / yaw / yaw_cont / yaw_mid）。
 * 必须在 navigation.c 的递推之前调用。 */
void  imu_update(void);
/* Foreground scheduler supplies actual elapsed time after a missed idle tick. */
void  imu_update_elapsed(uint32 elapsed_ms);

/* 航向归零。由 Navigation_Reset() 代为调用，控制层不直接调。
 * ★ yaw_bias 刻意不清零：归零这一刻（正要发车）正是它最准的时候。 */
void  imu_yaw_reset(void);

/* 静止残余零偏的在线更新（时间常数 5s，见 my_common.h 的 YAW_BIAS_ALPHA）。
 * 由控制层只在发车前（ST_ARMED）每拍调用一次，是本工程零偏的主来源。 */
void  imu_bias_track(void);

/* 上位机请求后启动非阻塞零偏标定；每个有效控制拍采一个样本。 */
typedef enum { IMU_CALIB_IDLE = 0, IMU_CALIB_RUNNING, IMU_CALIB_OK, IMU_CALIB_FAILED } imu_calib_state_t;
uint8 imu_calib_start(void);
void  imu_calib_task(void);
int   imu_calib_progress(void);             /* 0..100 */
imu_calib_state_t imu_calib_state(void);
const char *imu_calib_failure_reason(void);
void  imu_calib_commit(void);
void  imu_calib_discard(void);

/* 零偏的 Flash 持久化（空函数，load 恒返回 false）。填上就生效，调用方都不用改。
 * 库里的坑和三条硬约束写在 my_common.h 的 IMU_BIAS_FLASH_ENABLE 一节。 */
void  imu_flash_save_bias(void);
bool  imu_flash_load_bias(void);

#endif
