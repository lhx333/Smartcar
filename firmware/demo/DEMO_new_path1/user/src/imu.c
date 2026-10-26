/* ==================================================================================
 *  imu.c —— IMU 驱动层 + 航向解算
 *
 *  陀螺 z 轴 → °/s → 去零偏 + 刻度校正 → 积分成 yaw。
 *  主循环轮询六轴原始数据；只有陀螺 Z 轴参与航向积分。
 *  [PORT n] 标记的是从 TC377 工程移植时动过的地方。
 * ================================================================================== */

#include "imu.h"                    /* [PORT 1] 它带去 my_common.h 和 zf_common_headfile.h */

#include <math.h>

float gyro_z = 0;      // deg/s in vehicle coordinates; bias removed and YAW_SCALE_CAL applied.

// 零偏（°/s）。三轴一组，但只有 z 参与航向。
#if (IMU_MODEL == IMU_MODEL_RC)
/* 不沿用旧模块零偏；仅在上位机触发标定后计算静态零偏。 */
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
#else
float gyro_bias_x = -0.030, gyro_bias_y = 0.123, gyro_bias_z = -0.147;
#endif
float gyro_std_x = 0, gyro_std_y = 0, gyro_std_z = 0;

// 弧度。yaw 折到 [-pi,pi]；yaw_cont 不折（验收看它，出弯应读到 +pi）；
// yaw_mid 是中点航向，navigation.c 的递推用它；yaw_bias 是残余零偏（°/s）
float yaw = 0, yaw_cont = 0, yaw_rate = 0, yaw_mid = 0, yaw_bias = 0;

uint8 is_calibrated = 0;
uint8 is_stable = 0;

#if (IMU_MODEL == IMU_MODEL_RC)
volatile int16 imu_rc_acc[3] = {0, 0, 0};
volatile int16 imu_rc_gyro[3] = {0, 0, 0};
static volatile uint8 imu_rc_sample_ready = 0;
static volatile uint32 imu_rc_age_ms = IMU_SAMPLE_TIMEOUT_MS;
static uint8 imu_rc_online = 0;
static uint8 imu_rc_initialized = 0;
static uint32 imu_rc_health_check_ms = 0;

/* Quaternion/INT2 mode is disabled. Poll both raw vectors from foreground or
 * calibration loop; the foreground never races a D1 SPI callback anymore. */
void imu_rc_poll_sample(void)
{
    if (!imu_rc_online) return;
    imu660rc_get_acc();
    imu660rc_get_gyro();
    imu_rc_acc[0] = imu660rc_acc_x;
    imu_rc_acc[1] = imu660rc_acc_y;
    imu_rc_acc[2] = imu660rc_acc_z;
    imu_rc_gyro[0] = imu660rc_gyro_x;
    imu_rc_gyro[1] = imu660rc_gyro_y;
    imu_rc_gyro[2] = imu660rc_gyro_z;
    imu_rc_sample_ready = 1;
    imu_rc_age_ms = 0;
}
#endif

void imu_init(void)
{
    /* Probe RC before calling the vendor initializer: its absent-device path
     * polls WHO_AM_I for 256 ms. A fast probe keeps startup telemetry flowing
     * when the sensor is missing; a present device gets the normal setup. */
#if (IMU_MODEL == IMU_MODEL_RC)
    uint8 init_failed;
    spi_init(IMU660RC_SPI, SPI_MODE0, IMU660RC_SPI_SPEED,
             IMU660RC_SPC_PIN, IMU660RC_SDI_PIN, IMU660RC_SDO_PIN, SPI_CS_NULL);
    gpio_init(IMU660RC_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_ms(10);
    init_failed = imu660rc_is_present() ? IMU_INIT() : 1U;
#else
    uint8 init_failed = IMU_INIT();
#endif

#if (IMU_MODEL == IMU_MODEL_RC)
    /* RC raw polling mode does not use the D1/INT2 interrupt. */
    imu_rc_health_check_ms = 0;
    imu_rc_sample_ready = 0;
    imu_rc_age_ms = IMU_SAMPLE_TIMEOUT_MS;
    imu_rc_online = (uint8)(!init_failed && imu660rc_is_present());
    imu_rc_initialized = imu_rc_online;
    if (imu_rc_online)
        imu_rc_poll_sample();
#endif

    gyro_z = 0.0f;
    is_calibrated = 0;
    is_stable = 0;

}

/* Incremental Welford calibration state. Updated only by foreground code. */
static imu_calib_state_t s_calib_state = IMU_CALIB_IDLE;
static uint32 s_calib_count;
static float s_calib_mean[3], s_calib_m2[3];
static const char *s_calib_failure = "NONE";

uint8 imu_calib_start(void)
{
    if (s_calib_state == IMU_CALIB_RUNNING) return 0;
    if (!imu_data_valid())
    {
        s_calib_state = IMU_CALIB_FAILED;
        s_calib_failure = "NO_IMU";
        return 0;
    }
    is_calibrated = 0;
    is_stable = 0;
    gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0f;
    yaw_bias = 0.0f;
    s_calib_count = 0;
    s_calib_mean[0] = s_calib_mean[1] = s_calib_mean[2] = 0.0f;
    s_calib_m2[0] = s_calib_m2[1] = s_calib_m2[2] = 0.0f;
    s_calib_failure = "NONE";
    s_calib_state = IMU_CALIB_RUNNING;
    return 1;
}

void imu_calib_task(void)
{
    float sample[3];
    uint8 axis;
    uint32 count;
    if (s_calib_state != IMU_CALIB_RUNNING) return;
    if (!imu_data_valid())
    {
        s_calib_state = IMU_CALIB_FAILED;
        s_calib_failure = "IMU_LOST";
        return;
    }
#if (IMU_MODEL == IMU_MODEL_RC)
    sample[0] = (float)imu_rc_gyro[0] * GYRO_CONV_FACTOR;
    sample[1] = (float)imu_rc_gyro[1] * GYRO_CONV_FACTOR;
    sample[2] = IMU_Z_TO_DPS(imu_rc_gyro[2]);
#else
    IMU_GET_GYRO();
    sample[0] = sample[1] = 0.0f;
    sample[2] = IMU_Z_TO_DPS(IMU_GYRO_Z);
#endif
    count = ++s_calib_count;
    for (axis = 0; axis < 3; ++axis)
    {
        float delta = sample[axis] - s_calib_mean[axis];
        s_calib_mean[axis] += delta / (float)count;
        s_calib_m2[axis] += delta * (sample[axis] - s_calib_mean[axis]);
    }
    if (count >= CALIBRATION_SAMPLES) imu_calib_commit();
}

int imu_calib_progress(void)
{
    if (s_calib_state == IMU_CALIB_OK) return 100;
    if (s_calib_state != IMU_CALIB_RUNNING) return 0;
    return (int)((s_calib_count * 100U) / CALIBRATION_SAMPLES);
}

imu_calib_state_t imu_calib_state(void) { return s_calib_state; }
const char *imu_calib_failure_reason(void) { return s_calib_failure; }

void imu_calib_commit(void)
{
    uint8 axis;
    float variance[3];
    if (s_calib_state != IMU_CALIB_RUNNING || s_calib_count < CALIBRATION_SAMPLES) return;
    for (axis = 0; axis < 3; ++axis)
    {
        variance[axis] = s_calib_m2[axis] / (float)(s_calib_count - 1U);
        if (!isfinite(s_calib_mean[axis]) || !isfinite(variance[axis]) ||
            variance[axis] > IMU_CALIB_STD_MAX_DPS * IMU_CALIB_STD_MAX_DPS ||
            fabsf(s_calib_mean[axis]) > IMU_CALIB_BIAS_MAX_DPS)
        {
            s_calib_state = IMU_CALIB_FAILED;
            s_calib_failure = "UNSTABLE";
            is_calibrated = 0;
            is_stable = 0;
            return;
        }
    }
    gyro_bias_x = s_calib_mean[0];
    gyro_bias_y = s_calib_mean[1];
    gyro_bias_z = s_calib_mean[2];
    gyro_std_x = sqrtf(variance[0]);
    gyro_std_y = sqrtf(variance[1]);
    gyro_std_z = sqrtf(variance[2]);
    yaw_bias = 0.0f;
    imu_yaw_reset();
    is_calibrated = 1;
    is_stable = 1;
    s_calib_state = IMU_CALIB_OK;
}

void imu_calib_discard(void)
{
    if (s_calib_state == IMU_CALIB_RUNNING)
    {
        s_calib_state = IMU_CALIB_FAILED;
        s_calib_failure = "CANCELLED";
    }
}

/* 折到 [-pi, pi]。只在本文件用 —— 两份回绕实现只要差一个常数，
 * 航向环就会在 ±180° 附近抖。 */
static float imu_wrap_angle(float rad)
{
    while (rad >=  PI_F) rad -= 2.0f * PI_F;
    while (rad <  -PI_F) rad += 2.0f * PI_F;
    return rad;
}

/* ★ yaw_bias 不复位：它是发车前那几十秒估出来的，复位这一刻正是它最准的时候。 */
void imu_yaw_reset(void)
{
    yaw      = 0.0f;
    yaw_cont = 0.0f;
    yaw_rate = 0.0f;
    yaw_mid  = 0.0f;
}

/* 静止残余零偏跟踪。只在发车前（ST_ARMED 分支）每拍调一次 ——
 * 靠速度判「静止」不可靠：赛后在终点把车拎起来时轮子不转，陀螺却看着车在转。 */
void imu_bias_track(void)
{
#if !IMU_USE_PRECALIBRATED_DEFAULTS
    float e = gyro_z - yaw_bias;

    /* 单拍残差超门限 = 车真的在动（或上电标定没跑成），这时不更新 */
    if (fabsf(e) > YAW_BIAS_GATE_DPS) return;

    yaw_bias += YAW_BIAS_ALPHA * e;
#endif
}

bool imu_data_valid(void)
{
#if (IMU_MODEL == IMU_MODEL_RC)
    return imu_rc_online && imu_rc_sample_ready && imu_rc_age_ms < IMU_SAMPLE_TIMEOUT_MS;
#else
    return true;
#endif
}

void imu_update_elapsed(uint32 elapsed_ms)
{
    float dt = (float)elapsed_ms / 1000.0f;
#if (IMU_MODEL == IMU_MODEL_RC)
    /* The D1 IRQ is intentionally unused. Check WHO_AM_I every 100ms so a
     * disconnected sensor still trips the existing IMU timeout/failsafe. */
    imu_rc_health_check_ms += elapsed_ms;
    if (imu_rc_health_check_ms >= IMU_SAMPLE_TIMEOUT_MS)
    {
        imu_rc_health_check_ms = 0;
        if (imu_rc_initialized)
            imu_rc_online = imu660rc_is_present();
    }
    if (!imu_rc_online)
    {
        imu_rc_sample_ready = 0;
        imu_rc_age_ms = IMU_SAMPLE_TIMEOUT_MS;
        return;
    }
    imu_rc_poll_sample();
    if (!imu_data_valid()) return;
#else
    IMU_GET_GYRO();                                     /* [PORT 2] */
#endif

    /* Map sensor Z to the vehicle convention, convert to deg/s, remove the
     * bias measured in that SAME convention, then apply scale before integration.
     * A physical CCW turn must increase yaw. */
    gyro_z = (IMU_Z_TO_DPS(IMU_GYRO_Z) - gyro_bias_z) * YAW_SCALE_CAL;

    /* 这里扣的是**残余**零偏 yaw_bias（gyro_z 出来时已经减过静态零偏了）。
     * 两级顺序不能颠倒 —— 跟踪器估的是「总补偿量」。 */
    yaw_rate = (gyro_z - yaw_bias) * DEG2RAD;

    {
        float d_yaw = yaw_rate * dt;

        /* 中点航向必须在更新 yaw **之前**算：位置递推要的是「本拍起点 + 半个步长」，
         * 用更新后的 yaw 会整整多走一个 d_yaw，而且这个误差是单边的、会一路累加。 */
        yaw_mid = yaw + 0.5f * d_yaw;

        yaw = imu_wrap_angle(yaw + d_yaw);
        yaw_cont += d_yaw;
    }
}

void imu_update(void)
{
    imu_update_elapsed(PIT_D);
}

/* [PORT 5] 零偏的 Flash 持久化（留空，留给参赛队）。
 * 填上就生效，「Flash 里有值就直接用」自然成立，调用方都不用改。
 * 要求和踩坑见 my_common.h 的 IMU_BIAS_FLASH_ENABLE 一节。 */

void imu_flash_save_bias(void)      /* 【空函数，留给参赛队】写 gyro_bias_* 进 Flash */
{
}

bool imu_flash_load_bias(void)      /* 【空函数，留给参赛队】读到有效记录返回 true */
{
    return false;
}
