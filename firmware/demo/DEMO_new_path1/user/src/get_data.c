/* ==================================================================================
 *  get_data.c —— 传感器采集
 *
 *  只把这一拍的原始读数拿进来，不做换算：编码器读增量并清零、乘 ENC_L/R_SIGN；
 *  ADC 段由 USE_ADC 包住（惯导组不用电感，默认不编译）。
 *  里程换算在 state_machine.c 里做一次，METERS_PER_COUNT 只应该有一个使用者。
 * ================================================================================== */

#include "zf_common_headfile.h"
#include "my_common.h"

/* 本拍两路编码器的增量（0 左 1 右），符号已归一化成「正 = 车向前」。
 * state_machine.c 读这个数组算里程。 */
int16 encoder_delta[ENCODER_COUNT] = {0, 0};

/* 单路的原始值也留着，排查时直接看单路 */
int16 encoder_data_l = 0;
int16 encoder_data_r = 0;

/* 把 ENCx_SIGN 应用到单路读数上（某一路符号反了不报错，只会把里程整体拉偏） */
static int16 apply_sign(int16 value, int8 sign)
{
    return (sign >= 0) ? value : (int16)(-value);
}

// 读取左右两路编码器的增量并清零
// 编码器距离系数由 my_common.h 中的实测距离/count 比例标定。
void get_encoder(void)
{
    encoder_data_l = encoder_get_count(ENCODER_L);          // 读取编码器的值
    encoder_clear_count(ENCODER_L);                         // 清空编码器的值

    encoder_data_r = encoder_get_count(ENCODER_R);
    encoder_clear_count(ENCODER_R);

    encoder_delta[0] = apply_sign(encoder_data_l, ENC_L_SIGN);
    encoder_delta[1] = apply_sign(encoder_data_r, ENC_R_SIGN);
}

#if USE_ADC
// 读取 4 路 ADC 得到四路电感值，error 由第 1 路和第 4 路求差得到
// 电磁组的差比和算法不在此文件里
uint8 channel_index = 0;
adc_channel_enum channel_list[ADC_CHANNEL_NUMBER] =
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4,
};
uint16_t adc_buffer[ADC_CHANNEL_NUMBER];

// 用于差比和运算的电感差值
float adc_error;

void get_adc(void)
{
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
        adc_buffer[channel_index] = adc_convert(channel_list[channel_index]);   // 采集对应的 adc 信号
    }
    adc_error = (float)(adc_buffer[0] - adc_buffer[3]);                         // 得到电感信号计算偏差
}
#endif

// 采集本拍所有传感器数据，主循环每个控制周期调用一次
// 必须在 imu_update() 之前（IMU 驱动自己读传感器，不从这里喂数据）
void get_data(void)
{
    get_encoder();

#if USE_ADC
    get_adc();
#endif
}
