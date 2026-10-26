/* Host-only hardware shim; never added to the Keil include path. */
#ifndef HOST_ZF_COMMON_H
#define HOST_ZF_COMMON_H
typedef unsigned char uint8;
typedef signed char int8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef int gpio_pin_enum;
typedef int pwm_channel_enum;
#define C7 7
#define C9 9
#define PWM2_MODULE0_CHA_C6 1
#define PWM2_MODULE1_CHA_C8 2
#define PWM4_MODULE2_CHA_C30 3
#define PWM_DUTY_MAX 10000
#define GPIO_HIGH 1
#define GPIO_LOW 0
void gpio_set_level(gpio_pin_enum pin, int level);
void pwm_set_duty(pwm_channel_enum channel, uint32 duty);
#endif
