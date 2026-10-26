#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "speed_control.h"
#include "servo_control.h"
#include "pwm_test_config.h"
static uint32 pwm[4];
void gpio_set_level(gpio_pin_enum pin, int level) { (void)pin; (void)level; }
void pwm_set_duty(pwm_channel_enum channel, uint32 duty)
{
    assert(channel >= 1 && channel <= 3); pwm[channel] = duty;
}
int main(void)
{
    const uint32 centre = (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_M);
    speed_init();
    assert(pwm[1] == 0 && pwm[2] == 0 && pwm[3] == centre);
    speed_apply(0, PWM_TEST_DUTY_PERCENT / DUTY_CMD_MAX, PWM_TEST_DUTY_PERCENT / DUTY_CMD_MAX);
    assert(abs((int)pwm[1] - 3000) <= 1 && abs((int)pwm[2] - 3000) <= 1);
    assert(pwm[3] == centre);
    speed_stop();
    assert(pwm[1] == 0 && pwm[2] == 0 && pwm[3] == centre);
    puts("PASS: actual output layer writes 30% (3000/10000), servo centre and immediate zero");
    return 0;
}
