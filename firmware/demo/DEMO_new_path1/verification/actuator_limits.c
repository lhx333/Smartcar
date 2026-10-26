/* Exercise the actual firmware output functions with captured PWM writes. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "speed_control.h"
#include "servo_control.h"
#include "robust_speed_pi.h"

static uint32 pwm[4];
void gpio_set_level(gpio_pin_enum pin, int level) { (void)pin; (void)level; }
void pwm_set_duty(pwm_channel_enum channel, uint32 duty)
{
    assert(channel >= 1 && channel <= 3);
    pwm[channel] = duty;
}
static void check_servo(float angle)
{
    const uint32 expected = (uint32)SERVO_MOTOR_DUTY(angle);
    assert(pwm[3] == expected);
}
int main(void)
{
    int i;
    float previous = 0.0f;
    const uint32 maximum = (uint32)(VEHICLE_PWM_MAX_PERCENT * PWM_DUTY_MAX / 100.0f);
    const float launch = VEHICLE_PWM_START_PERCENT / VEHICLE_PWM_MAX_PERCENT;
    const uint32 launch_raw = (uint32)(VEHICLE_PWM_START_PERCENT * PWM_DUTY_MAX / 100.0f);
    speed_init();
    check_servo(SERVO_MOTOR_M);
    speed_apply(0.0f, launch, launch);
    assert(abs((int)pwm[1] - (int)launch_raw) <= 1);
    assert(abs((int)pwm[2] - (int)launch_raw) <= 1);
    for (i = 0; i < 100; ++i) speed_apply(100.0f, 100.0f, 100.0f);
    assert(pwm[1] == maximum && pwm[2] == maximum);
    check_servo(SERVO_MOTOR_M + VEHICLE_SERVO_MAX_DEG);
    for (i = 0; i < 100; ++i) speed_apply_debug(-100.0f, 100.0f, 100.0f);
    check_servo(SERVO_MOTOR_M - VEHICLE_SERVO_MAX_DEG);
    servo_set(100.0f);
    check_servo(SERVO_MOTOR_M + VEHICLE_SERVO_MAX_DEG);
    servo_set_debug(-100.0f);
    check_servo(SERVO_MOTOR_M - VEHICLE_SERVO_MAX_DEG);
    servo_set(NAN);
    check_servo(SERVO_MOTOR_M);
    assert(robust_speed_pi(13, 100, 0, &previous, .005f, VEHICLE_PWM_MAX_PERCENT) == VEHICLE_PWM_MAX_PERCENT);
    speed_apply(0.0f, -1.0f, -1.0f);
    assert(pwm[1] == 0 && pwm[2] == 0);
    speed_stop();
    assert(pwm[1] == 0 && pwm[2] == 0);
    check_servo(SERVO_MOTOR_M);
    puts("Actual firmware actuator limits: PASS");
    return 0;
}
