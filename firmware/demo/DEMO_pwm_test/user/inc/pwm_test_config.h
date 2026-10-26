#ifndef PWM_TEST_CONFIG_H
#define PWM_TEST_CONFIG_H
/* Independent open-loop identification test; no path_planner JSON is read. */
#define PWM_TEST_DUTY_MILLI_PERCENT (30000)
#define PWM_TEST_DURATION_MS        (1000U)
#define PWM_TEST_DUTY_PERCENT       (PWM_TEST_DUTY_MILLI_PERCENT / 1000.0f)
#if PWM_TEST_DUTY_MILLI_PERCENT <= 0 || PWM_TEST_DUTY_MILLI_PERCENT > 100000
#error "PWM test duty must be in (0, 100] percent"
#endif
#if PWM_TEST_DURATION_MS == 0 || PWM_TEST_DURATION_MS % 5 != 0
#error "PWM test duration must be a positive multiple of the 5 ms PIT tick"
#endif
#endif
