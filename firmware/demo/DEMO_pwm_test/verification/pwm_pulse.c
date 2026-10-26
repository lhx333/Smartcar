#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "state_machine.h"
#include "app_runtime.h"
#include "navigation.h"
#include "imu.h"
#include "pwm_test_config.h"

mock_scb_t mock_scb;
mock_dwt_t mock_dwt;
mock_debug_t mock_debug;
int16 encoder_delta[ENCODER_COUNT];
uint8 is_calibrated, is_stable;
float yaw, yaw_mid, yaw_rate;
static uint32 irq_mask;
static int in_irq, imu_healthy, drive_writes;
static float left_output, right_output;
static void dispatch(void);

uint32 __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32 mask) { irq_mask = mask; dispatch(); }
void NVIC_SetPriority(IRQn_Type irq, uint32 priority) { (void)irq; (void)priority; }
void pit_ms_init(int channel, uint32 period) { (void)channel; assert(period == PIT_D); }
bool imu_data_valid(void) { return imu_healthy != 0; }
void imu_bias_track(void) {}
void imu_yaw_reset(void) { yaw = yaw_mid = yaw_rate = 0; }
void speed_stop(void) { left_output = right_output = 0; }
void speed_apply(float turn, float left, float right)
{
    assert(in_irq && g_sm.state == ST_RUN && turn == 0.0f);
    assert(left == 1.0f && right == 1.0f); /* Test limit is exactly 30%. */
    left_output = left * DUTY_CMD_MAX;
    right_output = right * DUTY_CMD_MAX;
    ++drive_writes;
}

#include "../user/src/app_runtime.c"
#include "../user/src/state_machine.c"
#include "../user/src/navigation.c"

static void dispatch(void)
{
    if (!irq_mask && !in_irq && (mock_scb.ICSR & SCB_ICSR_PENDSVSET_Msk))
    {
        mock_scb.ICSR = 0;
        in_irq = 1;
        state_machine_transition_irq();
        in_irq = 0;
    }
}
static void foreground(void)
{
    uint32 sample_tick, elapsed;
    assert(app_runtime_take_tick(&sample_tick, &elapsed));
    state_machine_control_task(elapsed, sample_tick);
    app_runtime_control_complete(sample_tick, DWT->CYCCNT);
}
static void tick(void) { app_runtime_tick_irq(); foreground(); }
static void reply_is(uint8 command)
{
    sm_reply_t reply;
    assert(state_machine_peek_reply(&reply) && reply.command == command);
    state_machine_drop_reply();
}
static void setup(void)
{
    irq_mask = in_irq = drive_writes = 0;
    imu_healthy = 1;
    mock_scb.ICSR = 0;
    is_calibrated = is_stable = 0;
    encoder_delta[0] = encoder_delta[1] = 0;
    speed_stop(); state_machine_init(); app_runtime_init(); Navigation_Init(); tick();
}
static void arm(void)
{
    state_machine_request_calibration(); reply_is(3);
    assert(state_machine_take_calibration_request());
    is_calibrated = 1;
    state_machine_set_calibration_active(0);
    tick();
    assert(g_sm.state == ST_ARMED && left_output == 0);
}
static void start(void)
{
    state_machine_request_start(); reply_is(1);
    assert(g_sm.state == ST_RUN && left_output == 30.0f && right_output == 30.0f);
}
int main(void)
{
    uint32 began;
    uint32 i;
    setup();
    state_machine_request_start(); reply_is(6);
    assert(g_sm.state == ST_INIT && drive_writes == 0);
    state_machine_request_debug(); reply_is(7);
    arm(); start(); began = app_runtime_now_ms();
    for (i = 1; i < PWM_TEST_DURATION_MS / PIT_D; ++i)
    {
        if (i == 80) encoder_delta[0] = encoder_delta[1] = 40;
        if (i == 100) { state_machine_request_start(); reply_is(6); }
        if (i == 120) { state_machine_request_debug(); reply_is(7); }
        state_machine_request_joystick(1, 0, 1);
        tick();
        assert(g_sm.state == ST_RUN && left_output == 30.0f && right_output == 30.0f);
        assert(g_sm.turn_cmd == 0 && g_sm.speed_target == 0);
    }
    assert(app_runtime_now_ms() - began == 995);
    /* End is enforced without executing the foreground task at all. */
    app_runtime_tick_irq();
    assert(app_runtime_now_ms() - began == 1000);
    assert(g_sm.state == ST_DONE && left_output == 0 && right_output == 0);
    assert(g_sm.duty_l == 0 && g_sm.duty_r == 0 && drive_writes == 1);
    foreground();
    assert(g_sm.v_l > 0 && g_sm.v_r > 0 && left_output == 0);
    for (i = 0; i < 40; ++i) tick();
    assert(left_output == 0 && g_sm.state == ST_DONE);
    state_machine_request_start(); reply_is(6);
    puts("PASS: immediate 30%, no speed/JOY correction, repeat START cannot extend pulse, PIT cuts to 0 at 1000 ms, coasting telemetry continues");

    setup(); arm(); start();
    for (i = 0; i < PWM_TEST_DURATION_MS / PIT_D; ++i) tick();
    assert(g_sm.state == ST_DONE && g_sm.err_code == 0 && left_output == 0);
    state_machine_request_stop(); reply_is(2);
    tick(); start();
    for (i = 0; i < 80; ++i) tick();
    state_machine_request_stop(); reply_is(2);
    for (i = 0; i < 300; ++i) tick();
    assert(g_sm.state == ST_ARMED && left_output == 0);
    puts("PASS: zero encoders still complete 1 s test; STOP aborts at 400 ms and cannot re-enable PWM");

    setup(); arm(); start();
    for (i = 0; i < CONTROL_WATCHDOG_MS / PIT_D; ++i) app_runtime_tick_irq();
    assert(g_sm.state == ST_ERROR && g_sm.err_code == SM_ERR_CONTROL_OVERRUN && left_output == 0);
    setup(); arm(); start(); imu_healthy = 0; tick();
    assert(g_sm.state == ST_ERROR && g_sm.err_code == SM_ERR_IMU_TIMEOUT && left_output == 0);
    puts("PASS: foreground watchdog and invalid IMU cut the pulse early");

    setup(); is_calibrated = 1; s_sensor_ready = 1;
    __disable_irq(); post_event(EVENT_READY); state_machine_request_start(); __set_PRIMASK(0);
    reply_is(6);
    assert(g_sm.state == ST_ARMED && drive_writes == 0);
    puts("PASS: calibration/readiness must precede START; coalesced READY/START cannot launch");
    return 0;
}
