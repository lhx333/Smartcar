/* Run actual state machine, scheduler and navigation with hardware boundaries
 * mocked. This exercises the observed zero-encoder launch and its deadlines. */
#include <assert.h>
#include <stdio.h>
#include "state_machine.h"
#include "app_runtime.h"
#include "navigation.h"
#include "imu.h"

mock_scb_t mock_scb;
mock_dwt_t mock_dwt;
mock_debug_t mock_debug;
int16 encoder_delta[ENCODER_COUNT];
uint8 is_calibrated, is_stable;
float yaw, yaw_mid, yaw_rate;
static uint32 irq_mask;
static int in_irq;
static float left_output, right_output;
static void dispatch(void);

uint32 __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32 mask) { irq_mask = mask; dispatch(); }
void NVIC_SetPriority(IRQn_Type irq, uint32 priority) { (void)irq; (void)priority; }
void pit_ms_init(int channel, uint32 period) { (void)channel; assert(period == PIT_D); }
bool imu_data_valid(void) { return true; }
void imu_bias_track(void) {}
void imu_yaw_reset(void) { yaw = yaw_mid = yaw_rate = 0; }
void speed_stop(void) { left_output = right_output = 0; }
void speed_apply(float turn, float left, float right)
{
    (void)turn;
    assert(in_irq && g_sm.state == ST_RUN);
    assert(left >= 0 && left <= 1 && right >= 0 && right <= 1);
    left_output = left * DUTY_CMD_MAX;
    right_output = right * DUTY_CMD_MAX;
}
void speed_apply_debug(float turn, float left, float right) { speed_apply(turn, left, right); }

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
static void tick(void)
{
    uint32 sample_tick, elapsed;
    app_runtime_tick_irq();
    assert(app_runtime_take_tick(&sample_tick, &elapsed));
    state_machine_control_task(elapsed, sample_tick);
    app_runtime_control_complete(sample_tick, DWT->CYCCNT);
}
static void expect_reply(uint8 command)
{
    sm_reply_t reply;
    assert(state_machine_peek_reply(&reply) && reply.command == command);
    state_machine_drop_reply();
}
static void setup(void)
{
    irq_mask = in_irq = 0;
    mock_scb.ICSR = 0;
    is_calibrated = is_stable = 0;
    encoder_delta[0] = encoder_delta[1] = 0;
    speed_stop();
    state_machine_init();
    app_runtime_init();
    Navigation_Init();
    tick();
}
static void calibrate_and_start(void)
{
    state_machine_request_calibration();
    assert(state_machine_take_calibration_request());
    expect_reply(3);
    /* Sensor algorithm completion is simulated; gating is production C. */
    is_calibrated = 1;
    state_machine_set_calibration_active(0);
    tick();
    assert(g_sm.state == ST_ARMED && left_output == 0);
    state_machine_request_start();
    expect_reply(1);
    tick();
    assert(g_sm.state == ST_RUN);
    assert(fabsf(left_output - DUTY_START_MIN) < 0.0001f);
}
int main(void)
{
    int i;
    setup();
    state_machine_request_start();
    expect_reply(6);
    assert(g_sm.state == ST_INIT && left_output == 0);
    calibrate_and_start();
    for (i = 0; i < 200; ++i) tick();
    assert(g_sm.state == ST_RUN && left_output > DUTY_CMD_MAX - 0.01f);
    for (; i < MOTOR_STALL_TIMEOUT_MS / PIT_D - 1; ++i) tick();
    assert(g_sm.state == ST_RUN);
    tick();
    assert(g_sm.state == ST_ERROR && g_sm.err_code == SM_ERR_MOTOR_STALL);
    assert(left_output == 0 && right_output == 0);
    state_machine_request_start();
    expect_reply(6);
    assert(g_sm.state == ST_ERROR && left_output == 0);
    puts("PASS: no uncalibrated start, exact first duty, bounded launch ramp, unchanged 2 s stall deadline, latched fault");

    setup();
    calibrate_and_start();
    for (i = 0; i < 50; ++i) tick();
    encoder_delta[0] = encoder_delta[1] = 10;
    tick();
    assert(!s_launch_l.active && !s_launch_r.active);
    assert(g_sm.state == ST_RUN);
    state_machine_request_stop();
    expect_reply(2);
    assert(g_sm.state == ST_ARMED && left_output == 0 && right_output == 0);
    tick();
    assert(left_output == 0 && right_output == 0);
    puts("PASS: forward wheel feedback ends launch assistance; STOP keeps PWM zero");

    setup();
    is_calibrated = 1;
    s_sensor_ready = 1;
    __disable_irq();
    post_event(EVENT_READY);
    state_machine_request_start();
    __set_PRIMASK(0);
    expect_reply(6);
    assert(g_sm.state == ST_ARMED && left_output == 0);
    puts("PASS: coalesced READY and premature START cannot auto-launch");
    return 0;
}
