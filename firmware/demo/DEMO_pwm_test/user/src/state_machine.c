/* Fixed straight PWM pulse. PendSV owns motor writes; PIT enforces pulse end. */
#include "state_machine.h"
#include "app_runtime.h"
#include "speed_control.h"
#include "navigation.h"
#include "imu.h"
#include "pwm_test_config.h"
#include <math.h>
#include <string.h>

#define EVENT_START     1U
#define EVENT_STOP      2U
#define EVENT_FAULT     4U
#define EVENT_DONE      8U
#define EVENT_READY     16U
#define EVENT_DEBUG     64U
#define EVENT_CALIBRATE 128U
#define REPLY_SLOTS     16U

volatile sm_telemetry_t g_sm;
volatile uint32 sm_reply_dropped, sm_transition_peak_cycles;
static volatile uint32 s_events, s_epoch, s_fault_epoch, s_done_epoch;
static volatile uint8 s_fault_error, s_sensor_ready;
static volatile uint8 s_calibration_pending, s_calibration_active;
static volatile uint32 s_sample_ms, s_pulse_started_ms;
static volatile uint8 s_pulse_active;
static volatile float s_sample_v_l, s_sample_v_r;
static sm_reply_t s_replies[REPLY_SLOTS];
static volatile uint8 s_reply_head, s_reply_tail, s_reply_count;
static uint32 s_control_epoch;
static float s_v_l, s_v_r;
extern int16 encoder_delta[ENCODER_COUNT];

static void pend_transition(void)
{
    __DMB();
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB();
    __ISB();
}
static void post_event(uint32 event)
{
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    s_events |= event;
    pend_transition();
    __set_PRIMASK(mask);
}
uint32 state_machine_epoch(void) { return s_epoch; }
void state_machine_request_start(void) { post_event(EVENT_START); }
void state_machine_request_stop(void) { post_event(EVENT_STOP); }
void state_machine_request_debug(void) { post_event(EVENT_DEBUG); }
void state_machine_request_calibration(void) { post_event(EVENT_CALIBRATE); }
void state_machine_request_joystick(uint8 up, uint8 down, int8 steering)
{ (void)up; (void)down; (void)steering; /* DEBUG/JOY cannot alter this test. */ }
uint8 state_machine_take_calibration_request(void)
{
    uint8 pending;
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    pending = s_calibration_pending;
    s_calibration_pending = 0;
    __set_PRIMASK(mask);
    return pending;
}
void state_machine_set_calibration_active(uint8 active) { s_calibration_active = active; }
void state_machine_request_fault(uint8 error, uint32 epoch)
{
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    if (epoch == s_epoch && !(s_events & EVENT_FAULT))
    {
        s_fault_error = error;
        s_fault_epoch = epoch;
        s_events |= EVENT_FAULT;
        pend_transition();
    }
    __set_PRIMASK(mask);
}
/* Stop invalidates the pulse before publishing the state. No foreground task
 * can subsequently write nonzero motor PWM. */
static void enter_state(sm_state_enum state, uint8 error)
{
    speed_stop();
    s_pulse_active = 0;
    ++s_epoch;
    g_sm.duty_l = g_sm.duty_r = g_sm.turn_cmd = 0.0f;
    g_sm.yaw_target = g_sm.yaw_err = g_sm.speed_target = 0.0f;
    g_sm.debug_mode = 0;
    g_sm.err_code = error;
    g_sm.state = state;
    if (state == ST_RUN)
    {
        /* Apply immediately in this IRQ. Start the 1000 ms clock at the first
         * actual PWM write, independently of foreground work or UART delivery. */
        speed_apply(0.0f, PWM_TEST_DUTY_PERCENT / DUTY_CMD_MAX,
                           PWM_TEST_DUTY_PERCENT / DUTY_CMD_MAX);
        s_pulse_started_ms = app_runtime_now_ms();
        s_pulse_active = 1;
        g_sm.duty_l = g_sm.duty_r = PWM_TEST_DUTY_PERCENT;
    }
}
static void reply_push(uint8 command)
{
    if (s_reply_count >= REPLY_SLOTS) { ++sm_reply_dropped; return; }
    s_replies[s_reply_tail].command = command;
    s_replies[s_reply_tail].state = (uint8)g_sm.state;
    s_replies[s_reply_tail].error = g_sm.err_code;
    s_reply_tail = (uint8)((s_reply_tail + 1U) % REPLY_SLOTS);
    __DMB();
    ++s_reply_count;
}
uint8 state_machine_peek_reply(sm_reply_t *reply)
{
    uint8 available;
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    available = s_reply_count != 0;
    if (available) *reply = s_replies[s_reply_head];
    __set_PRIMASK(mask);
    return available;
}
void state_machine_drop_reply(void)
{
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    if (s_reply_count)
    {
        s_reply_head = (uint8)((s_reply_head + 1U) % REPLY_SLOTS);
        --s_reply_count;
    }
    __set_PRIMASK(mask);
}
void state_machine_transition_irq(void)
{
    uint32 cycle_start = DWT->CYCCNT;
    uint32 events = s_events;
    uint8 start_was_armed = (uint8)(g_sm.state == ST_ARMED);
    uint8 started = 0;
    s_events = 0;
    if (events & EVENT_STOP)
        enter_state(is_calibrated ? ST_ARMED : ST_INIT, SM_ERR_NONE);
    else if ((events & EVENT_FAULT) && s_fault_epoch == s_epoch && g_sm.state != ST_ERROR)
        enter_state(ST_ERROR, s_fault_error);
    else if ((events & EVENT_DONE) && s_done_epoch == s_epoch && g_sm.state == ST_RUN)
        enter_state(ST_DONE, SM_ERR_NONE);
    else if (events & EVENT_CALIBRATE)
    {
        if (g_sm.state == ST_RUN || s_calibration_active) reply_push(4);
        else
        {
            is_calibrated = is_stable = s_sensor_ready = 0;
            s_calibration_active = 1;
            enter_state(ST_INIT, SM_ERR_NO_IMU);
            s_calibration_pending = 1;
            reply_push(3);
        }
    }
    else
    {
        if ((events & EVENT_READY) && g_sm.state == ST_INIT && s_sensor_ready)
            enter_state(ST_ARMED, SM_ERR_NONE);
        if ((events & EVENT_START) && start_was_armed && g_sm.state == ST_ARMED)
        {
            if (!is_calibrated || s_calibration_active)
                enter_state(ST_INIT, SM_ERR_NO_IMU);
            else if (!s_sensor_ready || app_runtime_now_ms() - s_sample_ms > 2U * PIT_D)
                enter_state(ST_ERROR, SM_ERR_IMU_TIMEOUT);
            else if (fabsf(s_sample_v_l) > V_DEADBAND_MPS || fabsf(s_sample_v_r) > V_DEADBAND_MPS)
                enter_state(ST_ERROR, SM_ERR_MOVING_START);
            else { enter_state(ST_RUN, SM_ERR_NONE); started = 1; }
        }
    }
    if (events & EVENT_STOP) reply_push(2);
    if (events & EVENT_START) reply_push(started ? 1 : 6);
    if (events & EVENT_DEBUG) reply_push(7);
    if (DWT->CYCCNT - cycle_start > sm_transition_peak_cycles)
        sm_transition_peak_cycles = DWT->CYCCNT - cycle_start;
    __DSB();
}
void state_machine_watchdog_irq(uint32 now_ms, uint32 control_age_ms)
{
    uint32 epoch = s_epoch;
    if (g_sm.state != ST_RUN) return;
    /* Pulse deadline is PIT-owned, so even a late foreground sample cannot
     * extend it. STOP/fault always wins simultaneous pulse-complete events. */
    if (s_pulse_active && now_ms - s_pulse_started_ms >= PWM_TEST_DURATION_MS)
    {
        uint32 mask = __get_PRIMASK();
        __disable_irq();
        if (epoch == s_epoch && g_sm.state == ST_RUN)
        {
            s_done_epoch = epoch;
            s_events |= EVENT_DONE;
            pend_transition();
        }
        __set_PRIMASK(mask);
    }
    else if (control_age_ms >= CONTROL_WATCHDOG_MS)
        state_machine_request_fault(SM_ERR_CONTROL_OVERRUN, epoch);
}
void state_machine_init(void)
{
    memset((void *)&g_sm, 0, sizeof(g_sm));
    g_sm.state = ST_INIT;
    g_sm.err_code = SM_ERR_NO_IMU;
    s_events = s_epoch = s_fault_epoch = s_done_epoch = 0;
    s_fault_error = s_sensor_ready = 0;
    s_calibration_pending = s_calibration_active = s_pulse_active = 0;
    s_sample_ms = s_pulse_started_ms = 0;
    s_sample_v_l = s_sample_v_r = s_v_l = s_v_r = 0;
    s_reply_head = s_reply_tail = s_reply_count = 0;
    sm_reply_dropped = sm_transition_peak_cycles = 0;
    s_control_epoch = ~0U;
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;
    NVIC_SetPriority(PendSV_IRQn, IRQ_PRIORITY_STATE);
}
void state_machine_control_task(uint32 elapsed_ms, uint32 sample_tick)
{
    uint32 epoch, mask;
    sm_state_enum state;
    float ds, dt = (float)elapsed_ms / 1000.0f;
    uint8 ready = is_calibrated && imu_data_valid() && !s_calibration_active;
    mask = __get_PRIMASK();
    __disable_irq();
    epoch = s_epoch;
    state = g_sm.state;
    __set_PRIMASK(mask);
    if (!elapsed_ms) return;
    s_v_l += SPEED_LPF_A * ((float)encoder_delta[0] * METERS_PER_COUNT / dt - s_v_l);
    s_v_r += SPEED_LPF_A * ((float)encoder_delta[1] * METERS_PER_COUNT / dt - s_v_r);
    mask = __get_PRIMASK();
    __disable_irq();
    s_sample_v_l = s_v_l; s_sample_v_r = s_v_r;
    s_sensor_ready = ready;
    s_sample_ms = sample_tick * (uint32)PIT_D;
    if (epoch == s_epoch) { g_sm.v_l = s_v_l; g_sm.v_r = s_v_r; }
    __set_PRIMASK(mask);
    if (!ready)
    {
        if (state == ST_RUN || state == ST_ARMED)
            state_machine_request_fault(is_calibrated ? SM_ERR_IMU_TIMEOUT : SM_ERR_NO_IMU, epoch);
        return;
    }
    if (state == ST_INIT) { post_event(EVENT_READY); return; }
    if (state == ST_ARMED) { imu_bias_track(); return; }
    if (state == ST_RUN || state == ST_DONE)
    {
        if (state == ST_RUN && s_control_epoch != epoch)
        { Navigation_Reset(); s_control_epoch = epoch; }
        ds = 0.5f * ((float)encoder_delta[0] + (float)encoder_delta[1]) * METERS_PER_COUNT;
        NavigationUpdate(ds, dt);
    }
    /* No PI, launch ramp, steering correction or encoder stall decision here.
     * Zero-encoder tests still deliver exactly the requested bounded pulse. */
}
