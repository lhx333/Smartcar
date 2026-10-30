/* Priority-0 state transitions and foreground control have separate ownership.
 * No navigation or sensor bus accesses occur in PendSV. */
#include "state_machine.h"
#include "app_runtime.h"
#include "speed_control.h"
#include "robust_speed_pi.h"
#include "motor_launch.h"
#include "component.h"
#include "straight_control.h"
#if CHASSIS_BRINGUP_MODE || IMU_USE_PRECALIBRATED_DEFAULTS
#error "Component tests require host calibration and explicit START; disable bringup/precalibrated defaults"
#endif
static float robust_prev_l, robust_prev_r, robust_prev2_l, robust_prev2_r;
static motor_launch_t s_launch_l, s_launch_r;
static float path_settle, path_wait;
#include "servo_control.h"
#include "navigation.h"
#include "imu.h"
#include <math.h>
#include <string.h>

#define EVENT_START 1U
#define EVENT_STOP  2U
#define EVENT_FAULT 4U
#define EVENT_DONE  8U
#define EVENT_READY 16U
#define EVENT_OUTPUT 32U
#define EVENT_DEBUG 64U
#define EVENT_CALIBRATE 128U
#define REPLY_SLOTS 16U

volatile sm_telemetry_t g_sm;
volatile uint32 sm_reply_dropped;
static volatile uint32 s_events, s_epoch, s_fault_epoch, s_done_epoch;
static volatile uint8 s_fault_error;
static volatile uint32 s_run_started_ms;
static volatile float s_request_pwm, s_request_hold, s_active_pwm, s_active_hold;
static volatile float s_request_turn, s_request_v1, s_request_v2;
static volatile float s_active_turn, s_active_v1, s_active_v2;
static volatile float s_request_servo,s_active_servo;
static volatile uint8 s_pwm_config_ready;
static volatile uint8 s_sensor_ready, s_debug_mode;
static volatile uint8 s_calibration_pending, s_calibration_active;
static volatile uint8 s_joy_accelerate, s_joy_decelerate;
static volatile int8 s_joy_steering;
static volatile uint32 s_joy_last_ms;
static volatile uint32 s_sample_ms;
static volatile float s_sample_v_l, s_sample_v_r;
static sm_reply_t s_replies[REPLY_SLOTS];
static volatile uint8 s_reply_head, s_reply_tail, s_reply_count;

/* Foreground owns these. The IRQ invalidates them with an epoch change. */
static uint32 s_control_epoch;
static float s_duty_l, s_duty_r, s_v_l, s_v_r;
static uint32 s_stall_l_ms, s_stall_r_ms;
static uint8 s_output_applied;
static float s_debug_target_mps;
static volatile struct
{
    uint32 epoch, tick;
    float turn, duty_l, duty_r, target, error, speed_target;
} s_output;
volatile uint32 sm_transition_peak_cycles;
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
uint8 state_machine_request_start(void) {
    uint32 mask = __get_PRIMASK();
    __disable_irq();
#if C_MODULE == 1 || C_MODULE == 4
    if (!s_pwm_config_ready || g_sm.state != ST_ARMED || (s_events & EVENT_STOP)) {
        __set_PRIMASK(mask); return 0;
    }
#endif
    s_events |= EVENT_START;
    pend_transition();
    __set_PRIMASK(mask);
    return 1;
}
uint8 state_machine_configure_circle(int8 turn_deg, uint8 servo_deg, uint8 v1_tenths, uint8 v2_tenths)
{
#if C_MODULE == 4
    uint32 mask;
    if (turn_deg < -90 || turn_deg > 90 || (turn_deg > -5 && turn_deg < 5) || servo_deg < 2 || servo_deg > VEHICLE_SERVO_MAX_DEG || v1_tenths < 2 || v1_tenths > 10 ||
        v2_tenths < 2 || v2_tenths > 10) return 0;
    mask = __get_PRIMASK(); __disable_irq();
    if (g_sm.state != ST_ARMED || (s_events & (EVENT_STOP | EVENT_CALIBRATE))) {
        __set_PRIMASK(mask); return 0;
    }
    s_request_turn=(float)turn_deg;
    s_request_servo=(float)servo_deg;
    s_request_v1=(float)v1_tenths/10.0f;
    s_request_v2=(float)v2_tenths/10.0f;
    s_pwm_config_ready=1;
    __set_PRIMASK(mask);
    return 1;
#else
    (void)turn_deg; (void)servo_deg; (void)v1_tenths; (void)v2_tenths;
    return 0;
#endif
}
uint8 state_machine_request_start_pwm(uint8 pwm_percent, uint8 hold_s)
{
#if C_MODULE == 1
    if (pwm_percent < 10 || pwm_percent > 40 || hold_s < 2 || hold_s > 5) return 0;
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    if (g_sm.state != ST_ARMED || (s_events & (EVENT_STOP | EVENT_CALIBRATE))) {
        __set_PRIMASK(mask); return 0;
    }
    s_request_pwm = (float)pwm_percent;
    s_request_hold = (float)hold_s;
    s_pwm_config_ready = 1;
    __set_PRIMASK(mask);
    return 1;
#else
    (void)pwm_percent; (void)hold_s;
    return 0;
#endif
}
void state_machine_request_debug(void) { post_event(EVENT_DEBUG); }
void state_machine_request_stop(void) { post_event(EVENT_STOP); }
void state_machine_request_calibration(void) { post_event(EVENT_CALIBRATE); }

void state_machine_request_joystick(uint8 accelerate, uint8 decelerate, int8 steering)
{
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    if (g_sm.state == ST_RUN && s_debug_mode)
    {
        s_joy_accelerate = accelerate;
        s_joy_decelerate = decelerate;
        s_joy_steering = steering;
        s_joy_last_ms = app_runtime_now_ms();
        __DMB();
    }
    __set_PRIMASK(mask);
}

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

void state_machine_set_calibration_active(uint8 active)
{
    s_calibration_active = active;
}

void state_machine_request_fault(uint8 error, uint32 epoch)
{
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    /* A preempted old control cycle must not fault a new run. */
    if (epoch == s_epoch && !(s_events & EVENT_FAULT))
    {
        s_fault_error = error;
        s_fault_epoch = epoch;
        s_events |= EVENT_FAULT;
        pend_transition();
    }
    __set_PRIMASK(mask);
}

static void request_done(uint32 epoch)
{
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    if (s_epoch == epoch)
    {
        s_done_epoch = epoch;
        s_events |= EVENT_DONE;
        pend_transition();
    }
    __set_PRIMASK(mask);
}

/* Only PendSV calls this. Stop first; invalidate in-flight work; publish state.
 * Reset/planning happens in the next foreground tick. */
static void enter_state(sm_state_enum state, uint8 error)
{
    speed_stop();
    ++s_epoch;
    g_sm.duty_l = g_sm.duty_r = g_sm.turn_cmd = 0.0f;
    g_sm.yaw_target = g_sm.yaw_err = 0.0f;
    g_sm.speed_target = 0.0f;
    g_sm.debug_mode = 0;
    g_sm.err_code = error;
    g_sm.state = state;
    if (state != ST_RUN) s_debug_mode = 0;
    if (state != ST_ARMED) s_pwm_config_ready = 0;
    if (state == ST_RUN) s_run_started_ms = app_runtime_now_ms();
}

static void reply_push(uint8 command)
{
    if (s_reply_count >= REPLY_SLOTS)
    {
        ++sm_reply_dropped;
        return;
    }
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
    uint8 debug_started = 0;
    uint8 start_was_armed = (uint8)(g_sm.state == ST_ARMED);
    uint8 start_accepted = 0;
    s_events = 0; /* All producers have lower priority. */
    /* STOP wins coalesced events and clears faults. START never clears faults
     * and a repeated START never resets a running path. */
    if (events & EVENT_STOP)
    {
        s_pwm_config_ready = 0;
        enter_state(is_calibrated ? ST_ARMED : ST_INIT, SM_ERR_NONE);
    }
    else if ((events & EVENT_FAULT) && s_fault_epoch == s_epoch && g_sm.state != ST_ERROR)
        enter_state(ST_ERROR, s_fault_error);
    else if ((events & EVENT_DONE) && s_done_epoch == s_epoch && g_sm.state == ST_RUN)
    {
        enter_state(ST_DONE, SM_ERR_NONE); /* PWM is zero before completion is queued. */
    }
    else if (events & EVENT_CALIBRATE)
    {
        if (g_sm.state == ST_RUN || s_calibration_active)
            reply_push(4); /* REJECTED/BUSY; never interrupt an active run/calibration. */
        else
        {
            s_calibration_active = 1;
            /* Invalidate the old readiness before the foreground samples; an
             * already-pending READY event must not re-arm during recalibration. */
            is_calibrated = 0;
            is_stable = 0;
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
            if (!is_calibrated)
                enter_state(ST_INIT, SM_ERR_NO_IMU);
            else if (!s_sensor_ready || app_runtime_now_ms() - s_sample_ms > 2U * PIT_D)
                enter_state(ST_ERROR, SM_ERR_IMU_TIMEOUT);
            else if (fabsf(s_sample_v_l) > V_DEADBAND_MPS || fabsf(s_sample_v_r) > V_DEADBAND_MPS)
                enter_state(ST_ERROR, SM_ERR_MOVING_START);
            else
            {
                s_active_pwm = s_request_pwm;
                s_active_hold = s_request_hold;
                s_active_turn=s_request_turn;
                s_active_servo=s_request_servo;
                s_active_v1=s_request_v1;
                s_active_v2=s_request_v2;
                s_pwm_config_ready = 0;
                s_debug_mode = 0; /* Component firmware only accepts START. */
                debug_started = s_debug_mode;
                s_joy_accelerate = s_joy_decelerate = 0;
                s_joy_steering = 0;
                s_joy_last_ms = app_runtime_now_ms();
                enter_state(ST_RUN, SM_ERR_NONE);
                if (events & EVENT_START) start_accepted = 1;
            }
        }
    }
    /* Foreground publishes a complete output packet; only this IRQ touches
     * motor/servo outputs after boot. STOP/fault/done invalidate its epoch.
     * There is no foreground PWM write that can resume after a STOP. */
    if ((events & EVENT_OUTPUT) && s_output.epoch == s_epoch &&
        g_sm.state == ST_RUN && s_output.tick == app_runtime_epoch_ticks())
    {
#if CHASSIS_BRINGUP_MODE
        (void)speed_bringup_task();
        (void)servo_bringup_task();
#else
        if (s_debug_mode)
            speed_apply_debug(s_output.turn, s_output.duty_l / DUTY_CMD_MAX,
                              s_output.duty_r / DUTY_CMD_MAX);
        else if (C_MODULE == 1)
            speed_apply_pwm_test(s_output.turn, s_output.duty_l, s_output.duty_r);
        else
            speed_apply(s_output.turn, s_output.duty_l / DUTY_CMD_MAX,
                        s_output.duty_r / DUTY_CMD_MAX);
#endif
        g_sm.yaw_target = s_output.target; g_sm.yaw_err = s_output.error;
        g_sm.turn_cmd = s_output.turn;
        g_sm.speed_target = s_output.speed_target;
        g_sm.debug_mode = s_debug_mode;
        g_sm.duty_l = s_output.duty_l; g_sm.duty_r = s_output.duty_r;
    }
    if (events & EVENT_STOP) reply_push(2);
    if (events & EVENT_START)
        reply_push(start_accepted ? 1 : 6); /* Explicitly reject every premature/unsafe START. */
    if (events & EVENT_DEBUG)
        reply_push(debug_started ? 5 : 7);
    if (DWT->CYCCNT - cycle_start > sm_transition_peak_cycles)
        sm_transition_peak_cycles = DWT->CYCCNT - cycle_start;
    __DSB();
}

void state_machine_watchdog_irq(uint32 now_ms, uint32 control_age_ms)
{
    uint32 epoch = s_epoch;
    if (g_sm.state != ST_RUN) return;
    if (control_age_ms >= CONTROL_WATCHDOG_MS)
        state_machine_request_fault(SM_ERR_CONTROL_OVERRUN, epoch);
    else if (now_ms - s_run_started_ms >= RUN_TIMEOUT_MS)
        state_machine_request_fault(SM_ERR_RUN_TIMEOUT, epoch);
}

void state_machine_init(void)
{
    memset((void *)&g_sm, 0, sizeof(g_sm));
    g_sm.state = ST_INIT;
    g_sm.err_code = SM_ERR_NO_IMU;
    s_events = s_epoch = s_fault_epoch = s_done_epoch = 0;
    s_sensor_ready = 0;
    s_debug_mode = 0;
    s_calibration_pending = s_calibration_active = 0;
    s_joy_accelerate = s_joy_decelerate = 0;
    s_joy_steering = 0;
    s_joy_last_ms = 0;
    s_sample_ms = s_run_started_ms = 0;
    s_sample_v_l = s_sample_v_r = 0;
    s_request_pwm = s_active_pwm = C_PWM_HIGH;
    s_request_hold = s_active_hold = C_PWM_HOLD_HIGH;
    s_request_turn=s_active_turn=C_CIRCLE_TARGET_YAW;
    s_request_servo=s_active_servo=C_CIRCLE_TURN;
    s_request_v1=s_active_v1=C_CIRCLE_V1;
    s_request_v2=s_active_v2=C_CIRCLE_V2;
    s_pwm_config_ready = 0;
    s_reply_head = s_reply_tail = s_reply_count = 0;
    sm_reply_dropped = 0;
    sm_transition_peak_cycles = 0;
    s_control_epoch = ~0U;
    s_duty_l = s_duty_r = s_v_l = s_v_r = 0;
    s_stall_l_ms = s_stall_r_ms = 0;
    s_output_applied = 0;
    s_debug_target_mps = 0.0f;
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;
    NVIC_SetPriority(PendSV_IRQn, IRQ_PRIORITY_STATE);
}

static float wrap180(float value)
{
    while (value >= 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}

static uint8 publish_output(uint32 epoch, uint32 tick, float turn,
                            float duty_l, float duty_r, float target, float error, float speed_target)
{
    uint8 published = 0;
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    if (epoch == s_epoch && g_sm.state == ST_RUN &&
        !(s_events & (EVENT_STOP | EVENT_FAULT | EVENT_DONE)) &&
        app_runtime_epoch_ticks() == tick)
    {
        s_output.epoch = epoch; s_output.tick = tick;
        s_output.turn = turn;
        s_output.duty_l = duty_l; s_output.duty_r = duty_r;
        s_output.target = target; s_output.error = error;
        s_output.speed_target = speed_target;
        s_events |= EVENT_OUTPUT;
        published = 1;
        pend_transition();
    }
    __set_PRIMASK(mask);
    return published;
}

void state_machine_control_task(uint32 elapsed_ms, uint32 sample_tick)
{
    uint32 epoch, mask;
    sm_state_enum state;
    uint8 debug_mode, joy_accelerate, joy_decelerate;
    int8 joy_steering;
    uint32 joy_last_ms, now_ms;
    float dt = (float)elapsed_ms / 1000.0f;
    float turn, target, speed_target, error, mean, diff;
    uint8 ready = is_calibrated && imu_data_valid();

    mask = __get_PRIMASK();
    __disable_irq();
    epoch = s_epoch;
    state = g_sm.state;
    debug_mode = s_debug_mode;
    joy_accelerate = s_joy_accelerate;
    joy_decelerate = s_joy_decelerate;
    joy_steering = s_joy_steering;
    joy_last_ms = s_joy_last_ms;
    __set_PRIMASK(mask);
    now_ms = app_runtime_now_ms();

    s_v_l += SPEED_LPF_A * ((float)encoder_delta[0] * METERS_PER_COUNT / dt - s_v_l);
    s_v_r += SPEED_LPF_A * ((float)encoder_delta[1] * METERS_PER_COUNT / dt - s_v_r);
    mask = __get_PRIMASK();
    __disable_irq();
    s_sample_v_l = s_v_l;
    s_sample_v_r = s_v_r;
    s_sensor_ready = ready;
    s_sample_ms = sample_tick * (uint32)PIT_D;
    if (epoch == s_epoch) { g_sm.v_l = s_v_l; g_sm.v_r = s_v_r; }
    __set_PRIMASK(mask);
    if (!ready)
    {
        if (state != ST_ERROR && state != ST_INIT)
            state_machine_request_fault(is_calibrated ? SM_ERR_IMU_TIMEOUT : SM_ERR_NO_IMU, epoch);
        return;
    }
    if (state == ST_INIT) { post_event(EVENT_READY); return; }

    if (s_control_epoch != epoch)
    {
        /* Target search and trigonometry in Navigation_Reset are MAIN ONLY. */
        Navigation_Reset();
        component_reset();
        path_settle=path_wait=0;
        s_control_epoch = epoch;
        s_duty_l = s_duty_r = 0;
        robust_prev_l = robust_prev_r = robust_prev2_l = robust_prev2_r = 0;
        motor_launch_reset(&s_launch_l, DUTY_START_MIN);
        motor_launch_reset(&s_launch_r, DUTY_START_MIN);
        s_stall_l_ms = s_stall_r_ms = 0;
        s_output_applied = 0;
        s_debug_target_mps = 0.0f;
        if (state == ST_RUN && !debug_mode)
        {
            s_duty_l = DUTY_START_MIN;
            if (s_duty_l > DUTY_CMD_MAX) s_duty_l = DUTY_CMD_MAX;
            if (s_duty_l < 0) s_duty_l = 0;
            s_duty_r = s_duty_l;
        }
        /* Discard pre-launch wheel pulses in the new path. */
    }
    else if (state == ST_RUN && !debug_mode)
    {
        float ds = 0.5f * ((float)encoder_delta[0] + (float)encoder_delta[1]) * METERS_PER_COUNT;
        NavigationUpdate(ds, dt);
    }

    if (state == ST_ARMED) { imu_bias_track(); return; }
    if (state == ST_RUN && C_MODULE != 5)
    {
        ComponentOutput o;
        if(elapsed_ms != PIT_D || app_runtime_epoch_ticks()!=sample_tick)
        {state_machine_request_fault(SM_ERR_CONTROL_OVERRUN,epoch);return;}
        o=component_step(dt,fmaxf(fabsf(s_v_l),fabsf(s_v_r)),s_active_pwm,s_active_hold,s_active_turn,s_active_servo,yaw*RAD2DEG,s_active_v1,s_active_v2);
        if(C_MODULE==1)
            o.turn=component_straight_turn(yaw,yaw_rate,s_v_l,s_v_r);
        g_nav.path_index=C_MODULE*10+o.phase;
        if(o.done<0){state_machine_request_fault(SM_ERR_RUN_TIMEOUT,epoch);return;}
        if(o.done){request_done(epoch);return;}
        if(o.open_loop){s_duty_l=s_duty_r=o.pwm;}
        else {
            s_duty_l=robust_speed_pi(s_duty_l,o.speed,s_v_l,&robust_prev_l,&robust_prev2_l,dt,DUTY_CMD_MAX);
            s_duty_r=robust_speed_pi(s_duty_r,o.speed,s_v_r,&robust_prev_r,&robust_prev2_r,dt,DUTY_CMD_MAX);
            s_duty_l=motor_launch_step(&s_launch_l,s_duty_l,o.speed,s_v_l,dt,DUTY_CMD_MAX,LAUNCH_PWM_RAMP_PERCENT_S,V_DEADBAND_MPS);
            s_duty_r=motor_launch_step(&s_launch_r,s_duty_r,o.speed,s_v_r,dt,DUTY_CMD_MAX,LAUNCH_PWM_RAMP_PERCENT_S,V_DEADBAND_MPS);
        }
        if(C_MODULE!=3){
            s_stall_l_ms=(s_duty_l>=DUTY_START_MIN && s_v_l<V_DEADBAND_MPS)?s_stall_l_ms+elapsed_ms:0;
            s_stall_r_ms=(s_duty_r>=DUTY_START_MIN && s_v_r<V_DEADBAND_MPS)?s_stall_r_ms+elapsed_ms:0;
            if(s_stall_l_ms>=MOTOR_STALL_TIMEOUT_MS || s_stall_r_ms>=MOTOR_STALL_TIMEOUT_MS)
            {state_machine_request_fault(SM_ERR_MOTOR_STALL,epoch);return;}
        }
        if(!isfinite(o.turn)||!isfinite(s_duty_l)||!isfinite(s_duty_r))
        {state_machine_request_fault(SM_ERR_CONTROL_OVERRUN,epoch);return;}
        if(!o.open_loop){
            mean=.5f*(s_duty_l+s_duty_r);
            diff=fmaxf(-SPEED_DUTY_DIFF_MAX,fminf(SPEED_DUTY_DIFF_MAX,s_duty_l-s_duty_r));
            s_duty_l=mean+.5f*diff;s_duty_r=mean-.5f*diff;
        }
        s_duty_l=fmaxf(0,fminf(C_MODULE==1?40.0f:DUTY_CMD_MAX,s_duty_l));
        s_duty_r=fmaxf(0,fminf(C_MODULE==1?40.0f:DUTY_CMD_MAX,s_duty_r));
        s_output_applied=publish_output(epoch,sample_tick,o.turn,s_duty_l,s_duty_r,
                                        0,C_MODULE==1?-yaw*RAD2DEG:0,o.speed);
        return;
    }
    if (state != ST_RUN) return;
    if (debug_mode && now_ms - joy_last_ms > DEBUG_LINK_TIMEOUT_MS)
    { state_machine_request_fault(SM_ERR_REMOTE_TIMEOUT, epoch); return; }
    if (elapsed_ms != PIT_D || app_runtime_epoch_ticks() != sample_tick)
    { state_machine_request_fault(SM_ERR_CONTROL_OVERRUN, epoch); return; }

#if CHASSIS_BRINGUP_MODE
    /* One idle wheel is intentional during the motor mapping test. Keep the
     * normal STOP/deadline protections, but skip normal two-wheel stall/PI. */
    (void)publish_output(epoch, sample_tick, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return;
#endif

    /* Stall time starts AFTER a nonzero duty was actually applied. */
    if (s_output_applied)
    {
        uint8 expect_manual_motion = (uint8)(!debug_mode || s_debug_target_mps >= V_DEADBAND_MPS);
        s_stall_l_ms = (expect_manual_motion && s_duty_l >= DUTY_START_MIN && s_v_l < V_DEADBAND_MPS)
                      ? s_stall_l_ms + elapsed_ms : 0;
        s_stall_r_ms = (expect_manual_motion && s_duty_r >= DUTY_START_MIN && s_v_r < V_DEADBAND_MPS)
                      ? s_stall_r_ms + elapsed_ms : 0;
    }
    if (s_stall_l_ms >= MOTOR_STALL_TIMEOUT_MS || s_stall_r_ms >= MOTOR_STALL_TIMEOUT_MS)
    { state_machine_request_fault(SM_ERR_MOTOR_STALL, epoch); return; }
    if (!debug_mode && (Navigation_Arrived() || path_wait>0))
    {
        path_wait+=dt;
        path_settle=(fabsf(s_v_l)<.03f && fabsf(s_v_r)<.03f)?path_settle+dt:0;
        (void)publish_output(epoch,sample_tick,0,0,0,0,0,0);
        if(path_settle>=C_SETTLE_HOLD)request_done(epoch);
        else if(path_wait>C_SETTLE_TIMEOUT)state_machine_request_fault(SM_ERR_RUN_TIMEOUT,epoch);
        return;
    }

    if (debug_mode)
    {
        if (joy_decelerate)
        {
            s_debug_target_mps -= DEBUG_DECEL_MPS2 * dt;
            if (s_debug_target_mps < 0.0f) s_debug_target_mps = 0.0f;
        }
        else if (joy_accelerate)
        {
            s_debug_target_mps += DEBUG_ACCEL_MPS2 * dt;
            if (s_debug_target_mps > DEBUG_MAX_SPEED_MPS)
                s_debug_target_mps = DEBUG_MAX_SPEED_MPS;
        }
        target = 0.0f;
        speed_target = s_debug_target_mps;
        turn = (float)joy_steering;
        error = 0.0f;
    }
    else
    {
        target = g_nav.bearing * RAD2DEG;
        if (!Navigation_TrackingValid())
        { state_machine_request_fault(SM_ERR_CONTROL_OVERRUN, epoch); return; }
        speed_target = Navigation_SpeedTarget();
        error = wrap180(target - yaw * RAD2DEG);
        turn = Navigation_TurnCommand();
    }
    if (turn > 1) turn = 1;
    if (turn < -1) turn = -1;
    /* First output is exactly the launch duty; integrate from the next tick. */
    if (s_output_applied)
    {
        if (debug_mode) {
            s_duty_l += DEBUG_SPEED_KI * (speed_target - s_v_l) * dt;
            s_duty_r += DEBUG_SPEED_KI * (speed_target - s_v_r) * dt;
        } else {
            s_duty_l = robust_speed_pi(s_duty_l,speed_target,s_v_l,&robust_prev_l,&robust_prev2_l,dt,DUTY_CMD_MAX);
            s_duty_r = robust_speed_pi(s_duty_r,speed_target,s_v_r,&robust_prev_r,&robust_prev2_r,dt,DUTY_CMD_MAX);
            /* The first path sample has a low speed ceiling. With no wheel
             * motion its spatial progress cannot increase, so PI alone can
             * remain near 13% until the stall deadline. Ramp through static
             * friction once per launch, without relaxing that deadline. */
            s_duty_l = motor_launch_step(&s_launch_l, s_duty_l, speed_target,
                s_v_l, dt, DUTY_CMD_MAX, LAUNCH_PWM_RAMP_PERCENT_S, V_DEADBAND_MPS);
            s_duty_r = motor_launch_step(&s_launch_r, s_duty_r, speed_target,
                s_v_r, dt, DUTY_CMD_MAX, LAUNCH_PWM_RAMP_PERCENT_S, V_DEADBAND_MPS);
        }
    }
    if (debug_mode && speed_target > 0.0f &&
        s_v_l < V_DEADBAND_MPS && s_v_r < V_DEADBAND_MPS &&
        s_duty_l < 1.0f && s_duty_r < 1.0f)
        s_duty_l = s_duty_r = DUTY_START_MIN;
    mean = 0.5f * (s_duty_l + s_duty_r);
    diff = s_duty_l - s_duty_r;
    if (diff > SPEED_DUTY_DIFF_MAX) diff = SPEED_DUTY_DIFF_MAX;
    if (diff < -SPEED_DUTY_DIFF_MAX) diff = -SPEED_DUTY_DIFF_MAX;
    s_duty_l = mean + 0.5f * diff;
    s_duty_r = mean - 0.5f * diff;
    if (s_duty_l < 0) s_duty_l = 0;
    if (s_duty_r < 0) s_duty_r = 0;
    if (s_duty_l > DUTY_CMD_MAX) s_duty_l = DUTY_CMD_MAX;
    if (s_duty_r > DUTY_CMD_MAX) s_duty_r = DUTY_CMD_MAX;
    if (!isfinite(turn) || !isfinite(s_duty_l) || !isfinite(s_duty_r))
    { state_machine_request_fault(SM_ERR_CONTROL_OVERRUN, epoch); return; }

    /* Publish only a small mailbox while IRQs are masked. The highest-priority
     * handler checks epoch and tick again and commits the hardware output. */
    s_output_applied = publish_output(epoch, sample_tick, turn, s_duty_l, s_duty_r, target, error, speed_target);
}

