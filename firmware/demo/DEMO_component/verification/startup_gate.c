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
static float turn_output;
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
    assert(in_irq && g_sm.state == ST_RUN);
    assert(left >= 0 && left <= 1 && right >= 0 && right <= 1);
    left_output = left * DUTY_CMD_MAX;
    right_output = right * DUTY_CMD_MAX;
    turn_output = turn;
}
void speed_apply_pwm_test(float turn, float left_percent, float right_percent)
{
    assert(in_irq && g_sm.state == ST_RUN);
    assert(left_percent >= 0 && left_percent <= 40);
    assert(right_percent >= 0 && right_percent <= 40);
    left_output = left_percent;
    right_output = right_percent;
    turn_output = turn;
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
int main(void)
{
    int i;
    assert(component_straight_turn(0,0,.6f,.8f)<0);
    assert(component_straight_turn(0,0,.8f,.6f)>0);
    assert(component_straight_turn(0,0,0,0)==0);
    setup();
    for(i=0;i<1000;i++)tick();
    assert(g_sm.state==ST_INIT && left_output==0 && right_output==0);
    if(C_MODULE==1||C_MODULE==4)assert(!state_machine_request_start());
    else {state_machine_request_start();expect_reply(6);}
    state_machine_request_debug();expect_reply(7);
    assert(g_sm.state==ST_INIT && left_output==0);
    state_machine_request_calibration();expect_reply(3);
    assert(state_machine_take_calibration_request());
    for(i=0;i<200;i++)tick();
    if(C_MODULE==1||C_MODULE==4)assert(!state_machine_request_start());
    else {state_machine_request_start();expect_reply(6);}
    assert(g_sm.state==ST_INIT && left_output==0 && right_output==0);
    /* IMU sampling/calculation is mocked; all start/output gates are real C. */
    is_calibrated=1;state_machine_set_calibration_active(0);tick();
    for(i=0;i<200;i++)tick();
    assert(g_sm.state==ST_ARMED && left_output==0 && right_output==0);
    if(C_MODULE==1){assert(state_machine_request_start_pwm(40,2));assert(g_sm.state==ST_ARMED && left_output==0);}
    if(C_MODULE==4){assert(state_machine_configure_circle(45,8,4,6));assert(g_sm.state==ST_ARMED && left_output==0);}
    assert(state_machine_request_start());
    expect_reply(1);tick();
    assert(g_sm.state==ST_RUN);
    if(C_MODULE==3)assert(left_output==0 && right_output==0);
    else assert(left_output>0 && right_output>0);
    if(C_MODULE==1){
        assert(left_output==40 && right_output==40);
        yaw=.05f;encoder_delta[0]=encoder_delta[1]=30;tick();
        assert(turn_output<0); /* positive yaw must steer back right */
        encoder_delta[0]=encoder_delta[1]=0;
    }
    if(C_MODULE==1||C_MODULE==4)assert(!state_machine_request_start());
    else {state_machine_request_start();expect_reply(6);}
    state_machine_request_stop();expect_reply(2);
    assert(left_output==0 && right_output==0 && g_sm.state==ST_ARMED);
    for(i=0;i<100;i++)tick();
    assert(left_output==0 && right_output==0 && g_sm.state==ST_ARMED);

    setup();is_calibrated=1;s_sensor_ready=1;
    __disable_irq();post_event(EVENT_READY);state_machine_request_start();__set_PRIMASK(0);
    expect_reply(6);
    for(i=0;i<100;i++)tick();
    assert(g_sm.state==ST_ARMED && left_output==0 && right_output==0);
    __disable_irq();post_event(EVENT_START);state_machine_request_stop();__set_PRIMASK(0);
    expect_reply(2);expect_reply(6);
    assert(g_sm.state==ST_ARMED && left_output==0 && right_output==0);
    printf("PASS module %d: boot/calibration/ARMED stay stopped; premature START rejected; fresh START required; STOP wins\n",C_MODULE);
    return 0;
}
