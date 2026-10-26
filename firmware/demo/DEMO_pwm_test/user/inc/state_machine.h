#ifndef _state_machine_h_
#define _state_machine_h_

#include "my_common.h"

typedef enum { ST_INIT = 0, ST_ARMED, ST_RUN, ST_DONE, ST_ERROR } sm_state_enum;
typedef enum
{
    SM_ERR_NONE = 0,
    SM_ERR_NO_IMU = 1,
    SM_ERR_MOVING_START = 2,
    SM_ERR_IMU_TIMEOUT = 3,
    SM_ERR_MOTOR_STALL = 4,
    SM_ERR_RUN_TIMEOUT = 5,
    SM_ERR_CONTROL_OVERRUN = 6,
    SM_ERR_UART_DMA = 7,
    SM_ERR_UART_RX = 8,
    SM_ERR_REMOTE_TIMEOUT = 9
} sm_error_enum;

typedef struct
{
    sm_state_enum state;
    float yaw_target, yaw_err;
    float v_l, v_r;
    float duty_l, duty_r;
    float turn_cmd;
    float speed_target;
    uint8 debug_mode;
    uint8 err_code;
} sm_telemetry_t;

typedef struct { uint8 command, state, error; } sm_reply_t;
extern volatile sm_telemetry_t g_sm;
extern volatile uint32 sm_reply_dropped;
extern volatile uint32 sm_transition_peak_cycles;

/* All state writes after initialization belong to priority-0 PendSV. This
 * application is bare metal; PendSV is reserved and cannot also run an RTOS. */
void state_machine_init(void);
void state_machine_transition_irq(void);
void state_machine_request_start(void);
void state_machine_request_debug(void);
void state_machine_request_joystick(uint8 accelerate, uint8 decelerate, int8 steering);
void state_machine_request_stop(void);
void state_machine_request_calibration(void);
uint8 state_machine_take_calibration_request(void);
void state_machine_set_calibration_active(uint8 active);
void state_machine_request_fault(uint8 error, uint32 epoch);
void state_machine_watchdog_irq(uint32 now_ms, uint32 control_age_ms);
uint32 state_machine_epoch(void);
uint8 state_machine_peek_reply(sm_reply_t *reply);
void state_machine_drop_reply(void);

/* FOREGROUND ONLY: sensor health, wheel-speed measurement and odometry.
 * Call after get_data() and imu_update_elapsed(). */
void state_machine_control_task(uint32 elapsed_ms, uint32 sample_tick);

#endif
