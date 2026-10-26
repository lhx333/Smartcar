/* RT1064: 5ms foreground acquisition/control/planning; priority-0 PendSV
 * state transitions; UART1 TX eDMA; compatible fixed-point binary telemetry at 200Hz. */
#include "zf_common_headfile.h"
#include "my_common.h"
#include "imu.h"
#include "speed_control.h"
#include "servo_control.h"
#include "navigation.h"
#include "state_machine.h"
#include "app_runtime.h"
#include "serial_link.h"
#include <stdio.h>
#include <math.h>

extern void get_data(void);
extern int16 encoder_delta[ENCODER_COUNT];
void Init_All(void);
static uint32 telemetry_encoder_l, telemetry_encoder_r;
static uint8 imu_init_attempted;
static uint8 first_telemetry_sent;
static uint8 calibration_report_state = 0xFF;
static int calibration_report_progress = -1;
static uint32 calibration_report_ms;

static const char *control_state_name(uint8 state)
{
    switch (state)
    {
        case ST_INIT: return "INIT";
        case ST_ARMED: return "ARMED";
        case ST_RUN: return "RUN";
        case ST_DONE: return "DONE";
        case ST_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

/* Fast frame: 27-byte payload + version/type + CRC + COBS = 32 wire bytes.
 * Capture only copies under the IRQ mask; all quantization/CRC runs foreground. */
static struct {
    sm_telemetry_t sm;
    float heading, rate, servo, x, y, progress;
    uint16 tick;
    uint8 valid;
    int path;
} sample;
static uint8 sample_ready;
static uint32 diagnostic_ms;
static void put16(uint8 *p, uint16 v) { p[0] = (uint8)v; p[1] = (uint8)(v >> 8); }
static void put32(uint8 *p, uint32 v) {
    put16(p, (uint16)v); put16(p + 2, (uint16)(v >> 16));
}
static int32 quantize(float v, float scale, int32 lo, int32 hi, uint8 *flags)
{
    float q = v * scale;
    if (!isfinite(q)) { *flags |= 128; return 0; }
    if (q < (float)lo) { *flags |= 128; return lo; }
    if (q > (float)hi) { *flags |= 128; return hi; }
    return (int32)(q >= 0 ? q + 0.5f : q - 0.5f);
}
static void telemetry_capture(uint32 tick)
{
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    sample.sm = g_sm;
    sample.servo = speed_applied_servo_offset();
    __set_PRIMASK(mask);
    sample.heading = yaw * RAD2DEG;
    sample.rate = yaw_rate * RAD2DEG;
    sample.x = g_nav.x; sample.y = g_nav.y;
    sample.progress = g_nav.progress_m;
    sample.path = g_nav.path_index;
    sample.tick = (uint16)tick;
    sample.valid = (uint8)((is_calibrated ? 1 : 0) | (imu_data_valid() ? 2 : 0));
    sample_ready = 1;
}
static uint8 telemetry_send(void)
{
    uint8 p[27], flags;
    if (!sample_ready) return 0;
    flags = sample.valid | (sample.sm.debug_mode ? 4 : 0);
    if (fabsf(sample.sm.turn_cmd) >= 0.999f) flags |= 8;
    if (fabsf(sample.servo - sample.sm.turn_cmd * CHASSIS_TURN_SIGN *
        (sample.sm.debug_mode ? DEBUG_TURN_MAX_DEG : CHASSIS_TURN_MAX_DEG)) > 0.01f) flags |= 16;
    if (sample.sm.duty_l >= DUTY_CMD_MAX || sample.sm.duty_r >= DUTY_CMD_MAX) flags |= 32;
    put16(p, sample.tick);
    p[2] = (uint8)(((uint8)sample.sm.state << 4) | (sample.sm.err_code & 15));
    put16(p+4, (uint16)quantize(sample.heading,100,-18000,18000,&flags));
    put16(p+6, (uint16)quantize(sample.sm.yaw_err,100,-18000,18000,&flags));
    put16(p+8, (uint16)quantize(sample.rate,10,-32768,32767,&flags));
    put16(p+10,(uint16)quantize(sample.sm.v_l,1000,-32768,32767,&flags));
    put16(p+12,(uint16)quantize(sample.sm.v_r,1000,-32768,32767,&flags));
    p[14]=(uint8)quantize(sample.sm.turn_cmd,127,-127,127,&flags);
    p[15]=(uint8)quantize(sample.servo,5,-127,127,&flags);
    p[16]=(uint8)quantize(sample.sm.duty_l,5,0,255,&flags);
    p[17]=(uint8)quantize(sample.sm.duty_r,5,0,255,&flags);
    put16(p+18,(uint16)quantize(sample.x,1000,-32768,32767,&flags));
    put16(p+20,(uint16)quantize(sample.y,1000,-32768,32767,&flags));
    put16(p+22,(uint16)quantize(sample.progress,1000,0,65535,&flags));
    p[24]=(uint8)quantize((float)sample.path,1,0,255,&flags);
    put16(p+25,(uint16)quantize(sample.sm.speed_target,1000,0,65535,&flags));
    p[3]=flags;
    if (!serial_link_frame(1,p,sizeof(p))) return 0;
    sample_ready = 0; /* Successful DMA enqueue consumes this snapshot once. */
    return 1;
}
static void diagnostic_report(void)
{
    uint8 p[32];
    uint32 now = app_runtime_now_ms();
    if (now - diagnostic_ms < 1000 || !serial_link_can_report()) return;
    put32(p, app_runtime_epoch_ticks());
    put32(p+4,telemetry_encoder_l); put32(p+8,telemetry_encoder_r);
    put32(p+12,app_control_peak_cycles); put32(p+16,app_control_missed_ticks);
    put32(p+20,serial_tx_dropped); put32(p+24,serial_tx_errors); put32(p+28,serial_rx_errors);
    if (serial_link_frame(3,p,sizeof(p))) diagnostic_ms=now;
}

/* Boot messages also use DMA; servicing them never waits for a byte. */
static void boot_message(const char *message)
{
    (void)serial_link_send_text(message);
    serial_link_service();
}

static void calibration_report(void)
{
    imu_calib_state_t state = imu_calib_state();
    int progress = imu_calib_progress();
    uint32 now_ms = app_runtime_now_ms();
    char line[64];
    int length;
    if (state == IMU_CALIB_RUNNING)
    {
        if (calibration_report_state == (uint8)state &&
            progress < calibration_report_progress + 5 && progress < 100 &&
            now_ms - calibration_report_ms < 1000U) return;
        length = snprintf(line, sizeof(line), "CALIBRATION,%s,%d\r\n",
                          calibration_report_state == (uint8)state ? "PROGRESS" : "START",
                          progress);
    }
    else if (state != (imu_calib_state_t)calibration_report_state)
    {
        if (state == IMU_CALIB_OK)
            length = snprintf(line, sizeof(line), "CALIBRATION,OK,100\r\n");
        else if (state == IMU_CALIB_FAILED)
            length = snprintf(line, sizeof(line), "CALIBRATION,FAILED,%s\r\n",
                              imu_calib_failure_reason());
        else
            length = snprintf(line, sizeof(line), "CALIBRATION,IDLE,0\r\n");
    }
    else if (now_ms - calibration_report_ms >= 1000U)
    {
        if (state == IMU_CALIB_OK)
            length = snprintf(line, sizeof(line), "CALIBRATION,OK,100\r\n");
        else if (state == IMU_CALIB_FAILED)
            length = snprintf(line, sizeof(line), "CALIBRATION,FAILED,%s\r\n",
                              imu_calib_failure_reason());
        else
            length = snprintf(line, sizeof(line), "CALIBRATION,IDLE,0\r\n");
    }
    else return;
    if (length > 0 && (uint32)length < sizeof(line) && serial_link_send_text(line))
    {
        calibration_report_state = (uint8)state;
        calibration_report_progress = progress;
        calibration_report_ms = now_ms;
    }
}

int main(void)
{
    uint32 tick, elapsed_ms, cycle_start, observed_epoch, epoch;
    int irq, length;
    uint8 report_pending = 0;
    sm_reply_t reply;
    char reply_line[48];

    clock_init(SYSTEM_CLOCK_600M);
    /* Make priority zero exclusive to the state exception. */
    for (irq = 0; irq < NUMBER_OF_INT_VECTORS - 16; ++irq)
        NVIC_SetPriority((IRQn_Type)irq, IRQ_PRIORITY_BACKGROUND);
    NVIC_SetPriority(SysTick_IRQn, IRQ_PRIORITY_BACKGROUND);
    NVIC_SetPriority(SVCall_IRQn, IRQ_PRIORITY_BACKGROUND);
    Init_All();
    speed_init();
    Navigation_Init();
    state_machine_init();
    serial_link_init();
    /* Start the scheduler/telemetry before touching the sensor. Startup telemetry is
     * useful while IMU is still absent or being initialized. */
    app_runtime_init();
    boot_message("BOOT,PROTO1,5MS,200HZ\r\n");
#if (IMU_MODEL == IMU_MODEL_RC)
    boot_message("IMU_MODE,POLLING,5MS\r\n");
#endif
    boot_message("SYS_MODE,PWM_TEST,OPEN_LOOP,5MS,200HZ\r\n");
    boot_message("CALIBRATION,IDLE,0\r\n");
    observed_epoch = state_machine_epoch();
    imu_init_attempted = 0;
    first_telemetry_sent = 0;

    while (1)
    {
        /* RX acknowledgements are queued by the top-priority UART IRQ. Start
         * pending DMA output before checking the next control tick; this is
         * nonblocking and keeps command confirmation out of the planner. */
        serial_link_service();
        /* Always give a due control sample first access to the CPU. Nothing
         * below waits for UART bandwidth or a DMA slot. */
        if (app_runtime_take_tick(&tick, &elapsed_ms))
        {
            cycle_start = DWT->CYCCNT;
            get_data();
            telemetry_encoder_l += (uint32)(int32)encoder_delta[0];
            telemetry_encoder_r += (uint32)(int32)encoder_delta[1];
            imu_update_elapsed(elapsed_ms);
            imu_calib_task();
            if (imu_calib_state() == IMU_CALIB_OK || imu_calib_state() == IMU_CALIB_FAILED)
                state_machine_set_calibration_active(0);
            state_machine_control_task(elapsed_ms, tick);
            telemetry_capture(tick);
            /* Bind report eligibility to this completed snapshot. A new PIT
             * tick cannot consume its report flag in the idle send branch. */
            if (app_runtime_take_telemetry()) report_pending = 1;
            app_runtime_control_complete(tick, cycle_start);
            continue;
        }
        if (!imu_init_attempted && first_telemetry_sent)
        {
            /* At least one startup frame has already been queued. Driver init
             * has a finite WHO_AM_I timeout; it never retries forever. */
            imu_init_attempted = 1;
            imu_init();
            calibration_report();
            continue;
        }
        if (state_machine_take_calibration_request())
        {
            /* A host retry can also reinitialize an IMU attached after boot. */
            if (!imu_data_valid())
            {
                imu_init();
                imu_init_attempted = 1;
            }
            if (imu_calib_start())
                state_machine_set_calibration_active(1);
            else
                state_machine_set_calibration_active(0);
            calibration_report();
            continue;
        }
        /* Retain an ACK until a DMA queue slot accepts it. Replies are ahead
         * of periodic reports and no ISR formats or transmits text. */
        if (state_machine_peek_reply(&reply))
        {
            length = snprintf(reply_line, sizeof(reply_line), "CMD,%s,%s,%u\r\n",
                              reply.command == 1 ? "START" :
                              reply.command == 2 ? "STOP" :
                              reply.command == 3 ? "CALIBRATE" :
                              reply.command == 4 ? "REJECTED_CALIBRATE" :
                              reply.command == 5 ? "DEBUG" :
                              reply.command == 6 ? "REJECTED_START" : "REJECTED_DEBUG",
                              control_state_name(reply.state), (unsigned int)reply.error);
            if (length > 0 && (uint32)length < sizeof(reply_line) &&
                serial_link_send_text(reply_line))
                state_machine_drop_reply();
            serial_link_service();
            continue;
        }
        epoch = state_machine_epoch();
        if (epoch != observed_epoch)
        {
            observed_epoch = epoch;
            report_pending = 1;
        }
        if (report_pending && serial_link_can_report() && telemetry_send())
        {
            report_pending = 0;
            first_telemetry_sent = 1;
        }
        calibration_report();
        diagnostic_report();
    }
}

void Init_All(void)
{
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, 17000, 0);
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, 17000, 0);
    encoder_quad_init(ENCODER_L, ENCODER_L_A, ENCODER_L_B);
    encoder_quad_init(ENCODER_R, ENCODER_R_A, ENCODER_R_B);
#if USE_ADC
    adc_init(ADC_CHANNEL1, ADC_12BIT);
    adc_init(ADC_CHANNEL2, ADC_12BIT);
    adc_init(ADC_CHANNEL3, ADC_12BIT);
    adc_init(ADC_CHANNEL4, ADC_12BIT);
#endif
    pwm_init(SERVO_MOTOR1_PWM, SERVO_MOTOR_FREQ, 0);
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
    /* serial_link_init enables RX after installing the SDK DMA TX handle. */
}

/* Compatibility entry used by the project's PIT ISR. */
void pit_handler(void)
{
    app_runtime_tick_irq();
}
