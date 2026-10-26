#include "app_runtime.h"
#include "state_machine.h"

static volatile uint32 s_ticks;
static uint32 s_consumed_ticks;
static volatile uint32 s_last_complete_tick;
/* Foreground owns report cadence; PIT must never publish a future sample. */
static uint8 s_telemetry_due;
static uint16 s_telemetry_phase;
volatile uint32 app_control_peak_cycles;
volatile uint32 app_control_missed_ticks;

void app_runtime_init(void)
{
    s_ticks = s_consumed_ticks = s_last_complete_tick = 0;
    s_telemetry_due = 0;
    s_telemetry_phase = 0;
    app_control_peak_cycles = app_control_missed_ticks = 0;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    NVIC_SetPriority(PIT_IRQn, IRQ_PRIORITY_TICK);
    pit_ms_init(PIT_CH, PIT_D);
}

uint32 app_runtime_epoch_ticks(void) { return s_ticks; }
uint32 app_runtime_now_ms(void) { return s_ticks * (uint32)PIT_D; }

void app_runtime_tick_irq(void)
{
    ++s_ticks;
    /* No SPI, trigonometry, path search, PI controller or serial formatting. */
    state_machine_watchdog_irq(app_runtime_now_ms(),
        (s_ticks - s_last_complete_tick) * (uint32)PIT_D);
}

uint8 app_runtime_take_tick(uint32 *tick, uint32 *elapsed_ms)
{
    uint32 delta, current;
    uint32 mask = __get_PRIMASK();
    __disable_irq();
    current = s_ticks;
    delta = current - s_consumed_ticks;
    s_consumed_ticks = current;
    __set_PRIMASK(mask);
    if (!delta) return 0;
    *tick = current;
    *elapsed_ms = delta * (uint32)PIT_D;
    /* Advance cadence only for the tick claimed above. An IRQ arriving after
     * that claim belongs to the NEXT sample, even if it interrupts this code.
     * Coalesce skipped idle ticks without generating fake catch-up reports.
     * Reduce delta first to keep multiplication bounded after long idle gaps. */
    {
        uint32 phase = s_telemetry_phase +
            (delta % 1000U) * (uint32)(TELEMETRY_HZ * PIT_D);
        if (delta >= 1000U || phase >= 1000U) s_telemetry_due = 1;
        s_telemetry_phase = (uint16)(phase % 1000U);
    }
    if (delta > 1)
    {
        app_control_missed_ticks += delta - 1;
        if (g_sm.state == ST_RUN)
            state_machine_request_fault(SM_ERR_CONTROL_OVERRUN, state_machine_epoch());
    }
    /* Read hardware once, with actual elapsed time; do not fake catch-up samples. */
    return 1;
}

void app_runtime_control_complete(uint32 tick, uint32 cycle_start)
{
    uint32 cycles = DWT->CYCCNT - cycle_start;
    if (cycles > app_control_peak_cycles) app_control_peak_cycles = cycles;
    s_last_complete_tick = tick;
    if (g_sm.state == ST_RUN && s_ticks != tick)
        state_machine_request_fault(SM_ERR_CONTROL_OVERRUN, state_machine_epoch());
}

uint8 app_runtime_take_telemetry(void)
{
    uint8 due = s_telemetry_due;
    s_telemetry_due = 0;
    return due;
}
