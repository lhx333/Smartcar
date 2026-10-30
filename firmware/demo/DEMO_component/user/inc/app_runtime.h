#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include "my_common.h"

/* Bare-metal scheduler: PIT publishes time; foreground owns all control work. */
void app_runtime_init(void);
void app_runtime_tick_irq(void);
uint32 app_runtime_now_ms(void);
uint32 app_runtime_epoch_ticks(void);
uint8 app_runtime_take_tick(uint32 *tick, uint32 *elapsed_ms);
void app_runtime_control_complete(uint32 tick, uint32 cycle_start);
/* Foreground only: consume after capturing the claimed control tick. */
uint8 app_runtime_take_telemetry(void);
extern volatile uint32 app_control_peak_cycles;
extern volatile uint32 app_control_missed_ticks;

#endif
