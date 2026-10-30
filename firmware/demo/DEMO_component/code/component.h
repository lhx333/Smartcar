#ifndef COMPONENT_H
#define COMPONENT_H
#include "component_config.h"
typedef struct { float speed,turn,pwm; int open_loop,phase,done; } ComponentOutput;
void component_reset(void);
ComponentOutput component_step(float dt,float measured_speed,float pwm_percent,float hold_s,float target_yaw_deg,float servo_deg,float yaw_deg,float v1,float v2);
#endif
