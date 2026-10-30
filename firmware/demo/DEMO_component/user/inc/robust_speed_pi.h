#ifndef ROBUST_SPEED_PI_H
#define ROBUST_SPEED_PI_H
#include "vehicle_params.h"
/* Incremental PID. P: duty %/(m/s), I: duty %/(m/s*s), D: duty %*s/(m/s). */
static float robust_speed_pi(float applied,float target,float measured,float *prev,float *prev2,float dt,float maximum) {
    float e=target-measured;
    float u=applied+VEHICLE_SPEED_KP*(e-*prev)+VEHICLE_SPEED_KI*e*dt
        +VEHICLE_SPEED_KD*(e-2.0f*(*prev)+*prev2)/dt;
    *prev2=*prev;
    *prev=e;
    return u<0?0:(u>maximum?maximum:u);
}
#endif
