#ifndef ROBUST_SPEED_PI_H
#define ROBUST_SPEED_PI_H
/* Units: duty percentage points, m/s, s. Commissioning gains: P=6, I=3, D=0. */
static float robust_speed_pi(float applied,float target,float measured,float *prev,float dt,float maximum) {
    float e=target-measured;
    float u=applied+6.0f*(e-*prev)+3.0f*e*dt;
    *prev=e;
    return u<0?0:(u>maximum?maximum:u);
}
#endif
