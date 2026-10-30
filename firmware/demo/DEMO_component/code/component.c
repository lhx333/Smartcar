#include <math.h>
#include <string.h>
#include "vehicle_params.h"
#include "component.h"
static float elapsed,steer_offset,steer_hold,circle_turn,settle,settle_elapsed;
static float circle_reached_at,circle_centered_at;
static unsigned int elapsed_ticks;
static int steer_index;
#include "test_pwm.h"
#include "test_speed.h"
#include "test_static_steering.h"
#include "test_circle.h"
void component_reset(void){elapsed=steer_offset=steer_hold=circle_turn=settle=settle_elapsed=0;circle_reached_at=circle_centered_at=-1;elapsed_ticks=0;steer_index=0;}
ComponentOutput component_step(float dt,float measured_speed,float pwm_percent,float hold_s,float target_yaw_deg,float servo_deg,float yaw_deg,float v1,float v2){
    ComponentOutput o;memset(&o,0,sizeof(o));
    switch(C_MODULE){
    case 1:test_pwm(elapsed_ticks,dt,pwm_percent,hold_s,&o);break;
    case 2:test_speed(elapsed,&o);break;
    case 3:test_static_steering(dt,&o);break;
    case 4:test_circle(elapsed,dt,target_yaw_deg,servo_deg,yaw_deg,v1,v2,&o);break;
    default:o.done=1;break;
    }
    elapsed+=dt;
    ++elapsed_ticks;
    if(o.done<0)return o;
    if(o.done && C_MODULE!=3){
        o.done=0;o.open_loop=1;o.pwm=0;o.speed=0;o.phase=9;
        settle_elapsed+=dt;
        settle=fabsf(measured_speed)<0.03f?settle+dt:0;
        if(settle>=C_SETTLE_HOLD)o.done=1;
        else if(settle_elapsed>C_SETTLE_TIMEOUT)o.done=-1;
    }
    return o;
}
