/* Portable tracking controller. Units: m, s, rad; +yaw/+turn left.
 * ROAD_ANGLE_PER_TURN MUST be measured at the wheels, not the servo horn.
 * Defaults are commissioning assumptions, not vehicle identification results. */
#ifndef ROBUST_CONTROL_H
#define ROBUST_CONTROL_H
#include <math.h>
#include <string.h>
#include "robust_trajectory.h"
#define ROAD_ANGLE_PER_TURN ROBUST_ROAD_ANGLE_RAD
#define ROBUST_RATE_KD 0.004f /* normalized turn per degree/s */
#define ROBUST_ERROR_LIMIT 0.35f
typedef struct {
    int index,valid;
    float progress,remaining,cross,look,look_x,look_y,bearing;
    float speed_target,turn,rate_filtered;
} RobustControl;
static float robust_clamp(float x,float a,float b) { return x<a?a:(x>b?b:x); }
static float robust_wrap(float x) {
    while(x>3.141592654f) x-=6.283185307f;
    while(x< -3.141592654f) x+=6.283185307f;
    return x;
}
static void robust_reset(RobustControl *c) {
    memset(c,0,sizeof(*c)); c->valid=1; c->look=ROBUST_LOOK_MIN;
}
static void robust_step(RobustControl *c,float x,float y,float yaw,float rate,float speed,float dt) {
    int i,best=c->index,j;
    float bestd=1e30f,progress=c->progress,dx,dy,f,ds,goal,kpp,ld,planned,ratio;
    float search_end=c->progress+0.30f+fabsf(speed)*0.50f;
    if(!isfinite(x)||!isfinite(y)||!isfinite(yaw)||!isfinite(rate)||!isfinite(speed)||!isfinite(dt)||dt<=0||dt>0.05f) {
        c->valid=0; c->speed_target=0; c->turn=0; return;
    }
    for(i=c->index;i<ROBUST_PATH_N-1 && ROBUST_PATH[i].s<=search_end;i++) {
        float vx=ROBUST_PATH[i+1].x-ROBUST_PATH[i].x;
        float vy=ROBUST_PATH[i+1].y-ROBUST_PATH[i].y;
        float l2=vx*vx+vy*vy,t,p,px,py,d2;
        if(l2<1e-12f) continue;
        t=robust_clamp(((x-ROBUST_PATH[i].x)*vx+(y-ROBUST_PATH[i].y)*vy)/l2,0,1);
        p=ROBUST_PATH[i].s+t*(ROBUST_PATH[i+1].s-ROBUST_PATH[i].s);
        if(p<c->progress) continue;
        /* Disallow branch jumps: bounded forward progress per measured control tick. */
        if(p>c->progress+fmaxf(0.04f,fabsf(speed)*dt*3.0f)) continue;
        px=ROBUST_PATH[i].x+t*vx; py=ROBUST_PATH[i].y+t*vy;
        d2=(x-px)*(x-px)+(y-py)*(y-py);
        if(d2<bestd){bestd=d2;best=i;progress=p;}
    }
    if(bestd==1e30f){c->valid=0;c->speed_target=0;c->turn=0;return;}
    c->index=best;c->progress=progress;c->cross=sqrtf(bestd);
    c->remaining=ROBUST_PATH[ROBUST_PATH_N-1].s-progress;
    ds=ROBUST_PATH[best+1].s-ROBUST_PATH[best].s;
    f=robust_clamp((progress-ROBUST_PATH[best].s)/fmaxf(ds,1e-6f),0,1);
    planned=ROBUST_PATH[best].v+f*(ROBUST_PATH[best+1].v-ROBUST_PATH[best].v);
    ld=ROBUST_PATH[best].look+f*(ROBUST_PATH[best+1].look-ROBUST_PATH[best].look);
    ld=fminf(ld,robust_clamp(ROBUST_LOOK_MIN+ROBUST_LOOK_TIME*fabsf(speed),ROBUST_LOOK_MIN,ROBUST_LOOK_MAX));
    c->look+=robust_clamp(ld-c->look,-0.30f*dt,0.30f*dt);
    goal=fminf(progress+c->look,ROBUST_PATH[ROBUST_PATH_N-1].s);
    j=best;while(j<ROBUST_PATH_N-2 && ROBUST_PATH[j+1].s<goal)j++;
    f=(goal-ROBUST_PATH[j].s)/fmaxf(ROBUST_PATH[j+1].s-ROBUST_PATH[j].s,1e-6f);
    c->look_x=ROBUST_PATH[j].x+f*(ROBUST_PATH[j+1].x-ROBUST_PATH[j].x);
    c->look_y=ROBUST_PATH[j].y+f*(ROBUST_PATH[j+1].y-ROBUST_PATH[j].y);
    dx=c->look_x-x;dy=c->look_y-y;
    c->bearing=(dx*dx+dy*dy<1e-8f)?yaw:atan2f(dy,dx);
    kpp=2.0f*(-sinf(yaw)*dx+cosf(yaw)*dy)/fmaxf(dx*dx+dy*dy,0.04f);
    c->rate_filtered+=dt/(0.030f+dt)*(rate-c->rate_filtered);
    c->turn=atanf(ROBUST_WHEELBASE*kpp)/ROAD_ANGLE_PER_TURN+
        ROBUST_RATE_KD*(speed*kpp-c->rate_filtered)*57.2957795f;
    /* Error/saturation reduction acts in addition to the offline speed ceiling. */
    ratio=fminf(1.0f,0.15f/fmaxf(c->cross,0.15f));
    ratio=fminf(ratio,0.65f/fmaxf(fabsf(c->turn),0.65f));
    planned*=ratio;
    c->turn=robust_clamp(c->turn,-1,1);
    c->speed_target+=robust_clamp(planned-c->speed_target,-ROBUST_BRAKE*dt,ROBUST_ACCEL*dt);
    c->valid=c->cross<=ROBUST_ERROR_LIMIT && fabsf(robust_wrap(c->bearing-yaw))<1.22f;
    if(!c->valid){c->speed_target=0;c->turn=0;}
}
#endif
