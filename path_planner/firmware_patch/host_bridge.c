#include "robust_control.h"
static RobustControl c;
__declspec(dllexport) void reset(void){robust_reset(&c);}
__declspec(dllexport) void tick(float x,float y,float yaw,float rate,float v,float dt,float *out){
 robust_step(&c,x,y,yaw,rate,v,dt);
 out[0]=c.speed_target;out[1]=c.turn;out[2]=c.cross;out[3]=c.progress;out[4]=(float)c.valid;
}
