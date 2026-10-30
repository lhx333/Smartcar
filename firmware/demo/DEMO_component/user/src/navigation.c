/* Replacement for step31 navigation.c; trajectory coordinates remain start-local. */
#include "navigation.h"
#include "imu.h"
#include "test_path.h"
#include "component_config.h"
#if CHASSIS_MODE != CHASSIS_STEERING_SERVO
#error "Robust bicycle tracking requires steering-servo chassis; differential mixing needs a separate model."
#endif
#include <math.h>
Navigation_t g_nav;
static RobustControl controller;
static void update_target(float dt) {
    if(C_MODULE!=5)return;
    test_path_step(&controller,g_nav.x,g_nav.y,yaw,yaw_rate,g_nav.speed,dt);
    g_nav.path_index=controller.index;g_nav.progress_m=controller.progress;
    g_nav.remaining_m=controller.remaining;g_nav.look_x=controller.look_x;
    g_nav.look_y=controller.look_y;g_nav.bearing=controller.bearing;
    { float dx=g_nav.x-ROBUST_PATH[ROBUST_PATH_N-1].x; float dy=g_nav.y-ROBUST_PATH[ROBUST_PATH_N-1].y; g_nav.dist_to_goal=sqrtf(dx*dx+dy*dy); }
}
float Navigation_SpeedTarget(void) { return controller.speed_target; }
float Navigation_TurnCommand(void) { return controller.turn; }
uint8 Navigation_TrackingValid(void) { return (uint8)controller.valid; }
void Navigation_Init(void) {
    memset(&g_nav,0,sizeof(g_nav));robust_reset(&controller);
    g_nav.path_count=ROBUST_PATH_N;update_target(DT);
}
void Navigation_Reset(void) {
    memset(&g_nav,0,sizeof(g_nav));robust_reset(&controller);imu_yaw_reset();
    g_nav.path_count=ROBUST_PATH_N;update_target(DT);
}
void NavigationUpdate(float ds,float dt) {
    if(dt<=0)return;
    g_nav.x+=ds*cosf(yaw_mid);g_nav.y+=ds*sinf(yaw_mid);g_nav.distance+=fabsf(ds);
    g_nav.speed+=SPEED_LPF_A*(ds/dt-g_nav.speed);update_target(dt);
}
uint8 Navigation_Arrived(void) {
    /* Full car crossed x=0 before approaching the runout endpoint. */
    return g_nav.remaining_m<0.06f && g_nav.dist_to_goal<0.10f;
}
