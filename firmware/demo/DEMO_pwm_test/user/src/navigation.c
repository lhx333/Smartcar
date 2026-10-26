/* Measurement-only odometry. Steering is held at centre by the state machine. */
#include "navigation.h"
#include "imu.h"
#include <math.h>
#include <string.h>
Navigation_t g_nav;
void Navigation_Init(void) { memset(&g_nav, 0, sizeof(g_nav)); }
void Navigation_Reset(void) { Navigation_Init(); imu_yaw_reset(); }
void NavigationUpdate(float ds, float dt)
{
    if (dt <= 0.0f) return;
    g_nav.x += ds * cosf(yaw_mid);
    g_nav.y += ds * sinf(yaw_mid);
    g_nav.distance += fabsf(ds);
    g_nav.speed += SPEED_LPF_A * (ds / dt - g_nav.speed);
    g_nav.progress_m = g_nav.distance;
}
uint8 Navigation_Arrived(void) { return 0; }
float Navigation_SpeedTarget(void) { return 0.0f; }
float Navigation_TurnCommand(void) { return 0.0f; }
uint8 Navigation_TrackingValid(void) { return 1; }
