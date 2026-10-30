/* Module 1 holds launch heading with an IMU yaw PD and an encoder wheel-difference
 * damping term. The rear motors remain open-loop at the requested equal PWM. */
#ifndef COMPONENT_STRAIGHT_CONTROL_H
#define COMPONENT_STRAIGHT_CONTROL_H
#include <math.h>
#include "my_common.h"
#include "component_config.h"
#include "vehicle_params.h"

static float component_straight_turn(float heading_rad, float yaw_rate_rad_s,
                                     float speed_l_mps, float speed_r_mps)
{
    float mean = 0.5f * (speed_l_mps + speed_r_mps);
    float cmd, limit;
    if (mean < 0.10f) return 0.0f;
    /* Positive physical yaw and faster right rear wheel imply leftward turn;
     * both therefore require a negative steering correction. */
    cmd = -C_STRAIGHT_KP * heading_rad * RAD2DEG
          -C_STRAIGHT_KD * yaw_rate_rad_s * RAD2DEG
          -C_STRAIGHT_WHEEL_K * (speed_r_mps - speed_l_mps);
    limit = C_STRAIGHT_MAX_DEG / VEHICLE_SERVO_MAX_DEG;
    if (cmd > limit) cmd = limit;
    if (cmd < -limit) cmd = -limit;
    return cmd;
}
#endif
