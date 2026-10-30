#ifndef MOTOR_LAUNCH_H
#define MOTOR_LAUNCH_H

/* Launch-only static-friction assistance. Duty units are percentage points.
 * The caller retains the normal stall deadline and final output limits. */
typedef struct
{
    float floor;
    unsigned char active;
} motor_launch_t;

static void motor_launch_reset(motor_launch_t *launch, float start)
{
    launch->floor = start;
    launch->active = 1;
}

static float motor_launch_step(motor_launch_t *launch, float duty, float target,
                               float measured, float dt, float maximum,
                               float ramp, float moving_threshold)
{
    if (measured >= moving_threshold) launch->active = 0;
    if (launch->active && target > 0.0f)
    {
        launch->floor += ramp * dt;
        if (launch->floor > maximum) launch->floor = maximum;
        if (duty < launch->floor) duty = launch->floor;
    }
    return duty;
}

#endif
