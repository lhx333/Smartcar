#ifndef ROBUST_TRAJECTORY_H
#define ROBUST_TRAJECTORY_H
typedef struct { float x,y,s,k,v,look; } RobustPoint;
static const RobustPoint ROBUST_PATH[]={{0,0,0,0,0,.35f},{1,0,1,0,0,.35f}};
#define ROBUST_PATH_N ((int)(sizeof(ROBUST_PATH)/sizeof(ROBUST_PATH[0])))
#define ROBUST_WHEELBASE (0.2f)
#define ROBUST_ROAD_ANGLE_RAD (0.2617993877991494f)
#define ROBUST_ACCEL (0.35f)
#define ROBUST_BRAKE (0.45f)
#define ROBUST_LOOK_MIN (0.35f)
#define ROBUST_LOOK_MAX (0.6f)
#define ROBUST_LOOK_TIME (0.5f)
#endif
