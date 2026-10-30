#ifndef COMPONENT_TEST_PATH_H
#define COMPONENT_TEST_PATH_H
/* Mode 5: actual shared Pure Pursuit tracker, with offline validated forward path. */
#include "robust_control.h"
static void test_path_step(RobustControl *control,float x,float y,float heading,float rate,float speed,float dt)
{ robust_step(control,x,y,heading,rate,speed,dt); }
#endif
