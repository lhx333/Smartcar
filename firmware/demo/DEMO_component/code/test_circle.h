/* Turn at v1 until IMU yaw reaches target, center steering, then drive straight
 * at v2 and stop. C_CIRCLE_TURN is the bounded servo command magnitude. */
static void test_circle(float t,float dt,float target_yaw_deg,float servo_deg,float yaw_deg,float v1,float v2,ComponentOutput *o) {
    float up=v1/C_SPEED_ACCEL, begin=up+C_CIRCLE_STRAIGHT;
    float step=C_STEER_RATE/VEHICLE_SERVO_MAX_DEG*dt, desired=0;
    if(t<up){o->speed=t*C_SPEED_ACCEL;o->phase=1;}
    else if(t<begin){o->speed=v1;o->phase=2;}
    else if(circle_reached_at<0){
        if((target_yaw_deg>0&&yaw_deg>=target_yaw_deg)||(target_yaw_deg<0&&yaw_deg<=target_yaw_deg))circle_reached_at=t;
        else if(t-begin>C_CIRCLE_HOLD1){o->done=-1;return;}
        if(circle_reached_at<0){o->speed=v1;o->phase=3;desired=(target_yaw_deg>0?1.0f:-1.0f)*servo_deg/VEHICLE_SERVO_MAX_DEG;}
    }
    if(circle_reached_at>=0&&circle_centered_at<0){
        float delta=v2-v1, a=delta>=0?C_SPEED_ACCEL:C_SPEED_DECEL, dv=(t-circle_reached_at)*a;
        o->speed=v1+(delta>=0?fminf(delta,dv):fmaxf(delta,-dv));o->phase=4;
    }
    circle_turn+=fmaxf(-step,fminf(step,desired-circle_turn));
    if(circle_reached_at>=0&&circle_centered_at<0&&fabsf(circle_turn)<.001f)circle_centered_at=t;
    if(circle_centered_at>=0){
        float ramp=fabsf(v2-v1)/(v2>=v1?C_SPEED_ACCEL:C_SPEED_DECEL);
        float straight_begin=fmaxf(circle_centered_at,circle_reached_at+ramp);
        if(t<straight_begin){float delta=v2-v1,a=delta>=0?C_SPEED_ACCEL:C_SPEED_DECEL,dv=(t-circle_reached_at)*a;o->speed=v1+(delta>=0?fminf(delta,dv):fmaxf(delta,-dv));o->phase=4;}
        else if(t<straight_begin+C_CIRCLE_HOLD2){o->speed=v2;o->phase=5;}
        else if(t<straight_begin+C_CIRCLE_HOLD2+v2/C_SPEED_DECEL){o->speed=fmaxf(0,v2-(t-straight_begin-C_CIRCLE_HOLD2)*C_SPEED_DECEL);o->phase=6;}
        else o->done=1;
    }
    o->turn=circle_turn;
}
