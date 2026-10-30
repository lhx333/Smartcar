static void test_static_steering(float dt, ComponentOutput *o) {
    float target=COMP_STEER_OFFSETS[steer_index];
    float d=target-steer_offset, step=C_STEER_RATE*dt;
    o->open_loop=1; o->phase=steer_index+1;
    if(fabsf(d)>step)steer_offset+=(d>0?step:-step);
    else {steer_offset=target; steer_hold+=dt;
        if(steer_hold>=C_STEER_HOLD){steer_hold=0;if(++steer_index>=C_STEER_COUNT){steer_index=C_STEER_COUNT-1;o->done=1;}}
    }
    o->turn=steer_offset/VEHICLE_SERVO_MAX_DEG;
}
