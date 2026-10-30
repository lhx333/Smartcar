static void test_speed(float t, ComponentOutput *o) {
    float up=C_SPEED_TARGET/C_SPEED_ACCEL;
    if(t<up){o->speed=t*C_SPEED_ACCEL;o->phase=1;}
    else if(t<up+C_SPEED_HOLD){o->speed=C_SPEED_TARGET;o->phase=2;}
    else if(t<up+C_SPEED_HOLD+C_SPEED_TARGET/C_SPEED_DECEL){o->speed=C_SPEED_TARGET-(t-up-C_SPEED_HOLD)*C_SPEED_DECEL;o->phase=3;}
    else o->done=1;
}
