/* Module 1: constant open-loop PWM, then an immediate zero-PWM coast. */
static void test_pwm(unsigned int tick, float dt, float pwm_percent, float hold_s, ComponentOutput *o) {
    unsigned int hold_ticks=(unsigned int)(hold_s/dt+0.5f);
    o->open_loop=1;
    if(tick<hold_ticks){o->pwm=pwm_percent;o->phase=1;}
    else{o->pwm=0;o->done=1;}
}
