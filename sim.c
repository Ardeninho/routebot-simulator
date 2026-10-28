#include "routebot.h"
#include <math.h>
static bool read_sensor(void *ctx,Sensors *out) {
    Sim *s=ctx;*out=s->sensors;return !s->dropout;
}
static void write_motor(void *ctx,Motors command) {
    Sim *s=ctx;s->motors=command;s->last_write_ms=s->sensors.sample_ms;
}
Hal sim_hal(Sim *s) { return (Hal){s,read_sensor,write_motor}; }
void sim_step(Sim *s,uint32_t now) {
    /* Independent simulated motor watchdog: stale commands stop the plant. */
    if((uint32_t)(now-s->last_write_ms)>100u) s->motors=(Motors){0,0};
    double dt=RB_STEP_MS/1000.0;
    double v=(s->motors.left+s->motors.right)/2;
    double omega=(s->motors.right-s->motors.left)/RB_TRACK_M;
    double mid=s->sensors.heading+omega*dt/2;
    s->sensors.x+=v*cos(mid)*dt;s->sensors.y+=v*sin(mid)*dt;
    s->sensors.heading=remainder(s->sensors.heading+omega*dt,2*3.141592653589793);
    s->distance+=fabs(v)*dt;s->sensors.sample_ms=now;
}
