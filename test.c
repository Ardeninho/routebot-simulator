#include "routebot.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
static Mission square(void) { return (Mission){.stops={{0,0},{1,1},{1,0},{0,1}},.count=4}; }
static void zero(Motors m) { assert(m.left==0 && m.right==0); }
static void permutation(Route r,size_t n) {
    bool seen[RB_MAX_STOPS]={false};assert(r.count==n+1 && r.order[0]==0 && r.order[n]==0);
    for(size_t i=0;i<n;i++) { assert(r.order[i]<n && !seen[r.order[i]]);seen[r.order[i]]=true; }
}
int main(void) {
    Mission m=square();Route r;
    for(int method=GIVEN;method<=INSERT;method++) { assert(plan(&m,(Method)method,&r));permutation(r,4);if(method!=GIVEN) assert(fabs(r.length-4)<1e-10); }
    m.count=RB_MAX_STOPS;
    for(size_t i=0;i<m.count;i++) m.stops[i]=(Point){(double)i,0};
    for(int method=GIVEN;method<=INSERT;method++) { assert(plan(&m,(Method)method,&r));permutation(r,m.count);assert(fabs(r.length-62)<1e-10); }
    m.count=1;assert(!plan(&m,GIVEN,&r));m=square();m.stops[0].x=NAN;assert(!plan(&m,GIVEN,&r));
    m=square();for(size_t i=0;i<m.count;i++) m.stops[i]=(Point){0,0};assert(plan(&m,INSERT,&r));permutation(r,4);assert(r.length==0);
    m=square();assert(plan(&m,INSERT,&r));
    Controller c;Sim s={0};Hal h=sim_hal(&s);
    assert(controller_start(&c,&m,&r,0));
    s.sensors.obstacle=true;controller_tick(&c,&h,0);assert(c.state==BLOCKED);zero(s.motors);
    s.sensors.obstacle=false;controller_tick(&c,&h,0);assert(c.state==RUNNING);
    s.sensors.estop=true;controller_tick(&c,&h,0);assert(c.state==FAULT && c.fault==ESTOP_FAULT);zero(s.motors);
    s.sensors.estop=false;controller_tick(&c,&h,0);assert(c.state==FAULT);zero(s.motors);
    assert(controller_start(&c,&m,&r,0));controller_tick(&c,&h,101);assert(c.fault==SENSOR_FAULT);zero(s.motors);
    assert(controller_start(&c,&m,&r,0));s.sensors.sample_ms=300001;controller_tick(&c,&h,300001);assert(c.fault==TIMEOUT_FAULT);zero(s.motors);
    assert(controller_start(&c,&m,&r,0));s.dropout=true;controller_tick(&c,&h,0);assert(c.fault==SENSOR_FAULT);zero(s.motors);s.dropout=false;
    assert(controller_start(&c,&m,&r,0));s.sensors.x=NAN;controller_tick(&c,&h,0);assert(c.fault==SENSOR_FAULT);zero(s.motors);
    s=(Sim){0};s.motors=(Motors){0.5,0.5};sim_step(&s,101);zero(s.motors);assert(s.distance==0);
    /* Unsigned elapsed time remains valid across a 32-bit timer wrap. */
    s=(Sim){0};s.sensors.sample_ms=UINT32_MAX-10;
    assert(controller_start(&c,&m,&r,UINT32_MAX-10));controller_tick(&c,&h,9);assert(c.state==RUNNING);
    r.order[1]=r.order[0];assert(!controller_start(&c,&m,&r,0));
    puts("Planner, limits, stop/resume, fault latching, stale sensor, timeout, watchdog, and timer-wrap checks passed.");
}
