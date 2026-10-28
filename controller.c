#include "routebot.h"
#include <math.h>
#include <string.h>
static double clamp(double x,double lo,double hi) { return fmax(lo,fmin(hi,x)); }
static void fail(Controller *c,Fault reason) { c->state=FAULT;c->fault=reason; }
bool controller_start(Controller *c,const Mission *m,const Route *r,uint32_t now) {
    if(!c || !m || !r) return false;
    memset(c,0,sizeof(*c));
    Route check;
    if(!plan(m,GIVEN,&check) || r->count!=m->count+1 || r->order[0]!=0 || r->order[m->count]!=0) return false;
    bool seen[RB_MAX_STOPS]={false};
    for(size_t i=0;i<m->count;i++) {
        size_t k=r->order[i];if(k>=m->count || seen[k]) return false;seen[k]=true;
    }
    c->mission=*m;c->route=*r;c->state=RUNNING;c->target=1;c->start_ms=now;
    return true;
}
void controller_tick(Controller *c,const Hal *hal,uint32_t now) {
    c->command=(Motors){0,0};
    if(c->state==RUNNING || c->state==BLOCKED) {
        Sensors s;
        if(!hal->read(hal->context,&s) || !isfinite(s.x) || !isfinite(s.y) || !isfinite(s.heading) || (uint32_t)(now-s.sample_ms)>100u) fail(c,SENSOR_FAULT);
        else if(s.estop) fail(c,ESTOP_FAULT);
        else if((uint32_t)(now-c->start_ms)>300000u) fail(c,TIMEOUT_FAULT);
        else if(s.obstacle) c->state=BLOCKED;
        else {
            c->state=RUNNING;
            Point goal=c->mission.stops[c->route.order[c->target]];
            double dx=goal.x-s.x,dy=goal.y-s.y,d=hypot(dx,dy);
            if(d<0.06) {
                c->target++;
                if(c->target==c->route.count) c->state=COMPLETE;
            } else {
                double error=remainder(atan2(dy,dx)-s.heading,2*3.141592653589793);
                double v=fmin(0.6,d)*fmax(0,cos(error));
                if(fabs(error)>0.7) v=0;
                double omega=clamp(3*error,-2,2);
                c->command.left=clamp(v-omega*RB_TRACK_M/2,-RB_MAX_SPEED,RB_MAX_SPEED);
                c->command.right=clamp(v+omega*RB_TRACK_M/2,-RB_MAX_SPEED,RB_MAX_SPEED);
            }
        }
    }
    hal->write(hal->context,c->command);
}
const char *state_name(State s) {
    switch(s) { case IDLE:return "IDLE";case RUNNING:return "RUNNING";case BLOCKED:return "BLOCKED";case COMPLETE:return "COMPLETE";case FAULT:return "FAULT"; }
    return "UNKNOWN";
}
