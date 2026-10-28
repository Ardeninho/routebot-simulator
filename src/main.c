#include "routebot.h"
#include <stdio.h>
#include <string.h>
int main(int argc,char **argv) {
    const char *scenario=argc>1?argv[1]:"normal";
    if(argc>2 || (strcmp(scenario,"normal") && strcmp(scenario,"obstacle") && strcmp(scenario,"estop") && strcmp(scenario,"dropout"))) {
        fprintf(stderr,"Usage: %s [normal|obstacle|estop|dropout]\n",argv[0]);return 2;
    }
    Mission mission={.stops={{0,0},{6,4},{1,4},{6,0},{3,2},{0,2}},.count=6};
    Route route;
    const char *methods[]={"given","nearest","insert"};
    for(int m=GIVEN;m<=INSERT;m++) {
        if(!plan(&mission,(Method)m,&route)) return 2;
        fprintf(stderr,"%s: %.2f m\n",methods[m],route.length);
    }
    Controller c;
    if(!controller_start(&c,&mission,&route,0)) return 2;
    Sim sim={0};Hal hal=sim_hal(&sim);
    puts("time_s,x_m,y_m,heading_rad,left_mps,right_mps,state,target,fault");
    for(uint32_t now=0;now<=301000;now+=RB_STEP_MS) {
        sim.sensors.obstacle=!strcmp(scenario,"obstacle") && now>=3000 && now<6000;
        sim.sensors.estop=!strcmp(scenario,"estop") && now>=3000;
        sim.dropout=!strcmp(scenario,"dropout") && now>=3000;
        controller_tick(&c,&hal,now);
        printf("%.2f,%.5f,%.5f,%.5f,%.4f,%.4f,%s,%zu,%d\n",now/1000.0,sim.sensors.x,sim.sensors.y,sim.sensors.heading,c.command.left,c.command.right,state_name(c.state),c.target,c.fault);
        if(c.state==COMPLETE || c.state==FAULT) {
            fprintf(stderr,"%s: %.2f s, %.2f m traveled, fault=%d\n",state_name(c.state),now/1000.0,sim.distance,c.fault);
            return c.state==COMPLETE?0:1;
        }
        sim_step(&sim,now+RB_STEP_MS);
    }
    return 2;
}
