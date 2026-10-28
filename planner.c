#include "routebot.h"
#include <math.h>
#include <string.h>
static double distance(Point a, Point b) { return hypot(a.x-b.x, a.y-b.y); }
static double edge(const Mission *m, size_t a, size_t b) { return distance(m->stops[a],m->stops[b]); }
bool plan(const Mission *m, Method method, Route *r) {
    if (!m || !r || m->count < 2 || m->count > RB_MAX_STOPS || method < GIVEN || method > INSERT) return false;
    for (size_t i=0;i<m->count;i++)
        if (!isfinite(m->stops[i].x) || !isfinite(m->stops[i].y) || fabs(m->stops[i].x)>1000 || fabs(m->stops[i].y)>1000) return false;
    size_t n=m->count;
    memset(r,0,sizeof(*r));
    r->count=n+1;
    bool used[RB_MAX_STOPS]={false};
    if (method==GIVEN) for(size_t i=0;i<n;i++) r->order[i]=i;
    if (method==NEAREST) {
        used[0]=true;
        for(size_t k=1;k<n;k++) {
            size_t best=n; double d=INFINITY;
            for(size_t i=0;i<n;i++) if(!used[i] && edge(m,r->order[k-1],i)<d) { best=i;d=edge(m,r->order[k-1],i); }
            r->order[k]=best;used[best]=true;
        }
    }
    if (method==INSERT) {
        size_t a=0,b=1;double shortest=INFINITY;
        for(size_t i=0;i<n;i++) for(size_t j=i+1;j<n;j++)
            if(edge(m,i,j)<shortest) { a=i;b=j;shortest=edge(m,i,j); }
        r->order[0]=a;r->order[1]=b;used[a]=used[b]=true;
        double near[RB_MAX_STOPS];
        for(size_t i=0;i<n;i++) near[i]=fmin(edge(m,i,a),edge(m,i,b));
        for(size_t k=2;k<n;k++) {
            size_t next=n;double d=INFINITY;
            for(size_t i=0;i<n;i++) if(!used[i] && near[i]<d) { next=i;d=near[i]; }
            size_t after=0;double increase=INFINITY;
            for(size_t i=0;i<k;i++) {
                size_t x=r->order[i],y=r->order[(i+1)%k];
                double delta=edge(m,x,next)+edge(m,next,y)-edge(m,x,y);
                if(delta<increase) { increase=delta;after=i; }
            }
            memmove(&r->order[after+2],&r->order[after+1],(k-after-1)*sizeof(size_t));
            r->order[after+1]=next;used[next]=true;
            for(size_t i=0;i<n;i++) if(!used[i]) near[i]=fmin(near[i],edge(m,i,next));
        }
    }
    size_t start=0,copy[RB_MAX_STOPS];
    for(size_t i=0;i<n;i++) { copy[i]=r->order[i];if(copy[i]==0) start=i; }
    for(size_t i=0;i<n;i++) r->order[i]=copy[(start+i)%n];
    if(r->order[1]>r->order[n-1]) for(size_t i=1,j=n-1;i<j;i++,j--) { size_t t=r->order[i];r->order[i]=r->order[j];r->order[j]=t; }
    r->order[n]=0;
    for(size_t i=0;i<n;i++) r->length+=edge(m,r->order[i],r->order[i+1]);
    return true;
}
