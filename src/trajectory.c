#include "sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int load_trajectory(NodePosition *p, const char *path) {
    if (!p || !path) return 0;
    FILE *f=fopen(path,"r"); if (!f) { perror(path); return 0; }
    size_t cap=32; p->trajectory=malloc(cap*sizeof(*p->trajectory));
    if (!p->trajectory) { fclose(f); return 0; }
    char line[512];
    while (fgets(line,sizeof(line),f)) {
        float t,x,y,z;
        if (line[0]=='#' || sscanf(line,"%f,%f,%f,%f",&t,&x,&y,&z)!=4) continue;
        if (p->trajectory_count==cap) {
            cap*=2; TrajectoryPoint *q=realloc(p->trajectory,cap*sizeof(*q));
            if (!q) { free(p->trajectory); p->trajectory=NULL; fclose(f); return 0; }
            p->trajectory=q;
        }
        p->trajectory[p->trajectory_count++]=(TrajectoryPoint){t,{x,y,z}};
    }
    fclose(f);
    if (p->trajectory_count<2) return 0;
    for (size_t i=1;i<p->trajectory_count;i++)
        if (p->trajectory[i].t<=p->trajectory[i-1].t) return 0;
    return 1;
}

int update_state(Node *n, NodePosition *p, float t) {
    if (!n || !p || p->trajectory_count<2) return 0;
    size_t i=0;
    if (t>=p->trajectory[p->trajectory_count-1].t) i=p->trajectory_count-2;
    else while (i+1<p->trajectory_count-1 && p->trajectory[i+1].t<t) ++i;
    TrajectoryPoint a=p->trajectory[i], b=p->trajectory[i+1];
    float dt=b.t-a.t; if (dt<=0) return 0;
    float u=(t-a.t)/dt; if (u<0) u=0; if (u>1) u=1;
    p->position=(Vec3){a.p.x+(b.p.x-a.p.x)*u,a.p.y+(b.p.y-a.p.y)*u,a.p.z+(b.p.z-a.p.z)*u};
    n->velocity=(Vec3){(b.p.x-a.p.x)/dt,(b.p.y-a.p.y)/dt,(b.p.z-a.p.z)/dt};
    float speed=norm(n->velocity);
    if (speed>1e-6f) {
        n->attitude.heading=deg(atan2f(n->velocity.y,n->velocity.x));
        n->attitude.pitch=deg(atan2f(n->velocity.z,hypotf(n->velocity.x,n->velocity.y)));
        n->attitude.roll=0.0f;
    }
    return 1;
}
