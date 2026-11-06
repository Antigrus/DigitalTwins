#include "sim.h"
#include <math.h>

Vec3 v3(float x, float y, float z) { Vec3 v = {x, y, z}; return v; }
Vec3 add(Vec3 a, Vec3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
Vec3 sub(Vec3 a, Vec3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
Vec3 scale(Vec3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }
float dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
float norm(Vec3 a) { return sqrtf(dot(a, a)); }
float deg(float r) { return r * 180.0f / PI; }

Mat3 compose_rot(Mat3 a, Mat3 b) {
    Mat3 r = {{{0}}};
    for (int i=0; i<3; ++i) for (int j=0; j<3; ++j)
        for (int k=0; k<3; ++k) r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}
static Mat3 rx(float a) {
    float c=cosf(a), s=sinf(a);
    Mat3 m={{{1,0,0},{0,c,-s},{0,s,c}}}; return m;
}
static Mat3 ry(float a) {
    float c=cosf(a), s=sinf(a);
    Mat3 m={{{c,0,s},{0,1,0},{-s,0,c}}}; return m;
}
static Mat3 rz(float a) {
    float c=cosf(a), s=sinf(a);
    Mat3 m={{{c,-s,0},{s,c,0},{0,0,1}}}; return m;
}
static Mat3 euler(Attitude a) {
    return compose_rot(compose_rot(rz(a.heading*PI/180.0f), ry(-a.pitch*PI/180.0f)), rx(a.roll*PI/180.0f));
}
Mat3 node_rot(Attitude a) { return euler(a); }
Mat3 array_rot(Attitude node, Attitude mount) { return compose_rot(node_rot(node), euler(mount)); }
Vec3 apply(Mat3 m, Vec3 v) {
    return v3(m.m[0][0]*v.x+m.m[0][1]*v.y+m.m[0][2]*v.z,
              m.m[1][0]*v.x+m.m[1][1]*v.y+m.m[1][2]*v.z,
              m.m[2][0]*v.x+m.m[2][1]*v.y+m.m[2][2]*v.z);
}
