#include "geodesy.h"
#include <math.h>
#include <stdio.h>
#define CHECK(v,e,t) do{if(fabs((v)-(e))>(t)){printf("FAIL line %d: %.12g != %.12g\n",__LINE__,(double)(v),(double)(e));return 1;}}while(0)
int main(void){Vec3d p=geodetic_to_ecef((Geodetic){0,0,0});CHECK(p.x,WGS84_A,1e-8);CHECK(p.y,0,1e-8);CHECK(p.z,0,1e-8);p=geodetic_to_ecef((Geodetic){90,0,0});CHECK(p.x,0,1e-7);CHECK(p.y,0,1e-7);CHECK(p.z,WGS84_A*(1-WGS84_F),1e-6);Geodetic g={0};p=geodetic_to_ecef((Geodetic){37.25,-122.1,1234.5});if(!ecef_to_geodetic(p,&g))return 1;CHECK(g.latitude_deg,37.25,1e-9);CHECK(g.longitude_deg,-122.1,1e-9);CHECK(g.altitude_m,1234.5,1e-5);Mat3d r=ned_to_ecef_matrix(0,0);Vec3d n=mat3d_apply(r,(Vec3d){1,0,0});CHECK(n.x,0,1e-12);CHECK(n.y,0,1e-12);CHECK(n.z,1,1e-12);Vec3d e=mat3d_apply(r,(Vec3d){0,1,0});CHECK(e.x,0,1e-12);CHECK(e.y,1,1e-12);CHECK(e.z,0,1e-12);Vec3d d=mat3d_apply(r,(Vec3d){0,0,1});CHECK(d.x,-1,1e-12);CHECK(d.y,0,1e-12);CHECK(d.z,0,1e-12);puts("geodesy tests passed");return 0;}
