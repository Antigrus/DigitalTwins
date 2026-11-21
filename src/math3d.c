#include "math3d.h"
#include <math.h>
Vec3 v3(double x,double y,double z){return (Vec3){x,y,z};}Vec3 add(Vec3 a,Vec3 b){return v3(a.x+b.x,a.y+b.y,a.z+b.z);}Vec3 sub(Vec3 a,Vec3 b){return v3(a.x-b.x,a.y-b.y,a.z-b.z);}Vec3 scale(Vec3 a,double s){return v3(a.x*s,a.y*s,a.z*s);}double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}double norm(Vec3 a){return sqrt(dot(a,a));}float deg(double r){return (float)(r*GEO_RAD_TO_DEG);}
Mat3 compose_rot(Mat3 a,Mat3 b){return mat3_multiply(a,b);}Vec3 apply(Mat3 m,Vec3 v){return mat3_apply(m,v);}
static Mat3 rx(double a){double c=cos(a),s=sin(a);Mat3 m={{{1,0,0},{0,c,-s},{0,s,c}}};return m;}static Mat3 ry(double a){double c=cos(a),s=sin(a);Mat3 m={{{c,0,s},{0,1,0},{-s,0,c}}};return m;}static Mat3 rz(double a){double c=cos(a),s=sin(a);Mat3 m={{{c,-s,0},{s,c,0},{0,0,1}}};return m;}
static Mat3 euler(Attitude a){double k=GEO_DEG_TO_RAD;return compose_rot(compose_rot(rz(a.heading*k),ry(a.pitch*k)),rx(a.roll*k));}
Mat3 node_rot(Attitude a){return euler(a);}Mat3 array_rot(Attitude n,Attitude m){return compose_rot(node_rot(n),euler(m));} Mat3 scan_rot(Attitude a){double k=GEO_DEG_TO_RAD;return compose_rot(compose_rot(rz(a.heading*k),ry(-a.pitch*k)),rx(a.roll*k));}
Mat3 node_rot_ecef(Attitude a,Geodetic g){return compose_rot(ned_to_ecef_matrix(g.latitude_deg,g.longitude_deg),node_rot(a));}
Mat3 array_rot_ecef(Attitude n,Geodetic g,Attitude m){return compose_rot(node_rot_ecef(n,g),euler(m));}
