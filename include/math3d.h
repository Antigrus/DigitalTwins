#ifndef MATH3D_H
#define MATH3D_H
#include "sim_types.h"
Vec3 v3(double x,double y,double z);Vec3 add(Vec3 a,Vec3 b);Vec3 sub(Vec3 a,Vec3 b);Vec3 scale(Vec3 a,double s);double dot(Vec3 a,Vec3 b);double norm(Vec3 a);float deg(double radians);
Mat3 node_rot(Attitude a);Mat3 scan_rot(Attitude scan);Mat3 array_rot(Attitude node,Attitude mount);Mat3 compose_rot(Mat3 a,Mat3 b);Vec3 apply(Mat3 matrix,Vec3 vector);Mat3 node_rot_ecef(Attitude a,Geodetic geo);Mat3 array_rot_ecef(Attitude node,Geodetic geo,Attitude mount);
#endif
