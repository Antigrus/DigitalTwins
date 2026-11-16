#ifndef MATH3D_H
#define MATH3D_H
#include "sim_types.h"
typedef struct { float m[3][3]; } Mat3;
Vec3 v3(float x,float y,float z);
Vec3 add(Vec3 a,Vec3 b);
Vec3 sub(Vec3 a,Vec3 b);
Vec3 scale(Vec3 a,float s);
float dot(Vec3 a,Vec3 b);
float norm(Vec3 a);
float deg(float radians);
Mat3 node_rot(Attitude attitude);
Mat3 array_rot(Attitude node,Attitude mount);
Mat3 compose_rot(Mat3 a,Mat3 b);
Vec3 apply(Mat3 matrix,Vec3 vector);
#endif
