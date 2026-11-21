#ifndef GEODESY_H
#define GEODESY_H
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { double x,y,z; } Vec3d;
typedef struct { double m[3][3]; } Mat3d;
typedef struct { double latitude_deg, longitude_deg, altitude_m; } Geodetic;
#define WGS84_A 6378137.0
#define WGS84_F (1.0/298.257223563)
#define WGS84_E2 (WGS84_F*(2.0-WGS84_F))
#define GEO_DEG_TO_RAD 0.017453292519943295769236907684886
#define GEO_RAD_TO_DEG 57.295779513082320876798154814105
Vec3d geodetic_to_ecef(Geodetic geo);
int ecef_to_geodetic(Vec3d ecef, Geodetic *geo);
Mat3d ned_to_ecef_matrix(double latitude_deg,double longitude_deg);
Mat3d ecef_to_ned_matrix(double latitude_deg,double longitude_deg);
Vec3d mat3d_apply(Mat3d m,Vec3d v);
Mat3d mat3d_transpose(Mat3d m);
Mat3d mat3d_multiply(Mat3d a,Mat3d b);
#ifdef __cplusplus
}
#endif
#endif
