#include "geodesy.h"
#include <math.h>
Vec3 geodetic_to_ecef(Geodetic g){double p=g.latitude_deg*GEO_DEG_TO_RAD,l=g.longitude_deg*GEO_DEG_TO_RAD,sp=sin(p),cp=cos(p),sl=sin(l),cl=cos(l),n=WGS84_A/sqrt(1-WGS84_E2*sp*sp);return (Vec3){(n+g.altitude_m)*cp*cl,(n+g.altitude_m)*cp*sl,(n*(1-WGS84_E2)+g.altitude_m)*sp};}
int ecef_to_geodetic(Vec3 p,Geodetic *g){if(!g||!isfinite(p.x)||!isfinite(p.y)||!isfinite(p.z))return 0;double lon=atan2(p.y,p.x),rho=hypot(p.x,p.y);if(rho<1e-9){g->latitude_deg=p.z>=0?90:-90;g->longitude_deg=0;g->altitude_m=fabs(p.z)-WGS84_A*sqrt(1-WGS84_E2);return 1;}double lat=atan2(p.z,rho*(1-WGS84_E2));for(int i=0;i<15;i++){double s=sin(lat),n=WGS84_A/sqrt(1-WGS84_E2*s*s),h=rho/cos(lat)-n,ln=atan2(p.z,rho*(1-WGS84_E2*n/(n+h)));if(fabs(ln-lat)<1e-14){lat=ln;break;}lat=ln;}double s=sin(lat),n=WGS84_A/sqrt(1-WGS84_E2*s*s),h=rho/cos(lat)-n;g->latitude_deg=lat*GEO_RAD_TO_DEG;g->longitude_deg=lon*GEO_RAD_TO_DEG;g->altitude_m=h;return 1;}
Mat3 ned_to_ecef_matrix(double latd,double lond){double p=latd*GEO_DEG_TO_RAD,l=lond*GEO_DEG_TO_RAD,sp=sin(p),cp=cos(p),sl=sin(l),cl=cos(l);Mat3 r={{{-sp*cl,-sl,-cp*cl},{-sp*sl,cl,-cp*sl},{cp,0,-sp}}};return r;}
Mat3 ecef_to_ned_matrix(double lat,double lon){return mat3_transpose(ned_to_ecef_matrix(lat,lon));}
Vec3 mat3_apply(Mat3 m,Vec3 v){return (Vec3){m.m[0][0]*v.x+m.m[0][1]*v.y+m.m[0][2]*v.z,m.m[1][0]*v.x+m.m[1][1]*v.y+m.m[1][2]*v.z,m.m[2][0]*v.x+m.m[2][1]*v.y+m.m[2][2]*v.z};}
Mat3 mat3_transpose(Mat3 m){Mat3 r;for(int i=0;i<3;i++)for(int j=0;j<3;j++)r.m[i][j]=m.m[j][i];return r;}
Mat3 mat3_multiply(Mat3 a,Mat3 b){Mat3 r={{{0}}};for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)r.m[i][j]+=a.m[i][k]*b.m[k][j];return r;}
