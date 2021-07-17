
#pragma once

#include "common.h"

//# define GS_USE_TRACE1  // keyword tracin

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define SGN(x) (((x)<0)?(-1.0f):(1.0f))
#define ROUND2(x) gs_round(x,0.01f)
#define TO_DEG(x) ROUND2( GS_TODEG(x) )
#define PI 3.14159265358979323846f
#define	DEG2RAD(x) (((x)*PI)/180.0)
#define	RAD2DEG(x) (((x)*180.0)/PI)

inline float bump ( float t ) // f([0,1])->[0,1]
{
	return 1.0f - pow ( 2.0f*t - 1, 4.0f ); //exponent has to be pair: 2 (low extremity speed), 4, 6, 8 (quicker), ...
}


float boundToRange(float in,float min,float max);
GsVec boundToRange(GsVec in,GsVec min,GsVec max);
GsVec boundToRange(GsVec in,float min,float max);
float bound(float b,float max);
int col_dist(GsColor a,GsColor b);

GsVec interp(GsVec start, GsVec end , float t);
float interp(float start,float end, float t);
float interp_cubic ( float t, float tmin, float tmax );
GsVec GetClosetPoint(GsVec A, GsVec B, GsVec P, bool segmentClamp);
long int fact(int n);
GsVec vecMult(GsVec a,GsVec b);
void bilateral_filter( GsImage *inImg,  double sigd,  double sigr,  int hh,  int hw  );
float safeACOS(float val);
void swapP(void * pOne, void * pTwo);
GsQuat vecToQuat(GsVec v,gsEulerOrder e);
GsQuat decomposeRotation(GsQuat q, GsVec vB);
void scaleImage(GsImage* newImage,GsImage* oldImage,int w,int h);
bool arrayContains(GsStrings *s,GsString str);
//this should calculate the torque needed to drive gRel to qDesired 
//torque should be returned in qCurrent coordinates
GsVec computePDTorque(float dt, GsQuat qCurrent, GsQuat qDesired, GsVec wRel, GsVec wRelD,float kp,float kd);

inline float RB(float d,float support,float height)
{
	return (float)(height*exp(-support*(d*d)));
}

/* defines a plane determined by three 3D points. */
struct plane_t {
	GsVec p1;
	GsVec p2;
	GsVec p3;
};

bool point_plane_check(const plane_t &pl, const GsVec &p);


void mirror_posture(KnPosture* p);

void translate_motion(KnMotion *motion,GsVec pos);
void rotate_motion(KnMotion *motion,GsQuat rot);
void translateMotionToOrigin(KnMotion* motion ,GsVec p);
void orientMotionToOrigin( KnMotion* motion );
void scale_motion(KnMotion *mtn, float factor);

void copyJointAngles( KnSkeleton* _dest,KnSkeleton* _origin );
rotation_type gs_euler_order_to_rotation_type(gsEulerOrder e);
gsEulerOrder rotation_type_to_gs_euler_order(rotation_type e);
rotation_type joint_euler_type_to_rotation_type( KnJointEuler::Type t);

GsString randomString(int num = 100);


double findnoise2(double x,double y);

double interpolate(double a,double b,double x);


double noise(double x,double y);