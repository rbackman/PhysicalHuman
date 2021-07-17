#include "common.h"
#include "util.h"


bool arrayContains(GsStrings *s,GsString str)
{
	for(int i=0;i<s->size();i++)
	{
		if(str == s->get(i))
			return true;
	}
	return false;
}
float boundToRange(float in,float min,float max)
{
	if(in<min)return min;
	if(in>max)return max;
	return in;
}
GsVec boundToRange(GsVec in,GsVec min,GsVec max)
{
	return GsVec(boundToRange(in.x,min.x,max.x),boundToRange(in.y,min.y,max.y),boundToRange(in.z,min.z,max.z)) ;
}

GsVec boundToRange( GsVec in,float min,float max )
{
	return boundToRange(in,GsVec(min,min,min),GsVec(max,max,max));
}

/** Bilateral filter. */
void bilateral_filter(
        GsImage *inImg,     /**< Input: input image */
        double sigd,        /**< Input: spatial sigma */
        double sigr,        /**< Input: edge sigma */
        int hh,             /**< Input: half-height of window to use */
        int hw              /**< Input: half-width of window to use */   
        )
{
	GsImage* tmpImg = new GsImage;

	tmpImg->init(inImg->w(),inImg->h());

	int nRows = tmpImg->h();
	int nCols = tmpImg->w();
	int res;
    int row, col, erow, ecol;
    double num, den, c, s;

    /* ignore pixels too close to the edge, since we can't fit a whole window
       around them. (could be smarter and do symmetry or something) */
    for( row = hh; row < nRows - hh; row++ ) {
        for( col = hw; col < nCols - hw; col++ ) {
            /* integrate over the window */
            num = 0;
            den = 0;
            erow = row;
            for(ecol = col - hw; ecol < col + hw; ecol++) {
                /* compute the spatial weight for this pixel */
                s = exp(-0.5 * pow(abs(ecol - col) / sigd, 2));

                /* compute the edge weight for this pixel */
                c = exp(-0.5 *  pow( abs(inImg->ptpixel(erow,ecol)->r  -  inImg->ptpixel(row,col)->r) / sigr  , 2 ));

                /* update the numerator and denominator */
                num += inImg->ptpixel(erow,ecol)->r * c * s;
                den += c * s;
            }

            /* fill in the filtered value for this pixel */
			res = (int) (num / den);
            tmpImg->ptpixel(row,col)->set(res,res,res);
        }
    }

    /* now do our second pass, this time for vertical filtering */
    for( row = hh; row < nRows - hh; row++ ) {
        for( col = hw; col < nCols - hw; col++ ) {

            /* integrate over the window */
            num = 0;
            den = 0;
            ecol = col;
            for(erow = row - hh; erow < row + hh; erow++) {
                /* compute the spatial weight for this pixel */
                s = exp(-0.5 * pow(abs(erow -row) / sigd, 2));

                /* compute the edge weight for this pixel */
                c = exp(-0.5 * 
                        pow( abs(tmpImg->ptpixel(erow,ecol)->r - 
                                tmpImg->ptpixel(row,col)->r) / sigr
                            , 2 ));

                /* update the numerator and denominator */
                num += tmpImg->ptpixel(erow,ecol)->r * c * s;
                den += c * s;
            }


            /* fill in the filtered value for this pixel */
		
			res = (int) (num / den);
			inImg->ptpixel(row,col)->set(res,res,res);
			
        }
    }
	delete tmpImg;
   
}
float bound(float b,float max){return boundToRange(b,-max,max);}
void swapP(void * pOne, void * pTwo)
{
	void * pTemp;
	pTemp = pOne;
	pOne = pTwo;
	pTwo = pTemp;
}


GsVec interp(GsVec start, GsVec end , float t)
{
	return start + (end - start)*t;;
}
float interp(float start,float end, float t)
{
	return start + (end-start)*t;
}
float interp_cubic ( float t, float tmin, float tmax )
{
	t = (t-tmin)/(tmax-tmin);    // normalize t to [0,1]
	t=-(2.0f*(t*t*t)) + (3.0f*(t*t));  // cubic spline
	return t*(tmax-tmin) + tmin; // scale back
}

GsVec GetClosetPoint(GsVec A, GsVec B, GsVec P, bool segmentClamp)
{
	GsVec AP = P - A;
	GsVec AB = B - A;
	float ab2 = AB.x*AB.x + AB.y*AB.y;
	float ap_ab = AP.x*AB.x + AP.y*AB.y;
	float t = ap_ab / ab2;
	if (segmentClamp)
	{
		if (t < 0.0f) t = 0.0f;
		else if (t > 1.0f) t = 1.0f;
	}
	GsVec Closest = A + AB * t;
	return Closest;
}

long int fact(int n)
{
	if (n<=1)
		return(1);
	else
		n=n*fact(n-1);
	return(n);
}

bool point_plane_check(const plane_t &pl, const GsVec &p)
{
	float x  = p.x;     float y  = p.y;	  float z  = p.z;

	float x1 = pl.p1.x; float y1 = pl.p1.y; float z1 = pl.p1.z;
	float x2 = pl.p2.x; float y2 = pl.p2.y; float z2 = pl.p2.z;
	float x3 = pl.p3.x; float y3 = pl.p3.y; float z3 = pl.p3.z;

	GsMat m(x  - x1, y  - y1, z  - z1, 0,
		x2 - x1, y2 - y1, z2 - z1, 0,
		x3 - x1, y3 - y1, z3 - z1, 0,
		0,       0,	0, 1);

	if (m.det3x3() < 0)
		return false;
	else
		return true;
}

inline GsQuat mirror_quat(const GsQuat &q)
{
	return GsQuat(-q.w,-q.x,q.y,q.z);
}


void translateMotionToOrigin(KnMotion* motion,GsVec origin)
{
	motion->apply_frame(0);
	motion->skeleton()->update_global_matrices();
	GsVec motion_start_pos = motion->skeleton()->root()->gcenter();	
	translate_motion(motion,-motion_start_pos+origin); 

}

void orientMotionToOrigin( KnMotion* motion  )
{
	motion->apply_frame(0);
	motion->skeleton()->update_global_matrices();
	GsQuat motion_start_rot = motion->skeleton()->root()->rot()->value();			
	rotate_motion(motion,decomposeRotation(motion_start_rot,GsVec::j).inverse());	
}
void translate_motion(KnMotion *motion, GsVec pos)
{
	KnSkeleton *sk = motion->skeleton();
	GsVec val;
	for (unsigned int i = 0; i <  motion->frames(); i++)
	{
		motion->apply_frame(i);
		sk->root()->pos()->value(sk->root()->pos()->value()+pos);
		motion->posture(i)->get();
	}	
}

void rotate_motion(KnMotion *motion, GsQuat rot)
{
	KnSkeleton *sk = motion->skeleton();

	GsVec pos;
	for (unsigned int i = 0; i < motion->frames(); i++) {
		motion->apply_frame(i);
		pos = sk->root()->pos()->value();
		pos = rot.apply(pos);

		sk->root()->rot()->value(rot*sk->root()->rot()->value());
		sk->root()->pos()->value(pos);

		motion->posture(i)->get();
	}
}
int scale_joints(KnJoint *joint, float factor)
{
	GsVec offset = joint->offset();
	joint->offset(offset / factor);

	for (int i = 0; i < joint->children(); i++)
		scale_joints(joint->child(i), factor);

	return 0;
}

void scale_motion(KnMotion *mtn, float factor)
{
	KnSkeleton *sk = mtn->skeleton();
	int size = mtn->frames();
	for (int i = 0; i < size; ++i) 
	{
		mtn->apply_frame(i);
		GsVec v = sk->root()->pos()->value();
		sk->root()->pos()->value(v * factor);
		mtn->posture(i)->get();
	}

	//scale_skeleton(sk, factor);


}


rotation_type gs_euler_order_to_channel_dof( gsEulerOrder e )
{
	switch(e)
	{
	case gsXYZ: return ROT_XYZ; break;
	case gsXZY:return ROT_XZY; break;
	case gsYXZ: return ROT_YXZ; break;
	case gsZXY:return ROT_ZXY; break;
	case gsZYX:return ROT_ZYX; break;
	}
	return ROT_XYZ;
}

gsEulerOrder rotation_type_to_gs_euler_order( rotation_type e )
{
	switch(e)
	{
	case ROT_XYZ : return gsXYZ; break;
	case ROT_XZY :return gsXZY; break;
	case ROT_YXZ: return gsYXZ ; break;
	case ROT_ZXY:return  gsZXY; break;
	case ROT_ZYX :return gsZYX; break;
	}
	return gsXYZ;
}

rotation_type joint_euler_type_to_rotation_type( KnJointEuler::Type t )
{
	switch(t)
	{
	case KnJointEuler::TypeXYZ: return ROT_XYZ;
	case KnJointEuler::TypeYXZ: return ROT_YXZ;
	case KnJointEuler::TypeZY: return ROT_ZYX;
	case KnJointEuler::TypeYZX: return ROT_YZX;
	}
	return ROT_XYZ;
}

GsQuat decomposeRotation( GsQuat q, GsVec vB )
{
	GsVec vA = q.apply(vB);
	vA.normalize();

	float temp = 0;

	//compute the rotation that aligns the vector v in the two coordinate frames (A and T)
	GsVec rotAxis =  cross(vA,vB);
	rotAxis.normalize();

	float rotAngle = -safeACOS(dot(vA,vB));

	GsQuat TqA = GsQuat( rotAxis*(-1),rotAngle);
	TqA = TqA*q;
	TqA.normalize();
	return TqA;
}

GsVec computePDTorque( float dt, GsQuat qCurrent, GsQuat qDesired, GsVec wRel, GsVec wRelD,float kp,float kd )
{
	GsVec torque(0,0,0);
	GsQuat qErr = qCurrent.conjugate() * qDesired;
	qErr.normalize();

	if (qErr.angle()<=0.01)
	{
		//avoid the divide by close-to-zero. The orientations match, so the proportional component of the torque should be 0
	}else
	{
		torque = qErr.axis() * qErr.angle() * kp * (float)SGN(qErr.w);
		torque = qCurrent.apply(torque);
	}

	//the angular velocities are stored in parent coordinates, so it is ok to add this term now
	torque += (wRelD - wRel) * kd;

	return torque * dt * 100;
}

GsQuat vecToQuat( GsVec v,gsEulerOrder e )
{
	GsQuat q;
	gs_rot(e,q,v.x,v.y,v.z);
	q.normalize();
	return q;
}

GsVec vecMult( GsVec a,GsVec b )
{
	return GsVec(a.x*b.x,a.y*b.y,a.z*b.z);
}

float safeACOS( float val )
{
	if (val<-1)
		return (float)GS_PI;
	if (val>1)
		return 0;
	return acos(val);
}

void copyJointAngles( KnSkeleton* _dest,KnSkeleton* _origin )
{
	_dest->root()->pos()->value(_origin->root()->pos()->value());

	for(int i=0;i<_origin->joints().size();i++)
	{
		KnJoint* oj = _origin->joints()[i];
		KnJoint* dj = _dest->joint(oj->name());
		if(dj)
			dj->rot()->value(oj->rot()->value());
	}
}

GsString randomString(int num)
{
	GsString s;
	int sub = ((int)(gs_time()/num))*num;
	double t = gs_time() - sub;
	int n = (int)(t*num)%num;
	s<< n;
	
	return s;
}

int col_dist( GsColor a,GsColor b )
{
	return abs(a.r - b.r) + abs(a.g - b.g)+abs(a.b - b.b);
}

double findnoise2( double x,double y )
{
	int n=(int)x+(int)y*57;
	n=(n<<13)^n;
	int nn=(n*(n*n*60493+19990303)+1376312589)&0x7fffffff;
	return 1.0-((double)nn/1073741824.0);
}

double interpolate( double a,double b,double x )
{
	double ft=x * 3.1415927;
	double f=(1.0-cos(ft))* 0.5;
	return a*(1.0-f)+b*f;
}

double noise( double x,double y )
{
	double floorx=(double)((int)x);//This is kinda a cheap way to floor a double integer.
	double floory=(double)((int)y);
	double s,t,u,v;//Integer declaration
	s=findnoise2(floorx,floory); 
	t=findnoise2(floorx+1,floory);
	u=findnoise2(floorx,floory+1);//Get the surrounding pixels to calculate the transition.
	v=findnoise2(floorx+1,floory+1);
	double int1=interpolate(s,t,x-floorx);//Interpolate between the values.
	double int2=interpolate(u,v,x-floorx);//Here we use x-floorx, to get 1st dimension. Don't mind the x-floorx thingie, it's part of the cosine formula.
	return interpolate(int1,int2,y-floory);//Here we use y-floory, to get the 2nd dimension.
}

void scaleImage( GsImage* newImage,GsImage* oldImage,int w,int h )
{

}

