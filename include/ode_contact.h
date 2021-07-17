#include "common.h"

class ODEContact{
public:
	//this is the world coordinate of the origin of the contact force...
	GsVec cp;
	//this is the normal at the contact point
	GsVec n;
	//and this is the penetration depth
	double d;
	//this is the first rigid body that participated in the contact
	dBodyID rb1;
	//and this is the second...
	dBodyID rb2;
	//
	dJointID jid;
	//and this is the force applied (with f being applied to rb1, and -f to rb2)
	GsVec f;

	bool involves(dBodyID anID){return rb1==anID || rb2==anID;}

};