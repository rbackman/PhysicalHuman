#pragma once

#include "common.h"

#include "ode_object.h"

class Ball;
class Capsule;
class Line;

class ODESpring 
{
public:

	SnGroup* grp;
	Ball* p1Model;
	Ball* p2Model;
	Line* line;

	ODEWorld* world;

	ODEObject *b1;/*b1 should always be set otherwise it does nothing*/
	GsVec p1; /*the offset of b1*/
	ODEObject* b2;
	GsVec p2; /*in case the second object is not a body*/


	bool active;
	float lin_k;
	float  lin_d;

	bool use_orientation;
	GsQuat orientation_desired;
	float rot_k;
	float rot_d;

	float rest_length;
	float break_length;
	GsVec force;
	GsVec torque;
	float last_extent;



	ODESpring(ODEWorld* world, ODEObject *rb1,GsVec p2);
	ODESpring(ODEWorld* world, ODEObject *rb1 ,ODEObject* rb2);

	
	bool update();
};