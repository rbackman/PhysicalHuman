#pragma once
#include "ph_manip.h"

/*
this manipulator should work similarly to the SpringManip but instead of applying a force directly to the
rigid body it converts the force desired to equivalent joint torques by using the Jacobian transpose(which is 
equivalent to a ccd like cross product) of the PhysicalJoints between the effector and base.
*/
class VirtualSpring : public HumanManipulator
{
public:

	PhysicalJoint* effector;
	PhysicalJoint* base;

	Ball* p1Model;
	Ball* p2Model;
	Line* line;

	VirtualSpring(PhysicalHuman* human,VirtualModule* cont,GsString nme,GsString file);
	void init();
	void activate();
	bool evaluate();
};
