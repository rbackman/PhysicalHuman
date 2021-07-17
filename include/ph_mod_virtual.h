#pragma once
#include "ph_mod.h"
#include "util_manipulator.h"
#include "ph_mod_manip.h"

//The VirtualModule is similar in principal to the puppet controller but instead of applying forces
//directly it calculates the torques needed to achieve that virtual force in a set of joints with the equation t=J.inverse()*joint_virtual_force

class VirtualModule : public ManipulatorModule
{
public:	
	VirtualModule(PhysicalHuman* human,GsString file);
	~VirtualModule(void);
	void applyParameters();
	HumanManipulator* makeManip(GsString name,GsString file);
	bool evaluate();
	bool virtualForce(GsVec force,GsVec pos,PhysicalJoint* effector,PhysicalJoint* base);
};
