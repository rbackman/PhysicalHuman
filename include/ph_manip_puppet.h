#pragma once
#include "ph_manip.h"

/*manipulator that connects a spring to an ODE rigid body. can be used on a PhysicalJoint*/

class ODESpring;
class SpringManip:public HumanManipulator
{
public:
	ODESpring* spring;
    SpringManip(ManipulatorModule* cont, PhysicalJoint* j,GsString file);
	bool evaluate();
	void applyParameters();
};
