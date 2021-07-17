#pragma once
#include "ph_mod.h"
/*
this controller is responsible for computing the gravity compensation torques
that is depending on the orientation of the joints the force required to have the character
stay in position without falling. so for example to maintain a given setpoint when the characters
hand is straight out to the side takes more torque than if the arm is pointed down because of the
affective moment.
*/

class GravityModule : public Module
{
public:
	GravityModule(PhysicalHuman* human,const char* file);
	~GravityModule(void);
	bool evaluate();
	float mag;
private:
};
