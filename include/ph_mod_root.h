#pragma once
#include "ph_mod.h"

//This controller is specific for the hips to deal with single stance mode where torques get 
//large

class PID;
class SnTransform;

class RootModule : public Module
{
public:
	RootModule(PhysicalHuman* human,const char* file);
	~RootModule(void);
	bool evaluate();

private:
	void computeHipTorques(GsQuat qdes);
	GsString stateString();
	PID* rootPID;
	GsVec previousOffset;
	SnTransform* rootTransform; //global frame for root to achieve
};
