#pragma once
#include "ph_mod.h"

/*
The Balance controller is responsible for calculating the torques necessary in the legs to move the
the COM towards its desired location. it also accounts for a linear offset in joint angles depending
on if the right or left is the stance foot, this is to calm the battle between the reference controller
trying to keep the legs straight and the balance controller trying to rotate them.
*/
class BalanceModule : public Module
{
public:
	BalanceModule(PhysicalHuman* human,GsString file);
	~BalanceModule(void);
	void applyParameters();

	void init();
	void activate();
	bool evaluate();
			GsString stateString();
private:
	void computeBalanceTorques();
	GsVec getVirtualForce();
	GsVec forceContribution(PhysicalJoint* start, PhysicalJoint* end, GsVec pos);
	GsVec compTorque(PhysicalJoint* j, GsVec force);
	Line* vf_line;
	GsArray<Line*> vf_torque_lines;
	SnGroup* torque_lines;
        void computeSpineTorques(GsVec force, PhysicalJoint* upperBack);
        void computeLegTorques(GsVec force, PhysicalJoint* foot);



};
