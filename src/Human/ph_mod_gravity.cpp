#include "ph_mod_gravity.h"


GravityModule::GravityModule(PhysicalHuman* human,const char* file):Module(human,"GravityModule",file)
{
	CHECK_FLOAT(gravity_magnitude);
	verifyParameters();
}

GravityModule::~GravityModule(void)
{

}



bool GravityModule::evaluate()
{
bool valid = Module::evaluate();
	
	for (int i=0; i< h->numJoints(); i++)
	{
		PhysicalJoint* j = h->joint(i);
		if(j->pBool(joint_grav_comp))
			computeJointTorquesEquivalentToForce(h->joint(i), j->getJointPosition(), GsVec(0.0f, -pFloat(gravity_magnitude)*h->joint(i)->getBody()->getMass()*9.8f, 0.0f), NULL);
	}

return valid;
}


