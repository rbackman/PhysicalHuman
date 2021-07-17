#include "ph_mod_virtual.h"
#include "util_models.h"
#include "ph_manip_virtual.h"

VirtualModule::VirtualModule(PhysicalHuman* human,GsString file ):ManipulatorModule(human,"VirtualModule",file)
{
	
	CHECK_FLOAT(virtual_strength);
	CHECK_BOOL(virtual_align_feet);
	CHECK_FLOAT(virtual_align_feet_gain);

	_shortName = "virtual";
	verifyParameters();


}

VirtualModule::~VirtualModule(void)
{

}

void VirtualModule::applyParameters()
{
	Module::applyParameters();
	
}




HumanManipulator* VirtualModule::makeManip(GsString jointName,GsString file)
{
	PhysicalJoint* j = h->joint(jointName);
	if(j)
	{
		return (HumanManipulator*)(new VirtualSpring(h,this,jointName,file));
	}
	return 0;
}
bool VirtualModule::virtualForce(GsVec force,GsVec pos,PhysicalJoint* effector,PhysicalJoint* base)
{
		PhysicalJoint* currentJoint = effector;
		GsVec tmpV;
	
	while (currentJoint != base){
		if (currentJoint == 0)
			phout<<"VirtualSpring::evaluate() --> end was not a parent of start...\n";

		tmpV = currentJoint->getCOMPosition() - pos;

		GsVec tmpT = cross(tmpV,force);
		currentJoint->addGlobalTorque(tmpT);
		currentJoint = currentJoint->getParent();
	}

	//and we just have to do it once more for the end joint, if it's not NULL
	if (base != 0)
	{
		tmpV = currentJoint->getCOMPosition() - pos;
		GsVec tmpT = cross(tmpV,force);
		currentJoint->addGlobalTorque(tmpT);
	}
	return true;
}
bool VirtualModule::evaluate()
{
	if(! h->isStanding())return false;
	if (pBool(manip_module_match))
	{
		matchToSkeleton();
	}
	if(pBool(virtual_align_feet))
	{
		GsVec stanceGoal = (h->leftFoot()->getCOMPosition()+h->rightFoot()->getCOMPosition())/2.0f;
		GsVec vel = h->getCOMVelocity();
		GsVec lforce = pFloat(virtual_align_feet_gain,0)*(h->leftFoot()->getCOMPosition() - stanceGoal)- pFloat(virtual_align_feet_gain,1)*(h->leftFoot()->getVelocity()-vel);
		lforce.x=0;
		lforce.y=0;
		GsVec rforce = pFloat(virtual_align_feet_gain,0)*(h->rightFoot()->getCOMPosition() - stanceGoal)- pFloat(virtual_align_feet_gain,1)*(h->rightFoot()->getVelocity()-vel);
		rforce.x=0;
		rforce.y=0;
		
		virtualForce(-lforce,h->leftFoot()->getCOMPosition(),h->leftFoot(),h->leftFoot()->getParent()->getParent());
		virtualForce(-rforce,h->rightFoot()->getCOMPosition(),h->rightFoot(),h->rightFoot()->getParent()->getParent());
	}
	return ManipulatorModule::evaluate();
}




