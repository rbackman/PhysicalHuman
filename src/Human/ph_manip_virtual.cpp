#include "ph_manip_virtual.h"
#include "ph_mod_virtual.h"

VirtualSpring::VirtualSpring(PhysicalHuman* human,VirtualModule* cont,GsString nme,GsString file) : HumanManipulator(cont,human->joint(nme),"VirtualManip",file)
{


	CHECK_STRING(virtual_effector_joint);
	CHECK_STRING(virtual_base_joint);
	CHECK_FLOAT(virtual_spring_k);
	CHECK_FLOAT(virtual_spring_c);
	CHECK_BOOL(virtual_spring_use_orientation);
	CHECK_FLOAT(virtual_spring_kr);
	CHECK_FLOAT(virtual_spring_cr);
	CHECK_FLOAT(virtual_spring_rest_length);
	CHECK_FLOAT(virtual_spring_break_length);

	MAKE_TEMP_PARM(virtual_spring_p1,GsVec());
	MAKE_TEMP_PARM(virtual_spring_p2,GsVec());
	MAKE_TEMP_PARM(virtual_spring_force,GsVec());
	MAKE_TEMP_PARM(virtual_spring_torque,GsVec());
	MAKE_TEMP_PARM(virtual_spring_last_extent,GsVec());

	verifyParameters();

	
	line = new Line(pVec(virtual_spring_p1),pVec(virtual_spring_p2));


	p1Model = new Ball(pVec(virtual_spring_p1),0.01f);
	p2Model= new Ball(globalPosition()+pVec(virtual_spring_p2),0.01f);
	
	line->setColor(GsColor::green);
	grp->add(line->getGrp());

	grp->add(p1Model->getGrp());
	grp->add(p2Model->getGrp());

	
	
	effector = joint;
	base = human->joint(pString(virtual_base_joint));

}

void VirtualSpring::init()
{

}
void VirtualSpring::activate()
{

}
bool VirtualSpring::evaluate()
{

	GsVec pa =  effector->getCOMPosition()+ effector->getGlobalOrientation().apply(pVec(virtual_spring_p1));
	GsVec pb = globalPosition()+ pQuat(manipulator_orientation).apply(pVec(virtual_spring_p2));
	
	p1Model->setPosition(pa);
	p2Model->setPosition(pb);
	line->setPoints(pa,pb);
	/*
	this works starting from the effector going up to the joint specified but in a case
	like the foot I want it to go up the stance leg to the com
	*/
	
	if(pBool(manipulator_active))
	{
		PhysicalJoint* currentJoint = effector;
		GsVec tmpV;
		GsVec pGlobal = pa;
		GsVec ext = pb-pa;

		GsVec force = (ext)*pFloat(virtual_spring_k) + (pVec(virtual_spring_last_extent) - ext)*pFloat(virtual_spring_c);
		
		setP(virtual_spring_last_extent,ext);

		while (currentJoint != base){
			if (currentJoint == 0)
				phout<<"VirtualSpring::evaluate() --> end was not a parent of start...\n";
		
			tmpV = currentJoint->getCOMPosition() - pGlobal;

			GsVec tmpT = cross(tmpV,force);
			currentJoint->addGlobalTorque(tmpT);
			currentJoint = currentJoint->getParent();
		}
	
		//and we just have to do it once more for the end joint, if it's not NULL
		if (base != 0)
		{
			tmpV = currentJoint->getCOMPosition() - pGlobal;
			GsVec tmpT = cross(tmpV,force);
			currentJoint->addGlobalTorque(tmpT);
		}
		
	}
	if(pBool(virtual_spring_use_orientation))
		{
			GsQuat qcur = effector->getGlobalOrientation();
			GsQuat qdes = globalOrientation();
			GsVec t = computePDTorque(effector->getHuman()->getWorld()->getSimStep(),qcur,qdes,effector->getRotationalVelocity(),GsVec(),pFloat(virtual_spring_kr),pFloat(virtual_spring_cr));
			effector->addGlobalTorque(t);
		}
	return true;
}



