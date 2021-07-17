#include "ph_manip.h"
#include "ph_mod_manip.h"

HumanManipulator::HumanManipulator(ManipulatorModule* cont, GsString name,GsString typ,GsString file):Manipulator(name,typ,file)
{
	controller = cont;
	manager = controller->getManager();
	joint = 0;
	kn_joint = 0;
}
HumanManipulator::HumanManipulator(ManipulatorModule* cont,PhysicalJoint* j,GsString typ,GsString file):Manipulator(j->name(),typ,file)
{
	controller = cont;
		manager = controller->getManager();
	joint = j;
	kn_joint = 0;
	getQuatParameter(manipulator_orientation)->rotationOrder = j->getQuatParameter(joint_desired_rot)->rotationOrder;
	setInitial();
}
HumanManipulator::HumanManipulator(ManipulatorModule* cont,KnJoint* j,GsString typ, GsString file):Manipulator(GsString(j->name()),typ,file)
{
	controller = cont;
		manager = controller->getManager();
	kn_joint = j;
	joint = 0;
	rotation_type rotationO = joint_euler_type_to_rotation_type(j->euler()->type());
	getQuatParameter(manipulator_orientation)->rotationOrder =  rotationO;
	setInitial();
}
void HumanManipulator::match(KnJoint* j)
{
	GsQuat qJt;
	mat2quat(j->gmat(),qJt);
	GsVec jp = (j->gcenter() - start_position)+(start_orientation*qJt).apply(pVec(manipulator_offset));
	translation(jp);
	rotation(start_orientation*qJt);
}
void HumanManipulator::match(PhysicalJoint* j)
{
	translation((j->getJointPosition() - start_position)+(start_orientation*j->getGlobalOrientation()).apply(pVec(manipulator_offset)));
	rotation(start_orientation*j->getGlobalOrientation());
}
void HumanManipulator::match()
{
	if(kn_joint)
	{
		match(kn_joint);
	}
	else if (joint)
	{
		match(joint);
	}
}

void HumanManipulator::setInitial()
{
	
	if(joint)
	{
		start_position = joint->getJointPosition();
		start_orientation = joint->getGlobalOrientation();
	}
	else if (kn_joint)
	{
		start_position = kn_joint->gcenter();
		mat2quat(kn_joint->gmat(),start_orientation);
	}
	else
	{
		start_position.set(0,0,0);
		start_orientation.set(1,0,0,0);
	}
	GsMat m;
	compose(start_orientation,start_position,m);
	initial_mat(m);

}
