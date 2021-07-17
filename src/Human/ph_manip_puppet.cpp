
#include "ph_manip_puppet.h"
#include "ph_joint.h"
#include "ode_spring.h"
SpringManip::SpringManip(ManipulatorModule* cont, PhysicalJoint* j,GsString file):HumanManipulator(cont,j,"MagicSpring",file)
{
	CHECK_FLOAT(magic_spring_k);
	CHECK_FLOAT(magic_spring_c);
	CHECK_BOOL(magic_spring_use_orientation);
	CHECK_FLOAT(magic_spring_kr);
	CHECK_FLOAT(magic_spring_cr);
	CHECK_FLOAT(magic_spring_rest_length);
	CHECK_FLOAT(magic_spring_break_length);
	CHECK_VEC(magic_spring_p1);
	CHECK_VEC(magic_spring_p2);
	CHECK_VEC(magic_spring_force);
	CHECK_VEC(magic_spring_torque);
	verifyParameters();

	spring = new ODESpring(j->getHuman()->getWorld(),j->getBody(),GsVec());
	applyParameters();
}
void SpringManip::applyParameters()
{
	Manipulator::applyParameters();
	spring->lin_k = pFloat(magic_spring_k);
	spring->lin_d = pFloat(magic_spring_c);
	spring->rot_k = pFloat(magic_spring_kr);
	spring->rot_d = pFloat(magic_spring_cr);
	spring->use_orientation = pBool(magic_spring_use_orientation);
	spring->active = pBool(manipulator_active);
	spring->break_length = pFloat(magic_spring_break_length);
	spring->rest_length = pFloat(magic_spring_rest_length);
}
bool SpringManip::evaluate()
{
	Manipulator::evaluate();
	spring->p2 = globalPosition();
	spring->orientation_desired = globalOrientation();
	spring->update();
	return true;
}