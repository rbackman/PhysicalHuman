
# include "ph_manip_ik.h"
#include "ph_mod_manip.h"
#include "ph_mod_ik.h"
#include "util_models.h"

IkManipulator::IkManipulator (IKModule* ikc,KnJoint* j, GsString file):HumanManipulator((ManipulatorModule*)ikc,j,"IKManip",file)
 {

	CHECK_FLOAT(ik_manip_orbit);
	CHECK_BOOL(ik_manip_root);
	CHECK_BOOL(ik_manip_show_lines);

	verifyParameters();

    _ik = new KnIk;
	_ik->solve_closest(true);
	_lines = new SnLines;
	KnIk::Type t ;
	if(kn_joint->name()=="LeftHand")
    {	t = KnIk::LeftArm; 	_ik->init(t,kn_joint);}
	else if(kn_joint->name()=="RightHand")
	{	t = KnIk::RightArm;_ik->init(t,kn_joint);}
	else if(kn_joint->name()=="LeftFoot")
	{	t = KnIk::LeftLeg;_ik->init(t,kn_joint);}
	else if(kn_joint->name()=="RightFoot")
	{	t = KnIk::RightLeg;_ik->init(t,kn_joint);}

 



	lines(true);
	grp->add ( _lines );

	applyParameters();
 }
void IkManipulator::applyParameters()
{
	Manipulator::applyParameters();
	lines(pBool(ik_manip_show_lines));

}

IkManipulator::~IkManipulator ()
 {
   if ( _ik ) _ik->unref();
 }


void IkManipulator::lines ( bool b )
 {
   if ( b )
    { _ik->lines ( _lines );
      update ();
    }
   else
    { _ik->lines ( 0 );
      _lines->init();
    }
 }

bool IkManipulator::evaluate ()
 {
	Manipulator::evaluate();
	if(!controller->isActive())
	{
		return false;
	}
	if(!pBool(manipulator_active))
	{
		//	match();
			return false;
	}
	

	 solve();
	 
	 return true;
 }

GsString IkManipulator::stateString()
{
	GsString ss = name();
	ss<<"{\n";
	ss<<parameterAsString(manipulator_position);
	ss<<parameterAsString(manipulator_orientation);
	ss<<parameterAsString(manipulator_active);
	ss<<parameterAsString(manipulator_visible);
	ss<<parameterAsString(ik_manip_orbit);
	ss<<"}\n";
	return ss;
}

void IkManipulator::solve()
{
	if(pBool(ik_manip_root))
	{
		kn_joint->pos()->value(globalPosition());
		kn_joint->rot()->value(globalOrientation());
		kn_joint->skeleton()->update_global_matrices();
	}
	else
	{
		GsMat m;
		compose(globalOrientation(),globalPosition(),m);
		_ik->set_local(m);
		KnIk::Result r = _ik->solve(m,GS_TORAD(pFloat(ik_manip_orbit)));

		_ik->apply_last_result();
		//setP(ik_manip_orbit,_ik->orbit_angle());
	}
}



