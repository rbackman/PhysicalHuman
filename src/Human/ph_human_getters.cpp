#include "ph_joint.h"
#include "ph_human.h"
#include "ph_mod_root.h"
#include "ph_mod_com.h"
#include "ph_mod_ref.h"
#include "ph_mod_ik.h"
#include "ph_mod_contact.h"
#include "ph_manager.h"
#include "ph_state_manager.h"
#include "ph_motion_manager.h"

Module* PhysicalHuman::getModule(int i)
{
		if(i>_modules.size()-1)
		{
			phout<<"human has no controller "<<i<<gsnl;
			return 0;
		}
	return _modules.get(i);
}
Module* PhysicalHuman::getModule(const GsString& cont)
{
	for(int i=0;i<_modules.size();i++)
	{
		if(_modules.get(i)->name()==cont)
		{
			return _modules.get(i);
		}
	}
	_manager->message("couldn't find module ",cont);
	return 0;
}
GsArray<Serializable*> PhysicalHuman::getSerializables()
{
	GsArray<Serializable*> savs;
	for(int i=0;i<_joints.size();i++)
		savs.push(_joints.get(i));
	for(int i=0;i<_modules.size();i++)
		savs.push(_modules.get(i)->getSerializables());
	savs.push(this);
	savs.push(_motion_manager);
	savs.push(_state_manager);
	return savs;
}

KnSkeleton* PhysicalHuman::skref()
{
	return reference_module()->skref;
}
bool PhysicalHuman::staticBalanced()
{
	if(doubleSupported())
		return contact_module()->bothFeetContain(getCOMProjection());
	else
		return contact_module()->stanceFootContains(getCOMProjection());
}
PhysicalJoint* PhysicalHuman::leftToe(){return _left_toe_joint;}
PhysicalJoint* PhysicalHuman::rightToe(){return _right_toe_joint;}
PhysicalJoint* PhysicalHuman::leftFoot(){return _left_foot_joint;}
PhysicalJoint* PhysicalHuman::rightFoot(){return _right_foot_joint;}
PhysicalJoint* PhysicalHuman::stanceFoot(){return leftStance() ? leftFoot() : rightFoot();}
PhysicalJoint* PhysicalHuman::swingFoot(){return leftStance() ? rightFoot() : leftFoot();}
bool PhysicalHuman::stanceFootContact()
{
	if(getStanceState()== STANCE_LEFT)
		return contact_module()->leftFootContact();
	else 
		return contact_module()->rightFootContact();
}
PhysicalJoint* PhysicalHuman::swingKnee(){return swingFoot()->getParent();}
PhysicalJoint* PhysicalHuman::stanceKnee(){return stanceFoot()->getParent();}
PhysicalJoint* PhysicalHuman::swingHip(){return swingKnee()->getParent();}
PhysicalJoint* PhysicalHuman::stanceHip(){return stanceKnee()->getParent();}
PhysicalJoint* PhysicalHuman::root(){return _root_joint;}
PhysicalJoint* PhysicalHuman::hips(){return root();}
PhysicalJoint* PhysicalHuman::shoulder(){return _shoulder_joint;}
PhysicalJoint* PhysicalHuman::leftHand(){return _left_hand_joint;}
PhysicalJoint* PhysicalHuman::rightHand(){return _right_hand_joint;}
float PhysicalHuman::getStanceSwingRatio()
{
	if(doubleSupported())
		return contact_module()->pFloat(contact_stance_swing_ratio);
	else
		return 1.0f;
}


float getLength(KnJoint* j)
{
	if(j->child(0))
	{
		GsVec dis = j->child(0)->gcenter()-j->gcenter();
		return dis.len();
	}
	return 0;
}

GsQuat PhysicalHuman::getHeading()
{
	return decomposeRotation( hips()->getGlobalOrientation().conjugate(),GsVec(0,1,0)).conjugate();
}
bool PhysicalHuman::flying(){ return contact_module()->flying();}
float PhysicalHuman::computeMass()
{
	float mass=0;
	for(int i=0; i<_joints.size(); i++)
	{
		mass += _joints.get(i)->getBody()->getMass();
	}
	return mass;
}
PhysicalJoint* PhysicalHuman::joint(int idx)
{ 
	return _joints.get(idx);
}
PhysicalJoint* PhysicalHuman::joint(GsString name)
{
	for(int i=0;i<_joints.size();i++)
	{
		if(GsString::compare(GsString(_joints.get(i)->name()) ,name)==0)
		{
			return _joints.get(i);
		}
	}
	
	return 0;
}
