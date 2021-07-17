//PhysicalHuman.cpp

#include "ph_human.h"
#include "ph_joint.h"


#include "ph_mod_gravity.h"
#include "ph_mod_root.h"
#include "ph_mod_com.h"
#include "ph_mod_ref.h"
#include "ph_mod_ik.h"
#include "ph_mod_contact.h"
#include "ph_mod_puppet.h"
#include "ph_mod_balance.h"
#include "ph_mod_virtual.h"
#include "ph_mod.h"
#include "util_curve.h"
#include "util_models.h"

#include "ph_file_manager.h"
#include "ph_manager.h"

void PhysicalHuman::applyParameters()
{
	_vis_scene->set_visibility(pBool(human_show_skeleton),pBool(human_show_visual_geo),0,pBool(human_show_axis));
	
	float swingGainDMult = getGainMultSwing() * getGainDMult();
	float swingGainPMult = getGainMultSwing() * getGainPMult();

	float stanceGainDMult =  getGainMultStance()* getGainDMult();
	float stanceGainPMult =  getGainMultStance() * getGainPMult();
//	phout<<"swing gain p" <<swingGainPMult;
//	phout<<"  stance gain p"<<stanceGainPMult<<gsnl;

	swingFoot()->setGaingMult(swingGainPMult,swingGainDMult);
	swingKnee()->setGaingMult(swingGainPMult,swingGainDMult);
	swingHip()->setGaingMult(swingGainPMult,swingGainDMult);
	stanceFoot()->setGaingMult(stanceGainPMult,stanceGainDMult);
	stanceKnee()->setGaingMult(stanceGainPMult,stanceGainDMult);
	stanceHip()->setGaingMult(stanceGainPMult,stanceGainDMult);

	_motionSupportLine->visible(pBool(human_show_support_vec));
	if(pBool(human_collide_feet_only))
	{
		for(int i=0;i<numJoints();i++)
		{
			joint(i)->getBody()->collides(false);
		}
		leftFoot()->getBody()->collides(true);
		rightFoot()->getBody()->collides(true);
		joint("LeftToeBase")->getBody()->collides(true);
		joint("RightToeBase")->getBody()->collides(true);
	}
	else
	{
		for(int i=0;i<numJoints();i++)
		{
			joint(i)->getBody()->collides(true);
		}
	}
	for(int i=0;i<numJoints();i++)
	{
		joint(i)->getGroup()->visible(pBool(human_show_collision_geo));
	}
	_vis_scene->update();
	_heading_arrow->visible(pBool(human_show_heading));
	_desired_heading_arrow->visible(pBool(human_show_heading));
	_module_group->visible(pBool(human_show_controllers));
}

GsString PhysicalHuman::stateString()
{
	GsString f = "Human\n";
	f<<"{\n";
	f<< parameterAsString(human_stance_state);

	f<<parameterAsString(human_manual_com);
	f<<parameterAsString(human_gain_mult);
	f<<"}\n";



	return f;
}
GsString PhysicalHuman::jointStateString()
{
	GsString str = "#state file for ";
	str<< name() <<"  manips.. they are in a separate file since they have the same names as the joints\n";
	for(int i=0;i<numJoints();i++)
	{
		PhysicalJoint* j = joint(i);
		if(j)
		{
			str<<j->stateString();
		}
	}
		
return str;
}
GsVec PhysicalHuman::getSupportVecFromIK()
{
	GsVec stanceP;
	if(leftStance())
		stanceP = ik_module()->leftFootIKPos();
	else
		stanceP = ik_module()->rightFootIKPos();

	GsVec hipsP = ik_module()->getRootManipPos();
	_motionSupportVec =  hipsP-stanceP;
	_motionSupportLine->setPoints(stanceP,hipsP);
	return _motionSupportVec;
}
GsVec PhysicalHuman::getCOMVelocity()
{
	return com_module()->getCOMVelocity();
}
GsVec PhysicalHuman::getCOP()
{
	return contact_module()->getCOP();
}
GsVec PhysicalHuman::getCOM(){return com_module()->getCOM();}
GsVec PhysicalHuman::getDesiredCOM(){
	//return com_controller()->getDesiredCOM();
	if(pBool(human_manual_com))
		return contact_module()->stancePoint();

	return com_module()->getDesiredCOMFromSupportVec();
}
GsVec PhysicalHuman::getCOMProjection(){return com_module()->getCOMProjection();}
void PhysicalHuman::update()
{
	
	if(pBool(human_manual_com))
	{
		com_module()->setDesiredCOM(contact_module()->stancePoint());
	}
	else
	{
		//com_controller()->setP(com_desired_support_vector,getHeading().apply(getSupportVecFromIK()));
	}
	
	
	if(_manager->animationStep())
	{
		if(_heading_arrow->visible())
		{
			GsVec com = com_module()->getCOMProjection();
			_heading_arrow->setPosition(com);
			_heading_arrow->setRotation(getHeading());
			com.y-=0.01f;

			_desired_heading_arrow->setPosition(com);
			_desired_heading_arrow->setRotation(getDesiredHeading());
		}
	}
	
	
	for(int i=0;i<_modules.size();i++)
	{
		if(_modules.get(i)->isActive())
		{
			_modules.get(i)->evaluate();
		}
	}


	for(int i=0;i<_joints.size();i++)
	{
		_joints.get(i)->update();
	}

	for (int i=0;i<_external_forces.size();i++)
	{
		_external_forces[i].body->addForce(_external_forces[i].body->getPosition(),_external_forces[i].f);
		_external_forces[i].time_left-= getWorld()->getSimStep();
		if(_external_forces[i].time_left<=0)
		{
			_external_forces.remove(i);
		}
	}
}

bool PhysicalHuman::isStanding()
{
	return(hips()->getCOMPosition().y - getStancePosition().y> 0.1f);
}
bool PhysicalHuman::nonFootContact()
{
	return getManager()->getWorld()->nonFootContact();
}

GsString PhysicalHuman::characterName()
{
	return pString(human_character_name);
}

void PhysicalHuman::redraw()
{
	reference_module()->_knsref->update();
	com_module()->setP(com_desired_support_vector,getDesiredHeading().apply(getSupportVecFromIK()));
	if(_vis_skel)
	{
		for(int i=0;i<_joints.size();i++)
		{	
			_vis_skel->root()->pos()->value(hips()->getJointPosition());
			KnJoint* j = _vis_skel->joint(_joints.get(i)->name());
			if(j)
				j->rot()->value(_joints.get(i)->getRelativeOrientation());
			_vis_scene->update();
		}
	}
}

HumanManager* PhysicalHuman::getManager()
{
	return _manager;
}

void PhysicalHuman::setGainMult( float p, float d,float swp,float swd )
{
	setP(human_gain_mult,p,0);
	setP(human_gain_mult,d,1);
	if(swp<0)swp=0;
	if(swp>1)swp=1;
	if(swd<0)swd=0;
	if(swd>1)swd=1;
	setP(human_gain_mult,swp,2);
	setP(human_gain_mult,swd,3);
	applyParameters();
}
float PhysicalHuman::getGainPMult()
{
	return pFloat(human_gain_mult,0);
}

float PhysicalHuman::getGainDMult()
{
	return pFloat(human_gain_mult,1);
}
float PhysicalHuman::getGainMultStance()
{
	return pFloat(human_gain_mult,2);
}
float PhysicalHuman::getGainMultSwing()
{
	return pFloat(human_gain_mult,3);
}

void PhysicalHuman::reset()
{
	setP(human_desired_heading_delta,0.0f);
	balance_module()->setP(balance_jcom_velocity_desired,GsVec());
	for(int i=0;i<_joints.size();i++)
	{
		_joints[i]->setP(joint_rot_offset,GsVec());
		_joints[i]->getBody()->reset();
	}
}

void PhysicalHuman::addForce( GsVec dir, float dur ,ODEObject* body)
{
	body_force f;
	f.f = dir;
	f.time_left = dur;
	f.body = body;
	_external_forces.push(f);
	GsString mes = "added force : ";
	mes << dir.x<<" "<<dir.y<<" "<<dir.z << " duration:"<<dur;
	_manager->message(mes);
}



GsQuat PhysicalHuman::getDesiredHeading()
{
	GsQuat dHeading = GsQuat(); //getHeading();
	dHeading = dHeading*vecToQuat(GsVec(0.0f,pFloat(human_desired_heading_delta),0.0f), gsXYZ) ;
	dHeading = dHeading* vecToQuat(root()->pVec(joint_rot_offset),rotation_type_to_gs_euler_order(root()->rotationOrder()));
	
	return decomposeRotation( dHeading.conjugate(),GsVec(0,1,0)).conjugate();
}

GsString PhysicalHuman::moduleString()
{
	GsString fo;
	for(int i=0;i<numModules();i++)
	{
		fo<<getModule(i)->toString();
	}
	return fo;
}

GsString PhysicalHuman::jointString()
{
	GsString fo;
	for(int i=0;i<numJoints();i++)
	{
		fo<<joint(i)->toString();
	}
	if(_default_joint)
		fo<<_default_joint->toString();
	return fo;
}

GsVec PhysicalHuman::getStancePosition()
{
	return stanceFoot()->getCOMPosition();
}

float PhysicalHuman::getHeadingAngle()
{
	return angle( GsVec(1,0,0), getHeading().apply(GsVec(0,0,1)) );
}

float PhysicalHuman::totalTorque()
{
	float t = 0;
	for(int i=0;i<numJoints();i++)
	{
		t+=joint(i)->getLastTorque().len();
	}
	return t;
}

GsVec PhysicalHuman::stancePoint()
{
	return contact_module()->stancePoint();
}

float PhysicalHuman::floorHeight()
{
	return contact_module()->pFloat(contact_floor_height);
}

GsVec PhysicalHuman::getContactProjectionCenter()
{
	return contact_module()->getContactProjectionCenter();
}

bool PhysicalHuman::feetPlanted()
{
	return contact_module()->bothFeetPlanted();
}






void computeJointTorquesEquivalentToForce(PhysicalJoint* start, GsVec pGlobal, GsVec fGlobal, PhysicalJoint* end)
{
	//starting from the start joint, going towards the end joint, get the origin of each link, in world coordinates,
	//and compute the vector to the global coordinates of pLocal.

	PhysicalJoint* currentJoint = start;
	GsVec tmpV;
	
	while (currentJoint != end)
	{
		if (currentJoint == 0)
			phout<<"BalanceModule::computeJointTorquesEquivalentToForce --> end was not a parent of start...\n";
		tmpV = pGlobal - currentJoint->getJointPosition();
		GsVec tmpT = cross(tmpV,fGlobal);
		currentJoint->addGlobalTorque(-tmpT);
		currentJoint = currentJoint->getParent();
	}
	
	//and we just have to do it once more for the end joint, if it's not NULL
	if (end != 0){
		tmpV = pGlobal - currentJoint->getJointPosition();
		GsVec tmpT = cross(tmpV,fGlobal);
		currentJoint->addGlobalTorque(-tmpT);
	}
}


human_stance toggleStance( human_stance s )
{
	if(s==STANCE_LEFT)
		return STANCE_RIGHT;
	else
		return STANCE_LEFT;
}

