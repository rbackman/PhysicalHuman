#include "ph_mod_balance.h"
#include "util.h"
#include "ph_manager.h"
#include "ph_mod_contact.h"

BalanceModule::BalanceModule(PhysicalHuman* human,GsString file ):Module(human,"BalanceModule",file)
{

	CHECK_VEC(balance_jcom_gain_p);
	CHECK_VEC(balance_jcom_gain_d);
	CHECK_VEC(balance_jcom_velocity_desired);

	MAKE_TEMP_PARM(balance_linear_offset,GsVec());
	CHECK_BOOL(balance_jcom_virtual_force);
	CHECK_FLOAT(balance_linear_p);
	CHECK_FLOAT(balance_linear_d);
	CHECK_FLOAT(balance_linear_ankle_scale);
	CHECK_BOOL(balance_linear);
	CHECK_FLOAT(balance_linear_max_offset);
	CHECK_FLOAT(balance_linear_max_speed);
	CHECK_VEC(balance_vf_max);
	MAKE_TEMP_PARM(balance_d,GsVec());
	MAKE_TEMP_PARM(balance_v,GsVec());
	CHECK_VEC(balance_simbicon_gain_d);
	CHECK_VEC(balance_simbicon_gain_v);
	CHECK_BOOL(balance_simbicon_active);
	CHECK_FLOAT(balance_simbicon_root_scale);
	verifyParameters();

	vf_line = new Line(GsVec(),GsVec());
	vf_line->setColor(GsColor::red);
	grp->add(vf_line->getGrp());
	torque_lines = new SnGroup;
	grp->add(torque_lines);
}

BalanceModule::~BalanceModule(void)
{

}

void BalanceModule::applyParameters()
{
	Module::applyParameters();
	vf_line->visible(pBool(balance_jcom_virtual_force));
}
void BalanceModule::init()
{
	applyParameters();
	
}

void BalanceModule::activate()
{
	Module::activate();
	
	//determine next step
}
bool BalanceModule::evaluate()
{
	if(h->flying())
		return false;

	if(manager->animationStep())
	{
		for(int i=0;i<vf_torque_lines.size();i++)
		{
			delete vf_torque_lines.get(i);
		}
		vf_torque_lines.remove(0,vf_torque_lines.size());
		torque_lines->remove_all();
	}


		if( pBool(balance_jcom_virtual_force))
		{
			//if(!h->flying() && h->isStanding())
				computeBalanceTorques();
		}

		GsVec d = h->getHeading().inverse().apply( h->contact_module()->stancePoint() -  h->getCOM()) ;
		GsVec v = h->getHeading().inverse().apply( h->getCOMVelocity());
		setP(balance_d,d);
		setP(balance_v,v);
		
		if(pBool(balance_simbicon_active))
		{
			h->setP(human_double_support,false);
			
			
			GsVec sim_gain_d =  pVec(balance_simbicon_gain_d);
			GsVec sim_gain_v = pVec(balance_simbicon_gain_v);

			float ax = sim_gain_d.x*d.z + sim_gain_v.x*v.z;


			float az = sim_gain_d.z*d.x + sim_gain_v.z*v.x;
			
			
			h->root()->setP(joint_rot_offset,GsVec(pFloat(balance_simbicon_root_scale)*ax,0.0f,-pFloat(balance_simbicon_root_scale)*az));
			h->stanceHip()->setP(joint_rot_offset,GsVec());
			h->swingHip()->setP(joint_rot_offset,GsVec(ax,0.0f,az));
			h->stanceHip()->setP(joint_char_frame,false);
			
			//h->stanceHip()->setP(joint_use_pd,false);
			h->swingHip()->setP(joint_char_frame,true);
			
			//h->swingHip()->setP(joint_use_pd,true);
		}
		else if(pBool(balance_linear))
		{
			h->setP(human_double_support,true);

			/*
			GsVec d = h->getDesiredCOM() - h->getCOP();
			d.y=0;
			GsVec v = h->getCOMVelocity();
			if(d.len()>pFloat(balance_linear_max_offset))
			{
				phout<<"to far\n";
				d.set(0,0,0);// d.len(pFloat(balance_linear_max_offset));
				v.set(0,0,0);
			}
			else
			{
				if(v.len()>pFloat(balance_linear_max_speed))
				{
					phout<<"to fast\n";
					v.set(0,0,0);
					v.len(pFloat(balance_linear_max_speed));
				}
			}
			float ankScale = -pFloat(balance_linear_ankle_scale);
			GsVec gp = -pVec(balance_linear_p);
			GsVec gd = pVec(balance_linear_d);


			GsVec newOff = GsVec(gp.x*d.z + gd.x*v.z,0.0f, gp.z*d.x + gd.z*v.x);
			*/

			if(!h->flying() && h->isStanding())
			{
				GsVec d = h->getHeading().inverse().apply( h->getDesiredCOM() -  h->getCOM());
				GsVec v = h->getHeading().inverse().apply(  h->getCOMVelocity());

				float ankScale = - pFloat(balance_linear_ankle_scale);
				GsVec lin_gain_p = - pVec(balance_linear_p);
				GsVec lin_gain_d = pVec(balance_linear_d);


				GsVec newOff = GsVec(lin_gain_p.x*d.z + lin_gain_d.x*v.z,0.0f, lin_gain_p.z*d.x + lin_gain_d.z*v.x);

				if(newOff.len()>pFloat(balance_linear_max_offset))
					newOff.len(pFloat(balance_linear_max_offset));
					
				setP(balance_linear_offset,newOff);
			}
			else
			{
				setP(balance_linear_offset,GsVec());
			}

			h->stanceHip()->setP(joint_rot_offset,pVec(balance_linear_offset));
			h->stanceFoot()->setP(joint_rot_offset,pVec(balance_linear_offset)*pFloat(balance_linear_ankle_scale));

			if( h->doubleSupported())
			{
					h->swingHip()->setP(joint_rot_offset,pVec(balance_linear_offset));
				
					h->swingFoot()->setP(joint_rot_offset,pVec(balance_linear_offset)*pFloat(balance_linear_ankle_scale));
			}
	
		}
	
	
		
	

	

		
	return Module::evaluate();
}

GsVec BalanceModule::getVirtualForce()
{
	
	GsVec d = h->getCOM()- h->getDesiredCOM();
	GsVec v =h->getCOMVelocity() - pVec(balance_jcom_velocity_desired);
	
	GsVec desA;
	double comOffsetCoronal = 0;

	GsVec VF = vecMult(d,pVec(balance_jcom_gain_p)) - vecMult(v,pVec(balance_jcom_gain_d));

	//phout<<VF<<gsnl;
	//and this is the force that would achieve that - make sure it's not too large...
	VF = -VF * h->getMass(); 

	VF = h->getHeading().inverse().apply(VF);

	//phout<<"virtual force = "<<fA<<gsnl;
	VF = boundToRange(VF,-pVec(balance_vf_max),pVec(balance_vf_max));
	VF.y = 0;

	return VF;

}


GsVec BalanceModule::forceContribution(PhysicalJoint* start, PhysicalJoint* end, GsVec pos)
{
	GsVec f;
	PhysicalJoint* currentJoint = start->getParent();
	bool looking = true;
	while(looking)
	{
		f += (currentJoint->getJointPosition() - pos)*currentJoint->getBody()->getMass();
		if(currentJoint->getParent() == 0 || currentJoint->getParent() == end)looking = false;
		else currentJoint = currentJoint->getParent();
	}
	f /= h->getMass();
	return f;
}

GsVec BalanceModule::compTorque(PhysicalJoint* j, GsVec force)
{
	GsVec t;
	
	t = forceContribution(j,0,j->getCOMPosition());
	t = cross(t,force);
	t = vecMult(t,j->getGlobalOrientation().apply(j->pVec(joint_vf_scale))); 
		
	if(manager->animationStep())
	{
		if(pBool(module_visible))
		{
			vf_torque_lines.push(new Line(j->getJointPosition(),j->getJointPosition()+t*0.01f));
			vf_torque_lines.top()->setColor(GsColor::blue);
		}
	}
	j->addGlobalTorque(t);

	return t;

}


void BalanceModule::computeBalanceTorques(){
//applying a force at the COM induces the force f. The equivalent torques are given by the J' * f, where J' is
// dp/dq, where p is the COM.
	
	GsVec force = getVirtualForce();
	if(manager->animationStep())
		vf_line->setPoints(h->getCOM(),h->getCOM()+h->getHeading().apply(force)*0.001f);
	//phout<<force<<gsnl;

	PhysicalJoint* j;
			
	j = h->leftFoot();
	compTorque(j,force);

	
	j = j->getParent(); //knee
	compTorque(j,force);

	j = j->getParent(); //hip
	compTorque(j,force);
	
	j = h->rightFoot();
	compTorque(j,force);


	j = j->getParent(); //knee
	compTorque(j,force);


	j = j->getParent(); //hip
	compTorque(j,force);


	j = h->shoulder()->getParent();
	compTorque(j,force);

	j = h->shoulder();
	compTorque(j,force);
	if(manager->animationStep())
	{
		for(int i=0;i<vf_torque_lines.size();i++)
		{
			torque_lines->add(vf_torque_lines.get(i)->getGrp());;
		}
	}
}

GsString BalanceModule::stateString()
{
	
		GsString f = "BalanceModule\n";
		f<<"{\n";
		f<< parameterAsString(balance_jcom_velocity_desired);
		f<<parameterAsString(balance_simbicon_active);
		f<<parameterAsString(balance_linear);
		f<<parameterAsString(balance_simbicon_gain_d);
		f<<parameterAsString(balance_simbicon_gain_v);
		f<<"}\n";



		return f;
	
}





