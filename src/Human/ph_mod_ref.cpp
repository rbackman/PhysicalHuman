
#include "ph_mod_ref.h"

#include "ph_manager.h"
#include "ph_file_manager.h"

ReferenceModule::ReferenceModule(PhysicalHuman* human,const char* file) : Module(human,"ReferenceModule",file)
{
		
	CHECK_VEC(reference_skeleton_position);
	CHECK_BOOL(reference_skeleton_visible);

	verifyParameters();


	skref = new KnSkeleton;
	GsString skelName = manager->getFiles()->getKinematicsFile(human->pString(human_reference_skeleton_name));
			
	bool ok = skref->load(skelName);
	if ( !ok ) phout.fatal("Could not load reference skeleton file!");
	skref->init_values();
	skref->root()->pos()->value(pVec(reference_skeleton_position));
	skref->update_global_matrices();

	_knsref = new KnScene;
	grp->add(_knsref);
	_knsref->connect(skref);
	_knsref->set_skeleton_radius(0.5f);
	_knsref->update();
	grp->add(_knsref);

	applyParameters();
		
}
void ReferenceModule::applyParameters()
{
	Module::applyParameters();
	_knsref->set_visibility(pBool(reference_skeleton_visible),0,0,0);
	skref->root()->pos()->value(pVec(reference_skeleton_position));
	_knsref->update();
}

void ReferenceModule::init()
{
	
	for(int i=0;i<pids.size();i++)
	{
		delete pids.get(i);
	}
	pids.remove(0,pids.size());

	for(int i=0;i<h->numJoints();i++)
	{
		PhysicalJoint* j = h->joint(i);

		if(j->pBool(joint_use_pd))
		{
			if(h->skref())
			{
				KnJoint* knJoint = h->skref()->joint(j->name());
				if(!knJoint)
				{
					phout<<"joint "<<j->name()<<" isnt on ref skel\n";
				}
				else
				{
					pids.push(new PID( j,knJoint ));
				}
			}
		}
	}
}
void ReferenceModule::removePIDS()
{
	pids.remove(0,pids.size());
}

ReferenceModule::~ReferenceModule()
{
	for(int i=0;i<pids.size();i++)
		delete pids.get(i);
	pids.remove(0,pids.size());
}
PID* ReferenceModule::pid(const char* name)
{
	for(int i=0;i<pids.size();i++)
	{
		if(GsString::compare(name,pids.get(i)->getJoint()->name())==0)
			return pids.get(i);
	}
	return 0;
}

		

bool ReferenceModule::evaluate()
{
	for(int i=0;i<pids.size();i++)
	{
		if(pids.get(i)->getJoint()->pBool(joint_use_pd))
		{
			pids.get(i)->evaluate();
		}	
	}

	/* bubble torques ??
	for (int i=h->numJoints()-1;i>=0;i--)
	{
		PhysicalJoint* j = h->joint(i);
		if (j != h->stanceHip() && j != h->stanceKnee())
			if (j->parent != h->hips() && j->parent!=0 )
				j->parent->addGlobalTorque(j->getTorque());
	}*/


//	_knsref->update();

	return Module::evaluate();
}
void ReferenceModule::applyToRefSkel(KnSkeleton* sk)
{
	for(int i=0;i< sk->joints().size();i++)
	{
		h->skref()->joints().get(i)->rot()->value(sk->joints().get(i)->rot()->value());
	}
	
	h->skref()->root()->pos()->value(sk->root()->pos()->value());
	h->skref()->update_global_matrices();
}

GsVec ReferenceModule::calculateSkeletonCom()
{
	GsVec ncom;
	float totalMass = 0;
	for (int i=0;i<h->numJoints();i++)
	{
		PhysicalJoint* pj = h->joint(i);
		if(pj)
		{
			KnJoint* knj =	skref->joint(pj->name());
			if(knj)
			{
				float mass = pj->getBody()->getMass();
				totalMass += mass;
				ncom += mass* GsVec(knj->gcenter()+knj->rot()->value().apply(pj->getBoxOffset()) ); 
			}
		}
	}
	ncom = ncom/(totalMass);
	//ncom = pFloat(com_sample_ratio)*getCOM() + (1.0f-pFloat(com_sample_ratio))*ncom;

	return ncom;
}


PID::PID(PhysicalJoint* jt,KnJoint* kjnt)
{
	joint=jt;
	knJoint = kjnt;	
}
PID::~PID(void)
{

}

bool PID::evaluate()
{

// 	if(joint->name()=="LeftUpLeg")
// 	{
// 		phout<<"des "<<joint->getDesiredRotationalVelocity()<<gsnl;
// 	}
	GsVec t = computePDTorque(
		joint->getWorld()->getSimStep(),
		joint->getGlobalOrientation(),
		joint->getDesiredOrientation(knJoint) ,
		joint->getRotationalVelocity(),
		joint->getDesiredRotationalVelocity(),
		joint->pFloat(joint_gain_p)*joint->pFloat(joint_gain_mult_p),
		joint->pFloat(joint_gain_d)*joint->pFloat(joint_gain_mult_d)
		);
	
	//t is in global coords
	//now should be in local coords
	//t = vecMult(t,joint->pVec(joint_pd_scale));

	joint->addGlobalTorque(t );

	//if(joint->parent)
	//	joint->parent->addGlobalTorque(-t);

	return true;
}


