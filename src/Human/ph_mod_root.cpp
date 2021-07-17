#include "ph_mod_root.h"
#include "ph_human.h"
#include "ph_joint.h"
#include "ph_mod_contact.h"
#include "ph_manager.h"

RootModule::RootModule(PhysicalHuman* human,const char* file):Module(human,"RootModule",file)
{
	h = human;
	CHECK_FLOAT(root_gain_p);
	CHECK_FLOAT(root_gain_d);
	CHECK_FLOAT(root_max_ang);
	CHECK_FLOAT(root_stance_hip_damping);
	CHECK_FLOAT(root_stance_hip_max_velocity);
	CHECK_FLOAT(root_max_t);
	CHECK_FLOAT(root_strength);
	MAKE_TEMP_PARM(root_torque,GsVec());
}

RootModule::~RootModule(void)
{
}
GsString RootModule::stateString()
{
	GsString f = name();
	f<<"\n{\n";
	f<<parameterAsString(root_stance_hip_damping );
	f<<"}\n";
	return f;
}
bool RootModule::evaluate()
{
	bool valid = true;

	if(! h->stanceFootContact())return false;
	GsVec rootTorque;
	float rootStrength = pFloat(root_strength);

	if (rootStrength < 0)
		rootStrength = 0;
	if (rootStrength > 1)
		rootStrength = 1;
 
	GsQuat qDes =  h->getDesiredHeading()* h->skref()->root()->rot()->value();
// 	GsQuat qCurr =  h->root()->getGlobalOrientation();
// 	GsQuat qChange = qCurr.inverse()*qDes;
// 
// 
// 	if(qChange.angle()>pFloat(root_max_ang))
// 		qChange.set(qChange.axis(),(pFloat(root_max_ang)) );
// 	
// 	qDes = qCurr*qChange;

	qDes.normalize();

	//so this is the net torque that the root wants to see, in world coordinates
	rootTorque = computePDTorque(manager->getWorld()->getSimStep(), h->root()->getGlobalOrientation(),qDes, h->root()->getRotationalVelocity(), GsVec(),pFloat(root_gain_p),pFloat(root_gain_d));

	//h->root()->box->addGlobalTorque(rootTorque);


	//we need to compute the total torque that needs to be applied at the hips so that the final net torque on the root is rootTorque
	GsVec rootMakeupTorque;
	for (int i=0;i<h->numJoints();i++)
	{
		if (h->joint(i)->getParent() == h->hips())
		{	
			rootMakeupTorque += h->joint(i)->getTorque();
		}
	}
	

	rootMakeupTorque += rootTorque;

	setP(root_torque,rootMakeupTorque);

	if( pFloat(root_stance_hip_damping) > 0 ) 
	{
		GsVec wRel = h->hips()->getGlobalOrientation().apply(h->hips()->getRotationalVelocity()) - h->stanceHip()->getGlobalOrientation().apply(h->stanceHip()->getRotationalVelocity());
		float wRelLen = wRel.len();
		if (wRelLen > pFloat(root_stance_hip_max_velocity) )
			wRel = wRel * (pFloat(root_stance_hip_max_velocity) /wRelLen);
		h->stanceHip()->addGlobalTorque( wRel * (pFloat(root_stance_hip_damping) * wRelLen));

// 		wRel = h->hips()->getRotationalVelocity() - h->swingHip()->getRotationalVelocity();
// 		wRelLen = wRel.len();
// 		if (wRelLen > pFloat(root_stance_hip_max_velocity)  ) wRel = wRel * (pFloat(root_stance_hip_max_velocity) /wRelLen);
// 		h->swingHip()->addGlobalTorque(wRel * (pFloat(root_stance_hip_damping) * wRelLen));
	}
	



	//now, based on the ratio of the forces the feet exert on the ground, and the ratio of the forces between the two feet, we will compute the forces that need to be applied
	//to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!
	h->stanceHip()->addGlobalTorque( rootMakeupTorque * h->getStanceSwingRatio() * rootStrength);
	h->swingHip()->addGlobalTorque(  rootMakeupTorque * (1.0f-h->getStanceSwingRatio()) * rootStrength);



	return valid;
}




