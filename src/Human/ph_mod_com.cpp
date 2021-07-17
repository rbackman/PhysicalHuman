#include "ph_mod_com.h"
#include "ph_human.h"
#include "ph_joint.h"


#include <gsim/kn_skeleton.h>
#include "common.h"
#include "ph_manager.h"

COMModule::COMModule(PhysicalHuman* human,const char* file):Module(human,"COMModule",file)
{
	h=human;
	
	CHECK_VEC(com_proj_pos);
	CHECK_VEC(com_pos);
	CHECK_VEC(com_desired_pos);
	CHECK_FLOAT(com_sample_ratio);
	CHECK_VEC(com_velocity);
	CHECK_FLOAT(com_velocity_sample);
	CHECK_VEC(com_support_vector);
	CHECK_VEC(com_desired_support_vector);


	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif  

			setP(com_pos,computeCOM());

	com_model	 = new Ball(getCOM(),0.01f,GsColor::yellow); 
	com_proj_model = new Ball(getCOM(),0.01f,GsColor::yellow); 
	com_d_model	 = new Ball(getCOM(),0.01f,GsColor::blue); 
	com_d_proj_model = new Ball(getCOM(),0.01f,GsColor::green);
	com_d_support_vector_model = new Ball(getCOM(),0.01f,GsColor::green);
	support_line = new Line(h->getStancePosition(),getDesiredCOMFromSupportVec());
	grp->add(support_line->getGrp());
	grp->add(com_model->getGrp());
	grp->add(com_proj_model->getGrp());
	grp->add(com_d_model->getGrp());
	grp->add(com_d_proj_model->getGrp());
	grp->add(com_d_support_vector_model->getGrp());
}
void COMModule::init()
{
	setP(com_pos,computeCOM());
	setP(com_desired_support_vector,getCOM() - h->getStancePosition());
	
}
void COMModule::applyParameters()
{
	Module::applyParameters();
	GsVec ballPos = h->getStancePosition() + pVec(com_desired_support_vector);
	com_d_support_vector_model->setPosition(ballPos);
}
COMModule::~COMModule(void)
{

}



GsVec COMModule::computeSupportVector()
{
	GsVec vec;
	if(h->leftStance())
		vec = getCOM()-h->leftFoot()->getJointPosition();
	else
		vec = getCOM()-h->rightFoot()->getJointPosition();

	return vec;
}
GsVec COMModule::getSupportVector()
{
	return pVec(com_support_vector);
}

GsVec COMModule::getDesiredSupportVector()
{
// 	if(h->getStanceState() != STANCE_LEFT)
// 	{
// 		GsVec v = pVec(com_desired_support_vector);
// 		v.x = -v.x;
// 		return v;
// 	}
	return pVec(com_desired_support_vector);

}

GsVec COMModule::computeCOMVelocity()
{
	GsVec COMVel;
	float curMass = 0;
	float totalMass = 0;
	for (int i=0; i <h->numJoints(); i++){
		curMass = h->joint(i)->getBody()->getMass();
		totalMass += curMass;
		COMVel += h->joint(i)->getVelocity()*curMass;
	}

	COMVel /= totalMass;

	return COMVel*(1-pFloat(com_velocity_sample)) + pVec(com_velocity) * pFloat(com_velocity_sample);

}

bool COMModule::evaluate()
{
	setP(com_velocity, computeCOMVelocity());
	setP(com_pos,computeCOM());
	setP(com_support_vector,computeSupportVector());

	if(manager->animationStep())
	{
	
		GsVec desiredSupport = getDesiredCOMFromSupportVec();
		support_line->setPoints(h->getStancePosition(),desiredSupport);
		com_d_support_vector_model->setPosition(desiredSupport);
		GsVec com = pVec(com_pos);
		GsVec comd = pVec(com_desired_pos);
		com_model->setPosition(com);
		com_d_model->setPosition(comd);

		com.y = h->floorHeight();

		comd.y = com.y;
		setP(com_proj_pos,com);

		com_proj_model->setPosition(com);
		com_d_proj_model->setPosition(comd);
	}

	
	return Module::evaluate();
}

GsVec COMModule::computeCOM()
{
	GsVec ncom;
	float totalMass = 0;
	for(int i=0;i<h->numJoints();i++)
	{
		
		totalMass += h->joint(i)->getBody()->getMass();
		ncom += h->joint(i)->getBody()->getMass()* h->joint(i)->getCOMPosition(); 
	}
	ncom = ncom/(totalMass);
	ncom = pFloat(com_sample_ratio)*getCOM() + (1.0f-pFloat(com_sample_ratio))*ncom;

	return ncom;
}

GsVec COMModule::getDesiredCOMFromSupportVec()
{
	return h->getStancePosition() + h->getDesiredHeading().apply(getDesiredSupportVector());
}

GsString COMModule::stateString()
{
	GsString f = name();
	f<<"\n{\n";
	f<<parameterAsString(module_active);
	f<<parameterAsString(com_desired_pos);
	f<<parameterAsString(com_desired_support_vector);
	f<<"}\n";
	return f;
}

