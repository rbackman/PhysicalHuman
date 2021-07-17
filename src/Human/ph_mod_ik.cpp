#include "ph_mod_ik.h"
#include "ph_manip_ik.h"
#include "ph_mod_ref.h"
#include "util_models.h"



//IK Controller class
IKModule::IKModule(PhysicalHuman* human,const char* file): ManipulatorModule(human,"IKModule",file)
{
	_shortName = "ik";
	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif
}

void IKModule::init()
{
	ManipulatorModule::init();

	for(int i=0;i<manips.size();i++)
	{
		if(manips.get(i)->name() =="LeftFoot")leftFootIK = (IkManipulator*)manips.get(i);
		if(manips.get(i)->name() =="RightFoot")rightFootIK = (IkManipulator*)manips.get(i);
		if(manips.get(i)->name() =="LeftHand")leftHandIK = (IkManipulator*)manips.get(i);
		if(manips.get(i)->name() =="RightHand")rightHandIK = (IkManipulator*)manips.get(i);
		if(manips.get(i)->name() =="Hips")rootManip = (IkManipulator*)manips.get(i);
	}
	
	

}
HumanManipulator* IKModule::makeManip(GsString name,GsString file)
{
	KnJoint* j = h->skref()->joint(name);
	if(j)
	{
		IkManipulator* manip = new IkManipulator(this,j,file);
		
		return (HumanManipulator*)manip;
	}
	gsout<<"bad joint name "<<name<<" so no manip\n";

	return 0;
}

bool IKModule::evaluate()
{
	if(!isActive())return false;

	if(pBool(manip_module_match))
	{
		matchToSkeleton();
	}
	else
	{
		for (int i=0;i<numManips();i++)
		{
			if(!manips[i]->isActive())
				manips[i]->match();
		}
	}

	ManipulatorModule::evaluate();

	return true;
}
void IKModule::applyParameters()
{

	ManipulatorModule::applyParameters();


	ManipulatorModule::evaluate();

}


IkManipulator* IKModule::getRootManip()
{
		return rootManip;
}

IkManipulator* IKModule::getLeftFootManip()
{
		return leftFootIK;
}
IkManipulator* IKModule::getRightFootManip()
{
	return rightFootIK;
}
IkManipulator* IKModule::getLeftHandManip()
{
	return leftHandIK;
}
IkManipulator* IKModule::getRightHandManip()
{
	return rightHandIK;
}
void IKModule::setRightVisible(bool vis)
{
	getRightFootManip()->visible(vis);
}
void IKModule::setLeftVisible(bool vis)
{
	getLeftFootManip()->visible(vis);
}
void IKModule::setRootVisible(bool vis)
{
	getRootManip()->visible(vis);
}

float IKModule::getHipHeight()
{
	return h->skref()->root()->pos()->value(1);
}

GsVec IKModule::leftFootIKPos()
{
	return leftFootIK->globalPosition();
}
GsVec IKModule::rightFootIKPos()
{
	return rightFootIK->globalPosition();
}
GsQuat IKModule::leftFootIKRot()
{
	return leftFootIK->localOrientation();
}
GsQuat IKModule::rightFootIKRot()
{
	return rightFootIK->localOrientation();
}
GsVec IKModule::leftHandIKPos()
{
	return leftHandIK->localPosition();
}
GsVec IKModule::rightHandIKPos()
{
	return rightHandIK->localPosition();
}
GsQuat IKModule::leftHandIKRot()
{
	return leftHandIK->localOrientation();
}
GsQuat IKModule::rightHandIKRot()
{
	return rightHandIK->localOrientation();
}

GsQuat IKModule::getRootRot()
{
	return rootManip->localOrientation();
}

IKModule::~IKModule(void)
{
	
	grp->remove_all();

	rootManip=0;


}

void IKModule::setRootPosition( GsVec v )
{

	rootManip->translation(v);
	rootManip->evaluate();

}

GsVec IKModule::getRootManipPos()
{
	return rootManip->globalPosition();
}

void IKModule::solve(bool force)
{
	if(force)
	{
		rootManip->solve();
		leftFootIK->solve();
		rightFootIK->solve();
		rightHandIK->solve();
		leftHandIK->solve();
	}
	else
	{
		if(rootManip->isActive())rootManip->solve();
		if(leftFootIK->isActive())leftFootIK->solve();
		if(rightFootIK->isActive())rightFootIK->solve();
		if(rightHandIK->isActive())rightHandIK->solve();
		if(leftHandIK->isActive())leftHandIK->solve();
	}
}

IkManipulator* IKModule::getSwingManip()
{
	if(h->leftStance())
		return getRightFootManip();
	return getLeftFootManip();
}

IkManipulator* IKModule::getStanceManip()
{
	if(h->leftStance())
		return getLeftFootManip();
	return getRightFootManip();
}

IkManipulator* IKModule::getSwingHandManip()
{
	if(h->leftStance())
		return getRightFootManip();
	return getLeftFootManip();
}

IkManipulator* IKModule::getStanceHandManip()
{
	if(h->leftStance())
		return getLeftFootManip();
	return getRightFootManip();
}

