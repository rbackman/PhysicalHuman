#pragma once
#include "ph_mod_manip.h"

/*
this controller is responsible for solving the analytical ik to achieve the root and foot positions
as close as possible. maintains a set of IK manipulators that operate on _skref in reference_controller()
*/


class Manipulator;
class IkManipulator;


class IKModule : public ManipulatorModule
{
	
	enum foot_index
	{
		left_hand_index=0,
		right_hand_index,
		left_foot_index,
		right_foot_index,
		root_index
	};

private:
	
	IkManipulator* leftFootIK;
	IkManipulator* rightFootIK;
	IkManipulator* leftHandIK;
	IkManipulator* rightHandIK;
	IkManipulator* rootManip;
	
public:

	IKModule(PhysicalHuman* human,const char* file);
	~IKModule(void);
	HumanManipulator* makeManip(GsString name,GsString file);

	bool evaluate();

	SnGroup* getGroup(){return grp;}

	
	void setRightVisible(bool vis);
	void setLeftVisible(bool vis);
	void setRootVisible(bool vis);
	float getHipHeight();


	GsQuat leftFootIKRot();
	GsQuat rightFootIKRot();
	GsQuat getRootRot();
	GsQuat leftHandIKRot();
	GsQuat rightHandIKRot();


	GsVec leftHandIKPos();
	GsVec rightHandIKPos();
	GsVec leftFootIKPos();
	GsVec rightFootIKPos();


	IkManipulator* getLeftFootManip();
	IkManipulator* getRightFootManip();
	IkManipulator* getRootManip();
	IkManipulator* getLeftHandManip();
	IkManipulator* getRightHandManip();
	void applyParameters();

	void solve(bool force = true);
	void init();
	void setRootPosition( GsVec v );
	GsVec getRootManipPos();
	IkManipulator* getSwingManip();
	IkManipulator* getStanceManip();
	IkManipulator* getSwingHandManip();
	IkManipulator* getStanceHandManip();
};
