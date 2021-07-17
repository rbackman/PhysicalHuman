#pragma once
#include "ph_mod.h"

//This controller follows a reference KnSkeleton. It contains an array of pids linked between the ref
//joint and the corresponding PhysicalJoint. it assumes that all the joints in the PhysicalHuman have joints in the
//KnSkeleton with the same names.

class PID 
{
private:
	PhysicalJoint* joint;	
	KnJoint* knJoint;
public:
	PID(PhysicalJoint* jnt,KnJoint* kjnt);
	~PID(void);
	bool evaluate();
	PhysicalJoint* getJoint(){return joint;}
	KnJoint* getKnJoint(){return knJoint;}
	bool characterFrame;
private:
	//used for the integral component which I never really use so I may delete it.
	GsVec previousError;
	GsVec integral;
	GsVec setpoint;
	GsVec position;
	GsVec error;
	GsVec derivative;
	GsVec output;
};

class ReferenceModule: public Module
{
public:
	ReferenceModule(PhysicalHuman* human,const char* file);
	ReferenceModule(PhysicalHuman* human,KnSkeleton* sk,const char* file);
	~ReferenceModule();
	bool evaluate();
	void attachRefSkel(KnSkeleton* s);
	//this sets the reference skeleton joint angles based on the joint angles of the PhysicalHuman
	void applyToRefSkel(KnSkeleton* sk);
	void applyParameters();
	//get a pid by its name. used by the UI
	PID* pid(const char* name);
	void init();
	void removePIDS();
	void calculateFeetGains();
	GsVec calculateSkeletonCom();
	KnScene* _knsref;
	KnSkeleton* skref;
	KnScene* getKnScene(){return _knsref;}
private:
	GsArray<PID*> pids;
};
