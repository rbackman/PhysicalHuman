#pragma once
#include "common.h"
#include "ph_human.h"
#include "util_manipulator.h"

/*
	a generic manipulator that belongs to a ManipulatorController and may keep track of a PhysicalJoint or KnJoint.
*/

class ManipulatorModule;
class HumanManager;
class HumanManipulator: public Manipulator
{
protected:
	HumanManager* manager;

public:

	PhysicalJoint* joint; //for most things a physical joint will be the parent to match
	KnJoint* kn_joint; //for ik this will be the parent
	ManipulatorModule* controller;
	HumanManipulator(ManipulatorModule* cont, GsString name,GsString typ,GsString file);
	HumanManipulator(ManipulatorModule* cont, PhysicalJoint* j,GsString typ,GsString file);
	HumanManipulator(ManipulatorModule* cont, KnJoint* j,GsString typ, GsString file);
	void match();
	void match(KnJoint* j);
	void match(PhysicalJoint* j);
	void setInitial();

};