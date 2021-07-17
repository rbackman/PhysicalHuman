#pragma once

#include "common.h"
#include "ph_mod.h"

class HumanManipulator;
class ManipulatorModule;

/*
it proved convientent to have a generic manipulatorController class since several controllers use manipulators
like the ik puppet virtual..etc
*/

class ManipulatorModule : public Module
{
private:
	
	
protected:
	GsString _shortName;
	GsArray<HumanManipulator*> manips;

	HumanManipulator* frameManip;
public:

	ManipulatorModule(PhysicalHuman* human,GsString controllerName,GsString file);

	virtual HumanManipulator*  makeManip(GsString name,GsString file);

	GsVec getOrigin(){return pVec(manip_module_origin);}
	GsQuat getFrame(){return pQuat(manip_module_frame);}

	HumanManipulator* getManip(GsString l);
	HumanManipulator* getManip(int i){if(i==manips.size())return frameManip; else return manips[i];}
	HumanManipulator* getFrameManip(){return frameManip;}
	SnGroup* manipGroup;

	bool evaluate();
	
	GsArray<Serializable*> getSerializables();
	GsString stateString();
	GsString manipStateString();
	GsString manipString();
	GsString getShortName(){return _shortName;}
	void activateManip(GsString name);
	void deactivateManip(GsString name);
	void matchManips(); /*this will match to the manipulators parent*/
	void matchToSkeleton(); /*this will match to a skeleton overriding the parent*/
	void matchToSkeleton(KnSkeleton* sk);
	void matchToHuman(); /*this will match to a _human overriding the parent*/
	void matchFrameToHuman();
	void init();
	void setOrigin( GsVec pos, GsQuat rot );
	void applyParameters();
	int numManips(){return manips.size();}
	void lockSwing( bool param1 );
	HumanManager* getManager();
};
