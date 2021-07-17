#pragma once
#include "ph_mod.h"
#include "common.h"

class IKModule;
class HumanWindowGraphViewer;
class Channel;
/*
The walk controller should work like this:

to start the char should shift his weight to the stance foot with the stanceSwingRatio
once the COM is a percentage of the way to the desired location on the stance foot.
then the swing foot should start lifting.. the trajectory of the foot should be determined based 
on the inverted pendulum prediction. and feedback tracking with virtual forces will be applied to 
maintain the trajectory. If the foot makes contact earlier than expected the step phase should be 
set to 1 and the next,opposite stance, motion should be started. 


*/
class WalkController : public Module
{

public:
	WalkController(PhysicalHuman* human,const char* file);
	~WalkController(void);
	
	IKModule* ik;
	GsArray<Channel*> channels;
	
	SnManipulator* stepManip;
	Curve* com_line;
	Curve* swing_line;


	float walkSpeed;
	float stepSpeed;
	float stepHeight;
	float stanceOff;

	bool swingLifted;
	human_stance stanceFoot;
	float stepPhase;
	HumanWindowGraphViewer* graph;
	
	bool evaluate();
	void activate();
	
	void init();
	void stepTo(GsVec p3);
	Channel* getChannel(GsString name);
private:

public:

	
};
