#pragma once
#include "ph_mod.h"

/*
this controller is responsible for determining the current contact state and changing to a new one
when desired. or by automatically detecting changes in contact based on foot rotation and position
it maintains 4 pressure sensors on each foot that are computed from the rigid body collisions .

the contact controller has an idea where it wants to put the com but it can be overridden if there
is a motion playing

*/



class Sensor
{
private:
	GsVec val;
	float overSample;
	GsVec sensorPos;
	float len;
	PhysicalJoint* joint;
	int lastUpdate;
	bool active;
public:
	Sensor(PhysicalJoint* j, GsVec pos);
	Line* line;
	Ball* ball;
	//check if the sensor was activated on last simulation loop
	bool isActive(){return active;}
	//test if a contact applies to this sensor.. if it is close enough it assumes it is
	bool checkCollision(ODEContact* cont);
	bool update();


	//get the current sensor value;
	GsVec getVal(){return val;}
	GsVec getPos(){return joint->getCOMPosition() + joint->getGlobalOrientation().apply(sensorPos);}
};



class ContactModule : public Module
{
public:

	GsString stateString();
	ContactModule(PhysicalHuman* human,const char* file);
	~ContactModule(void);
	void init();
	//check if the COM projection is within the convex hull of the feet
	bool balanced();
	bool flying();
	GsVec leftComOffset();
	GsVec rightComOffset();
	GsVec stanceComOffset();
	GsVec swingComOffset();
	void FixFeet();
	bool bothFeetContain(GsVec p);
	bool leftFootContains(GsVec p);
	bool rightFootContains(GsVec p);
	bool stanceFootContains(GsVec p);
	bool swingFootContains(GsVec p);

	//determine if the foot is planted by checking all the sensors for recent activation
	bool leftFootPlanted();
	bool rightFootPlanted();
	bool stanceFootPlanted();
	bool swingFootPlanted();

	bool bothFeetPlanted();
	float leftFootStrength();
	float rightFootStrength();
	GsVec getIPStepPos();
	GsVec stancePoint();
	void setStanceSwingRatio(float f)
	{
		setP(contact_stance_swing_ratio, boundToRange(f,0,1));
	}

private:
	bool _flying;
	
	GsPolygon updateContactRegion();
	GsVec updateContactCenter();

	GsPolygon contact_poly;
	GsPolygon contact_proj_poly;

	SnGroup* _sensors_lines;
	SnLines* _contact_lines;
	SnLines* _contact_proj_lines;
	Ball* copBall;
	Box* ip_model;
	Ball* contact_model;


	enum sensor_positions
	{
		LFL,
		LFR,
		LBL,
		LBR,
		RFL,
		RFR,
		RBL,
		RBR
	};
	Sensor* sensors[8]; //these are the four virtual sensor values.


public:
	void setTransitionSpeed(float s){setP(contact_transition_speed,s); setP(contact_root_transition_speed ,s*0.5f);}
	GsVec getContactCenter(){return pVec(contact_pos);}
	GsVec getStableContactCenter();
	bool evaluate();
	GsVec getCOP(){return pVec(contact_cop_pos);}
	void applyParameters();
	bool leftFootContact();
	bool rightFootContact();
	GsVec getContactProjectionCenter(){return pVec(contact_proj_pos);}
	
};
