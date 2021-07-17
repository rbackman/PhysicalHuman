//ode_world.h


#ifndef __ODEWORLD_H__
#define __ODEWORLD_H__

#include <ode/ode.h>
#include "ode_contact.h"
#include "util_serializable.h"

#define MAX_CONTACT_FEEDBACK 64
#define MAX_CONTACT_POINTS 64
//#define TIME_MULTIPLIER 2 //don't go much above 5
#define STEP_ITERATIONS 10
//#define STEP_SIZE 0.03 //0.03 looks like real-time
class GsVec;
class ODEObject;

enum ode_world_parms
{
	world_time_scale,
	world_erp,
	world_cfm,
	world_contact_surface_layer,
	world_gravity,
	world_auto_disable_steps,
	world_simulation_over_sample,
	world_simulation_fps,
	world_gain_mult,
	world_ode_torque_color_feedback,
	world_bounce,
	world_friction,
	world_limit_contacts
};


class ODEWorld : public Serializable
{
public:
	ODEWorld(const GsString& file);
	~ODEWorld();

	void ODELoop();
	//void ResetTimer(); //use this for window resizes

	void applyParameters();

	float getSimStep();
	float getAnimationStep();
	dWorldID GetWorldID();
	dSpaceID GetSpaceID();
	GsVec GetGravity();
	dReal GetRayIntersectionDepth();
	bool collides(ODEObject* obj);
	void addObject( ODEObject* param1 );
	GsArray<ODEContact*> contactPoints;
	GsArray<ODEObject*> objects;

	//this is the max number of contacts that are going to be processed between any two objects
	int maxContactCount;

	dJointFeedback jointFeedback[MAX_CONTACT_FEEDBACK];
	//this is the current number of contact joints, for the current step of the simulation
	int jointFeedbackCount;
	bool nonFootContact(){return _bad_contact;}
	void badContact( bool val );
	ODEObject* getObject( dGeomID o1);
	ODEObject* getObject( dBodyID b1);
	void reset();
	dJointGroupID getJointGroupID();
private:
	bool _bad_contact;
	dWorldID theDynamicsWorldID;
	dSpaceID theCollisionSpaceID;
	dJointGroupID theJointGroupID;
	dReal timeScale;
	dReal rayIntersectionDepth; //hack to keep track of ray penetration depth
};

//Helper function (this must not be a member of ODEWorld since we are passing its address as an argument)
void PotentialHitCallback(void *data, dGeomID o1, dGeomID o2);

#endif
