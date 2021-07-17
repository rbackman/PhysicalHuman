

#ifndef __ODE_OBJECT_H__
#define __ODE_OBJECT_H__

#include "util_models.h"
#include "common.h"
#include "util.h"
#include <ode/ode.h>
#include "util_models.h"
#include "util_serializable.h"
#include "ode_world.h"





enum ode_object_parms
{
	ode_dynamic,
	ode_is_ground,
	ode_object_color,
	ode_position,
	ode_velocity,
	ode_orientation,
	ode_rotational_velocity,
	ode_mass,
	ode_density,
	ode_unique_properties,
	ode_friction,
	ode_bounce,
	ode_box_dim,
	ode_model_filename
};

struct body_force
{
	GsVec f;
	float time_left;
	ODEObject* body;
};

class ODEObject : public Serializable
{
public:
	ODEObject(ODEWorld* world, const GsString& name = "none",const GsString& file = "none");
	virtual ~ODEObject();
	void update(bool render);
	void setColor(GsColor c);
	GsVec getPosition();
	GsQuat getOrientation();
	void setVelocity(GsVec velocity);
	void setRotVelocity(GsVec velocity);
	void setPosition(GsVec velocity);
	void setOrientation(GsQuat q);

	bool Rotating();
	GsVec getRotationalVelocity();
	void addGlobalTorque(GsVec tq);
	void addForce(GsVec p,GsVec f);
	GsVec getVelocity();
	float getMass();
	dBodyID getBodyID();
	void SetBodyID(dBodyID bodyID);
	dGeomID GetGeomID();
	void SetGeomID(dGeomID geomID);
	bool isDynamic(){return pBool(ode_dynamic);}
	bool isGround();
	void setParameters(float friction,float restitution);
	void reset();
protected:
	// ODE stuff
	dBodyID thisBodyID;
	dGeomID thisGeomID;
	Model* model;
	ODEWorld* world;
	bool _collides;
public:
	Model* getModel(){return model;}
	bool collides(){return _collides;}
	void collides(bool c){_collides = c;}
};

#endif
