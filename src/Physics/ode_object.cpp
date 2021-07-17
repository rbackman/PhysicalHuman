//ODEObject.cpp

#include "ode_object.h"





ODEObject::ODEObject(ODEWorld* world, const GsString& name, const GsString& file) : Serializable(name)
{
	
	this->world = world;
	_collides = true;
	model = 0;
	thisBodyID = 0;
	thisGeomID = 0;
	world->addObject(this);
	if(file=="none")
	{
		MAKE_PARM(ode_position,GsVec());
		MAKE_PARM(ode_dynamic,true);
		MAKE_PARM(ode_is_ground,false);
		MAKE_PARM(ode_object_color,GsColor::blue);
		MAKE_PARM(ode_mass,1.0f);
		MAKE_PARM(ode_density,1.0f);
		MAKE_PARM(ode_orientation,GsQuat());
		//friction and bounce are set to zero since they should use global vals
		MAKE_PARM(ode_friction,0.0f); 
		MAKE_PARM(ode_bounce,0.1f);
		MAKE_PARM(ode_unique_properties,false);
	}
	else
	{
		loadParametersFromFile(file);
		CHECK_VEC(ode_position);
		CHECK_BOOL(ode_dynamic);
		CHECK_BOOL(ode_is_ground);
		CHECK_COLOR(ode_object_color);
		CHECK_QUAT(ode_orientation);
		if(!CHECK_FLOAT(ode_mass))setP(ode_mass,1.0f);
		if(!CHECK_FLOAT(ode_density))setP(ode_density,1.0f);
		CHECK_FLOAT(ode_friction);
		CHECK_FLOAT(ode_bounce);
		CHECK_BOOL(ode_unique_properties);
	}
	
		
		MAKE_TEMP_PARM(ode_velocity,GsVec());
		MAKE_TEMP_PARM(ode_rotational_velocity,GsVec());


}


ODEObject::~ODEObject()
{
	if (isDynamic())
	{
		dBodyDestroy(thisBodyID);
	}
	dGeomDestroy(thisGeomID);

	if(model)
		delete model;
	
}
void ODEObject::setColor(GsColor c)
{
	model->setColor(c);
}
float ODEObject::getMass()
{
	dMass mass;
	dBodyGetMass(thisBodyID,&mass);
	return (float)mass.mass;

}
void ODEObject::addForce(GsVec p,GsVec f)
{
	dBodyAddForceAtPos(thisBodyID,f.x,f.y,f.z,p.x,p.y,p.z);
}
void ODEObject::addGlobalTorque(GsVec tq)
{
	dBodyAddTorque(thisBodyID,tq.x,tq.y,tq.z);
}

void ODEObject::reset()
{
	setVelocity(GsVec());
	setRotVelocity(GsVec());
	dBodySetTorque(thisBodyID,0,0,0);
	dBodySetForce(thisBodyID,0,0,0);
}


GsVec ODEObject::getPosition()
{
	if(isDynamic())
	{
		const dReal* p = dBodyGetPosition(thisBodyID);
		return GsVec(p[0],p[1],p[2]);
	}
	const dReal* p = dGeomGetPosition(thisGeomID);
	return GsVec(p[0],p[1],p[2]);
}


GsQuat ODEObject::getOrientation()
{
	if(isDynamic())
	{
	const dReal* Q = dBodyGetQuaternion(thisBodyID);
	return GsQuat(Q[0],Q[1],Q[2],Q[3]);
	}
	else
	{
		return GsQuat();
	}
}
void ODEObject::update(bool render)
{
	if(isDynamic())
	{
		setP(ode_position,getPosition());
		setP(ode_orientation,getOrientation());
		setP(ode_velocity,getVelocity());
		setP(ode_rotational_velocity,getRotationalVelocity());
		if(render)
		{
			model->setPosition(pVec(ode_position));
			model->setRotation(getOrientation());
		}
		
	}
}

void ODEObject::setVelocity(GsVec velocity)
{
	if (isDynamic())
	{
		dBodySetLinearVel(thisBodyID, velocity.x, velocity.y, velocity.z);
	}
	else // this object is only a geom
	{
		//geoms have no velocity, so this shouldn't be called
	}
}
void ODEObject::setRotVelocity(GsVec velocity)
{
	if (isDynamic())
	{
		dBodySetAngularVel(thisBodyID, velocity.x, velocity.y, velocity.z);
	}
	else // this object is only a geom
	{
		//geoms have no velocity, so this shouldn't be called
	}
}
void ODEObject::setPosition(GsVec position)
{
	if (isDynamic())
	{
		dBodySetPosition(thisBodyID, position.x, position.y, position.z);
		
	}
	else // this object is only a geom
	{
		//geoms have no velocity, so this shouldn't be called
	}
	model->setPosition(position);
}
void ODEObject::setOrientation(GsQuat q)
{
	
if (isDynamic())
	{
		dQuaternion qt = {q.w,q.x,q.y,q.z};
		dBodySetQuaternion(thisBodyID, qt);
		
	}
	else // this object is only a geom
	{
		//geoms have no velocity, so this shouldn't be called
	}
	model->setRotation(q);
}
bool ODEObject::Rotating()
{
	if(getRotationalVelocity().len()>1.0)return true;
	return false;
}
GsVec ODEObject::getRotationalVelocity()
{
const dReal* velocity;
	GsVec newVelocity;

	if (isDynamic())
	{
		 
		velocity = dBodyGetAngularVel(thisBodyID);
		newVelocity = GsVec( velocity[0], velocity[1], velocity[2]); 
	}
	else // this object is only a geom
	{
		//geoms have no velocity, so this shouldn't be called
		
	}
	return newVelocity;
}
GsVec ODEObject::getVelocity()
{
	const dReal* velocity;
	GsVec newVelocity;

	if (isDynamic())
	{
		velocity = dBodyGetLinearVel(thisBodyID);

		newVelocity = GsVec( velocity[0], velocity[1], velocity[2]); 

		return newVelocity;
	}
	else // this object is only a geom
	{
		//geoms have no velocity, so this shouldn't be called
		return newVelocity;
	}
}

dBodyID ODEObject::getBodyID(){ return thisBodyID;}
void ODEObject::SetBodyID(dBodyID bodyID){thisBodyID = bodyID;}
dGeomID ODEObject::GetGeomID(){return thisGeomID;}
void ODEObject::SetGeomID(dGeomID GeomID){thisGeomID = GeomID;}

bool ODEObject::isGround()
{
	return pBool(ode_is_ground);
}

void ODEObject::setParameters( float friction,float restitution )
{
	//dBodySetMass()
}

