//ODEBox.cpp

#include "ode_model.h"
#include <math.h>
#include <gsim/gs_box.h>

ODEModel::ODEModel(ODEWorld* world, const GsString& name, const GsString& file): ODEObject(world,name, file)
{

	CHECK_VEC(ode_box_dim);
	CHECK_STRING(ode_model_filename);

	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif

	GsVec sze = pVec(ode_box_dim);
	GsVec pos = pVec(ode_position);
	
	model = new OBJModel(pString(ode_model_filename));

	if(pFloat(ode_mass)==0)
	{
		float newMass = sze.x*sze.y*sze.z;
		makeParameter(ode_mass,"ode_mass",(float)newMass);
	}

	thisGeomID = dCreateBox(world->GetSpaceID(), sze.x , sze.y,sze.z);

	if (pBool(ode_dynamic))
	{
		thisBodyID = dBodyCreate(world->GetWorldID());
		dGeomSetBody(thisGeomID, thisBodyID);

		dBodySetForce(thisBodyID, 0, 0, 0);
		
	}
	else
	{
		dGeomSetBody(thisGeomID, 0);
	}
	applyParameters();

}
void ODEModel::applyParameters()
{
	ODEObject::applyParameters();
	
	//if(thisBodyID)dBodyDestroy(thisBodyID);
	//if(thisGeomID)dGeomDestroy(thisGeomID);

	
	GsVec sze = pVec(ode_box_dim);
	GsVec pos = pVec(ode_position);

	setPosition(pos);
	setOrientation(pQuat(ode_orientation));

	if (pBool(ode_dynamic))
	{
		dReal newMass = pFloat(ode_mass);
		
		dMass mass;
		dBodyGetMass(thisBodyID, &mass);
		dMassAdjust(&mass, newMass);
		dBodySetMass(thisBodyID,&mass);
		
		GsVec vel = pVec(ode_velocity);
		setVelocity(vel);

		GsVec angVel = pVec(ode_rotational_velocity);
		setRotVelocity(angVel);

	}
	


}



ODEModel::~ODEModel()
{
	
	
}



