//ODEBox.cpp

#include "ode_box.h"


ODEBox::ODEBox(ODEWorld* world, const GsString& name, const GsString& file): ODEObject(world,name, file)
{
#ifdef PRINT_CONSTRUCTORS
	gsout<<"ODEBox(ODEWorld* world, GsString name, GsString file)"<<gsnl;
#endif

	CHECK_VEC(ode_box_dim);
	
	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif

	init();
}
void ODEBox::applyParameters()
{
	ODEObject::applyParameters();
	
	GsVec sze = pVec(ode_box_dim);
	GsVec pos = pVec(ode_position);

	setPosition(pos);

	setOrientation(pQuat(ode_orientation));


	if (pBool(ode_dynamic))
	{
		dReal newMass = pFloat(ode_mass);
		if(newMass<=0)
		{
			newMass = 0.5;
			phout<<"no mass found for "<<name()<<gsnl;
		}
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

void ODEBox::init()
{
	setType("Box");
	GsVec sze = pVec(ode_box_dim);
	GsVec pos = pVec(ode_position);

	model = new Box(pos,sze);
	model->setColor(pColor(ode_object_color));

	thisGeomID = dCreateBox(world->GetSpaceID(),sze.x,sze.y,sze.z);
	dGeomSetPosition(thisGeomID,pos.x,pos.y, pos.z);
	if (isDynamic())
	{
		thisBodyID = dBodyCreate(world->GetWorldID());
		dGeomSetBody(thisGeomID, thisBodyID);

		//multiply volume by 8 to get mass; measured in kilograms?
		dReal newMass = sze.x*sze.y*sze.z*1000*pFloat(ode_density);
		setP(ode_mass,(float)newMass);

		dMass mass;
		dBodyGetMass(thisBodyID, &mass);
		dMassAdjust(&mass, newMass);
		dBodySetMass(thisBodyID,&mass);


		dBodySetForce(thisBodyID, 0, 0, 0);
		dBodySetLinearVel(thisBodyID, 0, 0, 0);
		dBodySetAngularVel(thisBodyID, 0, 0, 0);

	}
	else
	{
		dGeomSetBody(thisGeomID, 0);
	}
	setPosition(pos);
}
ODEBox::ODEBox(ODEWorld* world, bool body, GsVec pos, GsVec sze) : ODEObject(world)
{
#ifdef PRINT_CONSTRUCTORS
	gsout<<"ODEBox(ODEWorld* world, bool body, GsVec pos, GsVec sze)"<<gsnl;
#endif
	setP(ode_dynamic,body);
	setP(ode_position,pos);
	
	
	makeParameter(ode_box_dim,"ode_box_dim",sze);

	setP(ode_dynamic,body);
	setP(ode_position,pos);

	init();
}

ODEBox::~ODEBox()
{

}



