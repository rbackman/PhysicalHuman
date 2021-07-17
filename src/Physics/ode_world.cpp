//ODEWorld.cpp

#include "ode_world.h"
#include "ode_object.h"

#define CAMERA_COLLIDE_BITS 10
#define RAY_COLLIDE_BITS 20


dReal g_rayIntersectionDepth=-1;

void ODEWorld::applyParameters()
{
	Serializable::applyParameters();
	dWorldSetERP(theDynamicsWorldID, (dReal)pFloat(world_erp));
	dWorldSetCFM(theDynamicsWorldID, (dReal)pFloat(world_cfm));
	dWorldSetContactSurfaceLayer(theDynamicsWorldID,(dReal)pFloat(world_contact_surface_layer));
	
	//dWorldSetAngularDamping(theDynamicsWorldID,0.001);
	//dWorldSetAngularDampingThreshold(theDynamicsWorldID,0.1);
	//dWorldSetAutoDisableSteps(theDynamicsWorldID, pInt(world_auto_disable_steps));
	dWorldSetGravity(theDynamicsWorldID, 0, -pFloat(world_gravity), 0);
}
float ODEWorld::getSimStep()
{
	return 1.0f
			/
		(float)( pInt(world_simulation_fps)* pInt(world_simulation_over_sample) );
}
float ODEWorld::getAnimationStep()
{
	return 1.0f
		/
		(float)( pInt(world_simulation_fps));
}

void ODEWorld::reset()
{
	jointFeedbackCount = 0;
	rayIntersectionDepth = -1;
	_bad_contact = false;
	
	for (int i=0;i<contactPoints.size();i++)
	{
		delete contactPoints[i];
	}
	contactPoints.size(0);
	objects.size(0);
	if(theDynamicsWorldID)
	{
		//dJointGroupEmpty(theJointGroupID);
		dWorldDestroy(theDynamicsWorldID);
		dJointGroupDestroy(theJointGroupID);
		dCloseODE();
	}


	dInitODE();


	theDynamicsWorldID = dWorldCreate();
	theCollisionSpaceID = dSimpleSpaceCreate(0);
	theJointGroupID = dJointGroupCreate(0);

	applyParameters();
}

ODEWorld::ODEWorld(const GsString& file):Serializable("ODEWorld")
{
	
	loadParametersFromFile(file);
	
	CHECK_BOOL(world_ode_torque_color_feedback);

	CHECK_FLOAT(world_time_scale);
	CHECK_FLOAT(world_erp);
	CHECK_FLOAT(world_cfm);
	CHECK_FLOAT(world_contact_surface_layer);
	CHECK_FLOAT(world_gravity);
	CHECK_INT(world_auto_disable_steps);
	CHECK_INT(world_simulation_over_sample);
	CHECK_INT(world_simulation_fps);
	CHECK_FLOAT(world_gain_mult);
	CHECK_FLOAT(world_bounce);
	CHECK_FLOAT(world_friction);
	CHECK_BOOL(world_limit_contacts);

	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif  

	timeScale = pFloat(world_time_scale);

	theDynamicsWorldID = 0;
	reset();
/*
	dInitODE ();

	_bad_contact = false;

	theDynamicsWorldID = dWorldCreate();
	theCollisionSpaceID = dSimpleSpaceCreate(0);
	//dVector3 center = {0, 0, 0};
	//dVector3 extents = {200, 100, 200};
	//theCollisionSpaceID = dQuadTreeSpaceCreate(0, center, extents, 5);
	theJointGroupID = dJointGroupCreate(0);
	//These are ONLY here to allow the callback function to have access to them.
	
	applyParameters();
	*/
//	dWorldSetAutoDisableThreshold(theDynamicsWorldID, (dReal)pFloat(world_auto_disable_threshold));
}
#include <gsim/gs_vec.h>
GsVec ODEWorld::GetGravity()
{
	dVector3 gravity;
	dWorldGetGravity(theDynamicsWorldID,gravity);
	return GsVec(gravity[0],gravity[1],gravity[2]);
}
ODEWorld::~ODEWorld()
{
	//dSpaceDestroy(theCollisionSpaceID);
	objects.size(0);
	dWorldDestroy(theDynamicsWorldID);
	dJointGroupDestroy(theJointGroupID);
	dCloseODE();
}

void ODEWorld::ODELoop()
{
	_bad_contact = false;
	//if (deltaTime > 0.1 || deltaTime <= 0)
	//{
	//	return; //we don't want the simulation to "explode" in these situations
	//}
	dReal deltaTime = getSimStep();

	//clear the previous list of contact forces
	for(int i=0;i<contactPoints.size();i++)
	{
		delete contactPoints.get(i);
	}

	contactPoints.size(0);
	jointFeedbackCount = 0;

	deltaTime *= pFloat(world_time_scale);

	// 1. Apply forces to bodies as necessary.
		//nothing for now...

	// 2. Adjust the joint parameters as necessary.
		//nothing for now...

	// 3,4. Call collision detection; add contacts to contact joint group.
		g_rayIntersectionDepth = -1; //reset this each time
		dSpaceCollide(theCollisionSpaceID, this, &PotentialHitCallback);
		rayIntersectionDepth = g_rayIntersectionDepth;

	// 5. Take a Simulation step.
	   dWorldQuickStep (theDynamicsWorldID,deltaTime); 
		// dWorldStepFast1(theDynamicsWorldID, deltaTime, STEP_ITERATIONS);
		//dWorldStep(theDynamicsWorldID, deltaTime);


	//copy over the force information for the contact forces
	for (int i=0;i<jointFeedbackCount;i++)
	{
		ODEContact* c = contactPoints.get(i);
		c->f = GsVec(jointFeedback[i].f1[0], jointFeedback[i].f1[1], jointFeedback[i].f1[2]);
		//make sure that the force always points away from the static objects
		//if (contactPoints[i].rb1->isLocked() && !contactPoints[i].rb2->isLocked()){
			c->f = c->f * (-1);
			dBodyID tid = c->rb1;
			c->rb1 = c->rb2;
			c->rb2 = tid;
		//}
	}

	// 6. Remove all joints from the contact group.
		dJointGroupEmpty(theJointGroupID);
}

void PotentialHitCallback(void *data, dGeomID o1, dGeomID o2)
{
	ODEWorld* world = (ODEWorld*)data;
	bool collide = true;
	ODEObject* obj1 = world->getObject(o1);
	ODEObject* obj2 = world->getObject(o2);

	if(world->pBool(world_limit_contacts))
	{
		if(world->collides(obj1)&&world->collides(obj2))
		{

		}
		else
		{
			collide = false;
		}
	}

	float bounce = world->pFloat(world_bounce);
	float mu = world->pFloat(world_friction);
	float cfm = world->pFloat(world_cfm);

	if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
	{ // colliding a space with someting
		dSpaceCollide2(o1, o2, data, &PotentialHitCallback);
		
		// now colliding all geoms internal to the space(s)
		if (dGeomIsSpace(o1))
		{
			dSpaceID o1_spaceID = (dSpaceID)o1; // this may not be valid
			dSpaceCollide(o1_spaceID, data, &PotentialHitCallback);
		}
		if (dGeomIsSpace(o2))
		{
			dSpaceID o2_spaceID = (dSpaceID)o2; // this may not be valid
			dSpaceCollide(o2_spaceID, data, &PotentialHitCallback);
		}
	}
	else
	{ // colliding two "normal" (non-space) geoms

		//use category/collide bitfields here

		//don't collide camera ray with camera sphere
		if (dGeomGetCollideBits(o1) == RAY_COLLIDE_BITS && 
			dGeomGetCollideBits(o2) == CAMERA_COLLIDE_BITS ||
			dGeomGetCollideBits(o1) == CAMERA_COLLIDE_BITS && 
			dGeomGetCollideBits(o2) == RAY_COLLIDE_BITS)
		{
			return;
		}

		int num_cp; // number of contact points
		dContactGeom cp_array[MAX_CONTACT_POINTS];
		num_cp = dCollide(o1, o2, MAX_CONTACT_POINTS, cp_array, sizeof(dContactGeom));

		dBodyID o1BodyID = obj1->getBodyID(); // dGeomGetBody(o1);
		dBodyID o2BodyID = obj2->getBodyID(); // dGeomGetBody(o2);

		// filter out collisions between joined bodies (except for contact joints)
		if (o1BodyID !=0 && o2BodyID != 0)
			if (num_cp > 0 && dAreConnectedExcluding(o1BodyID, o2BodyID, dJointTypeContact))
				
			{
			//	phout << "ignored connected\n";			
				return;
			}
		//// checking whether one of the geoms is camera ray
		if (dGeomGetCollideBits(o1) == RAY_COLLIDE_BITS || dGeomGetCollideBits(o2) == RAY_COLLIDE_BITS)
		{	
			//if a contact occurred...
			if(num_cp > 0)
			{
				
				//only keep smallest depth
				 if (g_rayIntersectionDepth == -1 || cp_array[0].depth < g_rayIntersectionDepth)
				{
					g_rayIntersectionDepth = cp_array[0].depth;
				}
			}
			else
			{
				return;
			}
		}
		else if(collide)//we want to do physical reactions with all objects but the ray
		{
			
			// add these contact points to the simulation
			for(int i=0; i<num_cp; i++)
			{
				dContact tempContact;// = new dContact;
				//tempContact.surface.mode = 0;

				//tempContact.surface.mode = dContactApprox1 | dContactSlip1 | dContactSlip2 | dContactSoftCFM;
				//tempContact.surface.slip1 = (dReal)0.0;
				//tempContact.surface.slip2 = (dReal)0.0;

				//tempContact.surface.mode = dContactApprox1;
				
				float mu_to_use = mu;
				float bounce_to_use = bounce;

				if(obj1->pBool(ode_unique_properties) && obj2->pBool(ode_unique_properties))
				{
					mu_to_use = min(obj1->pFloat(ode_friction),obj2->pFloat(ode_friction));	
					bounce_to_use = min(obj1->pFloat(ode_bounce),obj2->pFloat(ode_bounce));
					//here I am guessing that I should use the object with the highest bounce and lowest friction
						
						/*if(obj1->pFloat(ode_friction)<obj2->pFloat(ode_friction))
						{
							mu_to_use = obj1->pFloat(ode_friction);
						}
						else
						{
							mu_to_use = obj2->pFloat(ode_friction);
						}*/

						/*
						if(obj1->pFloat(ode_bounce)<obj2->pFloat(ode_bounce))
						{
							bounce_to_use = obj2->pFloat(ode_bounce);
						}
						else
						{
							bounce_to_use = obj1->pFloat(ode_bounce);
						}*/

				}
				else if(obj1->pBool(ode_unique_properties))
				{
					mu_to_use = min(obj1->pFloat(ode_friction),mu_to_use);
					bounce_to_use = min(obj1->pFloat(ode_bounce),bounce_to_use);
				}
				else if(obj2->pBool(ode_unique_properties))
				{
					mu_to_use = min(obj2->pFloat(ode_friction),mu_to_use);
					bounce_to_use = min(obj2->pFloat(ode_bounce),bounce_to_use);
				}
			
				tempContact.surface.mu = mu_to_use;
				tempContact.surface.bounce = bounce_to_use;
				
				tempContact.surface.mode = dContactBounce| dContactSoftCFM;
				
				tempContact.surface.bounce_vel = (dReal)0.0;
				tempContact.surface.soft_cfm = cfm;
				
				//tempContact.surface.mu = dInfinity;

				
				tempContact.geom = cp_array[i];
				//tempContact.surface.mu2 = 0;

				dJointID j = dJointCreateContact(world->GetWorldID(), world->getJointGroupID(), &tempContact);
				dJointAttach(j, dGeomGetBody(o1), dGeomGetBody(o2));
				ODEContact* c = new ODEContact;
				c->rb1 = o1BodyID;
				c->rb2 = o2BodyID;
				c->jid = j;
				c->cp = GsVec(tempContact.geom.pos[0],tempContact.geom.pos[1],tempContact.geom.pos[2]);
				
				if(world->jointFeedbackCount<MAX_CONTACT_POINTS)
				{
					dJointSetFeedback(j,&(world->jointFeedback[world->jointFeedbackCount]));
					world->jointFeedbackCount++;
					world->contactPoints.push(c);
				}
				else
				{
					gsout<<"to many contacts num_cp:"<<num_cp<<" jointFeedbackCount: "<<world->jointFeedbackCount<<gsnl;
				}
				
				
			}
		}
		else
		{
			if(num_cp>0)
			{
				//phout<<" bad collision has occured \n";
				world->badContact(true);
			}
			
		}
	}
}

//This function is taken from the test_boxstack.cpp test file.
//void ODEWorld::PotentialHitCallback(void *data, dGeomID o1, dGeomID o2)
//{
//  int i;
//
//  // exit without doing anything if the two bodies are connected by a joint
//  dBodyID b1 = dGeomGetBody(o1);
//  dBodyID b2 = dGeomGetBody(o2);
//  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;
//
//  dContact contact[MAX_CONTACT_POINTS];   // up to MAX_CONTACTS contacts per box-box
//  for (i=0; i<MAX_CONTACT_POINTS; i++) {
//    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
//    contact[i].surface.mu = dInfinity;
//    contact[i].surface.mu2 = 0;
//    contact[i].surface.bounce = 0.1;
//    contact[i].surface.bounce_vel = 0.1;
//    contact[i].surface.soft_cfm = 0.01;
//  }
//  if (int numc = dCollide (o1,o2,MAX_CONTACT_POINTS,&contact[0].geom,
//			   sizeof(dContact))) {
//    dMatrix3 RI;
//    dRSetIdentity (RI);
//    const dReal ss[3] = {0.02,0.02,0.02};
//    for (i=0; i<numc; i++) {
//      dJointID c = dJointCreateContact (theDynamicsWorldID,theJointGroupID,contact+i);
//      dJointAttach (c,b1,b2);
//    }
//  }
//}

dWorldID ODEWorld::GetWorldID()
{
	return theDynamicsWorldID;
}

dSpaceID ODEWorld::GetSpaceID()
{
	return theCollisionSpaceID;
}

dReal ODEWorld::GetRayIntersectionDepth()
{
	return rayIntersectionDepth;
}
ODEObject* ODEWorld::getObject( dGeomID o1)
{
	for (int i=0;i<objects.size();i++)
	{
		if(objects[i]->GetGeomID() == o1)
		{
			return objects[i];
		}
	}
	phout<<"couldn't find object by dGeomID\n";
	return 0;
}
ODEObject* ODEWorld::getObject( dBodyID b1)
{
	for (int i=0;i<objects.size();i++)
	{
		if(objects[i]->getBodyID() == b1)
		{
			return objects[i];
		}
	}
	phout<<"couldn't find object by dBodyID\n";
	return 0;
}
bool ODEWorld::collides(ODEObject* obj)
{
	if(obj->isGround())
		return true;

	return obj->collides();
}

void ODEWorld::addObject( ODEObject* param1 )
{
	objects.push(param1);
}

void ODEWorld::badContact( bool val )
{
	_bad_contact = true;
}


dJointGroupID ODEWorld::getJointGroupID()
{
	return theJointGroupID;
}
