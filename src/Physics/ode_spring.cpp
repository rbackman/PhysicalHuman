//ODEBox.cpp

#include "ode_spring.h"
#include "ode_object.h"
#include "ode_world.h"
#include "util.h"
#include "util_models.h"

#include <math.h>


ODESpring::ODESpring(ODEWorld* world, ODEObject *rb1,GsVec anchor)
{
		b1 = rb1;
		b2 = 0;
		p2 = anchor;
		this->world = world;
		lin_k = 100.0f;
		lin_d = 20.0f;
		rot_k = 100.0f;
		rot_d = 20.0f;
		rest_length = 0.0f;
		use_orientation = true;
		break_length = 10.0f;
		active = true;
		p1Model = new Ball(b1->getPosition(),0.01f);
		p2Model = new Ball(p2,0.01f);
		p1 = GsVec();
		p2 = GsVec();
		grp = new SnGroup;
		grp->add(p1Model->getGrp());
		grp->add(p2Model->getGrp());
		line = new Line(b1->getPosition(),p2);
		grp->add(line->getGrp());
}

ODESpring::ODESpring(ODEWorld* world, ODEObject *rb1, ODEObject* rb2)
{
	b1 = rb1;
	b2 = rb2;
	this->world = world;
	active = true;
	lin_k = 100.0f;
	lin_d = 20.0f;
	rot_k = 100.0f;
	rot_d = 20.0f;
		rest_length = 0.0f;
		use_orientation = true;
break_length = 10.0f;

	p1Model = new Ball(b1->getPosition(),0.01f);
	p2Model = new Ball(b2->getPosition(),0.01f);
	p1 = GsVec();
	p2 = GsVec();
	grp = new SnGroup;
	grp->add(p1Model->getGrp());
	grp->add(p2Model->getGrp());
	line = new Line(b1->getPosition(),b2->getPosition());
	grp->add(line->getGrp());

}

bool ODESpring::update()
{
	

	GsVec pa =  b1->getPosition()+ b1->getOrientation().apply(p1);
	
	GsVec pb;
	if(b2 !=0 )
	{
		pb = b2->getPosition()+ b2->getOrientation().apply(p2);
	}
	else
	{
		pb = p2;
	}
	p1Model->setPosition(pa);
	p2Model->setPosition(pb);
	
	line->setPoints(p1,pb);

	if(!active)return false;


	GsVec stretch =  pb - pa;

	force = pb - pa; 
	float ext = (stretch.len()-rest_length);
	force.len(lin_k*ext);
	
	b1->addForce(pa,force);

	if(b2!=0)
		b2->addForce(pb,-force);
	
	b1->addForce(pa,-b1->getVelocity()*lin_d);

	if(b2!=0)
		b2->addForce(pb,-b2->getVelocity()*lin_d);
		

	if(use_orientation)
	{
		GsVec trq;
		if(b2)
		{
			orientation_desired = b2->getOrientation();
		}
		
		torque = computePDTorque(world->getSimStep(), b1->getOrientation(),orientation_desired, b1->getRotationalVelocity(),GsVec(),rot_k,rot_d);
			
		b1->addGlobalTorque(torque);
		if(b2)
		{
			b2->addGlobalTorque(computePDTorque(world->getSimStep(), b1->getOrientation(),orientation_desired, b2->getRotationalVelocity(),GsVec(),rot_k,rot_d));
		}
	}


	
	
	return true;

}




	

	


