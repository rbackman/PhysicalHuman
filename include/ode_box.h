
#pragma once

#include "ode_object.h"
#include "ode_world.h"


class ODEBox : public ODEObject
{
public:
	ODEBox(ODEWorld* world, bool body, GsVec pos, GsVec size);
	
	ODEBox(ODEWorld* world, const GsString& name, const GsString& file);

	~ODEBox();
	void applyParameters();
	GsVec dimension(){return pVec(ode_box_dim);}
	void init();
};


