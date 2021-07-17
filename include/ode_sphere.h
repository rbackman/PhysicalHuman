

#ifndef __ODESPHERE_H__
#define __ODESPHERE_H__

#include "ode_object.h"
#include "ode_world.h"

#include <ode/ode.h>

class ODESphere : public ODEObject
{
public:
	ODESphere(ODEWorld* world, bool body, GsVec pos, dReal rad);
	~ODESphere();

	void Draw();
	dReal GetRadius();

private:
	dReal radius;

};

#endif
