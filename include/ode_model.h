

#ifndef __ODEBOX_H__
#define __ODEBOX_H__

#include "ode_object.h"
#include "ode_world.h"
#include "ode_box.h"

#include <ode/ode.h>
#include "util_models.h"

class ODEModel : public ODEObject
{
public:
	ODEModel(ODEWorld* world, const GsString& name, const GsString& file);
	~ODEModel();
	void applyParameters();
	GsVec dimension(){return pVec(ode_box_dim);}
	

};

#endif
