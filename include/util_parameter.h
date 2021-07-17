
#include "common.h"

enum parameter_types
{
	INT_PARM,
	BOOL_PARM,
	VEC_PARM,
	COLOR_PARM,
	FLOAT_PARM,
	STRING_PARM,
	QUAT_PARM
};
class ControlParameter
{
public:
	parameter_types type;
	GsString name;

	GsString getString();
	bool save;
#ifdef VERIFY_PARAMETERS
	bool checked;
	void set( ControlParameter* otherP );
	ControlParameter(){checked=false; save = true;}
	ControlParameter( ControlParameter* otherP){checked=true; save = otherP->save; set(otherP);}
#endif

};

class QuatParameter : public ControlParameter
{public:
GsQuat val;
rotation_type rotationOrder;
QuatParameter(){type = QUAT_PARM;rotationOrder = ROT_XYZ;}
};
class ColorParameter : public ControlParameter
{public:
GsColor val;
ColorParameter(){type = COLOR_PARM;}
};
class VecParameter : public ControlParameter
{public:
GsVec val;
VecParameter(){type = VEC_PARM;}
};

class FloatParameter : public ControlParameter
{public:
GsArray<float> val;
FloatParameter(){type = FLOAT_PARM;}
};
class BoolParameter : public ControlParameter
{public:
bool val;
BoolParameter(){type = BOOL_PARM;}
};
class IntParameter : public ControlParameter
{public:
GsArray<int> val;
IntParameter(){type = INT_PARM;}
};
class StringParameter : public ControlParameter
{public:
GsStrings val;
StringParameter(){type = STRING_PARM;}
~StringParameter();
void setStrings(GsVar* v);
void init()
{
	val.size(0);
}
void replace( GsString old, GsString s );
bool add( GsString name );
void insert( int idx, GsString st );
};