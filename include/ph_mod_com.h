#pragma once
#include "ph_mod.h"

/*
This module simply keeps track of the COM and the desired com so it doesn't do a whole lot.. so 
its not really a controller..
*/

class COMModule : public Module
{
public:
	COMModule(PhysicalHuman* human,const char* file);
	~COMModule(void);
	bool evaluate();
	
	inline GsVec getCOM(){return pVec( com_pos);}
	GsVec getCOMVelocity(){return pVec(com_velocity);}
	inline GsVec getCOMProjection(){return pVec(com_proj_pos);}
	inline GsVec getDesiredCOM(){return pVec( com_desired_pos);}
	void setDesiredCOM(GsVec p){setP(com_desired_pos,p);}

	GsVec getSupportVector();
	GsVec getDesiredCOMFromSupportVec();
	GsString stateString();
	void applyParameters();

private:
	void init();

	GsVec computeCOMVelocity();
	GsVec computeCOM();
	GsVec computeSupportVector();
	GsVec getDesiredSupportVector();

	Ball* com_model;
	Ball* com_d_model;
	Ball* com_proj_model;
	Ball* com_d_proj_model;
	Ball* com_d_support_vector_model;
	Line* support_line;
};
