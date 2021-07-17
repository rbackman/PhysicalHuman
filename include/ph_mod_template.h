#pragma once
#include "ph_mod.h"

//this is a template that makes a good starting place for a new controller.. copy this

class TemplateModule : public Module
{
public:
	TemplateModule(PhysicalHuman* human,const char* file);
	~TemplateModule(void);
	bool evaluate();
private:
//stuff
	
};
