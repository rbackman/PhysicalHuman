#include "ph_mod_template.h"


TemplateModule::TemplateModule(PhysicalHuman* human,const char* file):Module(human,"TemplateController", file)
{

}

TemplateModule::~TemplateModule(void)
{

}

bool TemplateModule::evaluate()
{
bool valid = Module::evaluate();
	

return valid;
}


