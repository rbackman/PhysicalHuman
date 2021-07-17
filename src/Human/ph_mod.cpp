#include "ph_mod.h"

Module::~Module()
{
	grp->remove_all();
	grp->unref();
}
Module::Module(PhysicalHuman* human,GsString cname,const char* file):Serializable(cname,"Controller")
{
	loadParametersFromFile(file);
	
	CHECK_BOOL(module_active);
	CHECK_BOOL(module_visible);
	h=human;
	manager = h->getManager();

	grp = new SnGroup; grp->ref();
	grp->separator(true);
	visible(pBool(module_visible));
	_manip_controller=false;
}
GsString Module::stateString()
{
	return GsString("");
}
void Module::loadStateFromFile(GsString file)
{
	phout<<name()<<"doesn't implement load state from file\n";
}
GsArray<Serializable*> Module::getSerializables()
{
	GsArray<Serializable*> sav;
	sav.push(this);
	return sav;
}

void Module::applyParameters()
{
	Serializable::applyParameters();
	visible(pBool(module_visible));
}
void Module::visible(bool v)
{
	setP(module_visible,v);
	grp->visible(v);
}
void Module::init()
{

}
bool Module::evaluate()
{
	if(!isActive())return false;
	return true;
}