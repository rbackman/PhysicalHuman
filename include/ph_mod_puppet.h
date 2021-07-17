#pragma once
#include "ph_mod.h"
#include "ph_mod_manip.h"
#include "ph_manip_puppet.h"


class PuppetModule : public ManipulatorModule
{
public:

	PuppetModule(PhysicalHuman* human,const char* file);

	HumanManipulator* makeManip(GsString j,GsString file);
};
