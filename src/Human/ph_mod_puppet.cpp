#include "ph_mod_puppet.h"
#include "ph_mod_ref.h"

PuppetModule::PuppetModule(PhysicalHuman* human,const char* file):ManipulatorModule(human,"PuppetModule",file)
{
	_shortName = "puppet";
	verifyParameters();
}
HumanManipulator* PuppetModule::makeManip(GsString jointName,GsString file)
{
	PhysicalJoint* j = h->joint(jointName);
	if(j)
	{
		SpringManip* manip = new SpringManip(this,j,file);
		return (HumanManipulator*)manip;
	}
	return 0;
}




