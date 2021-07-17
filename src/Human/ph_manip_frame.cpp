

# include "ph_manip_frame.h"
#include "ph_mod_manip.h"


FrameManipulator::FrameManipulator (ManipulatorModule* cont, GsString file):HumanManipulator(cont,"FrameManip",cont->name()<<"FrameManip",file)
 {
	verifyParameters();
 }
void FrameManipulator::applyParameters()
{
	Manipulator::applyParameters();
	controller->setOrigin(SnManipulator::translation(),SnManipulator::rotation());
}
bool FrameManipulator::evaluate ()
 {
	Manipulator::evaluate();
	if(!controller->isActive())return false;
	if(!pBool(manipulator_active))return false;

	controller->setOrigin(SnManipulator::translation(),SnManipulator::rotation());

	 return true;
 }



