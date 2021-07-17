#pragma once

#include "ph_manip.h"
#include "util_models.h"

/*
A manipulator for controlling the reference frame or origin of a controller. 
the controller pointer must reference a ManipulatorController
*/
class FrameManipulator : public HumanManipulator
 { public :
  
  FrameManipulator (ManipulatorModule* ikc, GsString file);
	void applyParameters();
    bool evaluate ();
 };






