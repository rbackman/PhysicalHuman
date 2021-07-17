# pragma once



#include "common.h"

#ifdef USE_KINECT
class KinectManager;
class KinectMainWin;
#endif

class KnMotion;
class Manipulator;
class HumanWindow;
class HumanManager;

class AppMain 
 { public :
  
	AppMain (const GsString& file);
	
	HumanWindow* mainwin;
	HumanManager* manager;
	
	void update();


	//kinect tracking
	Manipulator* trackManip;
	


	int updateCount;
	void savePrefs();
	void applyParameters();
	GsString processCmd( const GsString& cmd );
	void listCommands();
	bool foundMouse;
};



