# pragma once



#include "common.h"


#ifdef USE_KINECT
class KinectManager;
class KinectMainWin;
#endif


class GameWindow;
class HumanManager;
class Manipulator;

class Game
 { public :
  
	Game (GsString file);
	
	GameWindow* mainwin;
	HumanManager* manager;
	
	void update();

	#ifdef USE_KINECT
	//kinect tracking
	Manipulator* trackManip;
	bool track;
	int frameCount;

		int kinect_sample_delay;
	KinectMainWin* kinectWin;
	KinectManager* kinect;
	bool openKinect();
	#endif

	int updateCount;
	GsString processCmd( GsString cmd );
	void listCommands();
	
};



