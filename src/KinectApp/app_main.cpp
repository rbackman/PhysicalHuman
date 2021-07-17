#include "kinect_manager.h"
#include "kinect_main_win.h"
#include "util_server.h"
# include <gsim/fl.h>
#include "kinect_marker.h"


int main ( int argc, char** argv )
{
	//Server* server = new Server;
	KinectMainWin* kinectWin = new KinectMainWin;
	
	kinectWin->show();

	//server->startServer();

	while(true)
	{
		kinectWin->update();

		//kinect->currentKinect()->unlock();
		fltk::check();
	}
	return 0;
}


