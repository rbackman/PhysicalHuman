#pragma once
#include <gsim/og_viewer.h>
class GameWindow;

class GameViewer:public OgGSim::Viewer
{
	GameWindow* _window;
public:
	GameViewer(const char* cfgPath,GameWindow* window);
	 bool processUnbufferedKeyInput();
	 bool update();
	void init( bool askconfig );
};