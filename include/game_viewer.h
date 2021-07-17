# pragma once

# include <gsim/fl_viewer.h>
#include <gsim/kn_scene.h>
#include <gsim/fl_popup_menu.h>

enum app_viewer_parms
{
	camera_fovy,
	camera_eye,
	camera_center,
	camera_up,
	camera_znear,
	camera_zfar,
	camera_aspect,
	camera_scale,
	camera_rotation,
	camera_translation,
	camera_follow
};



class GameWindow;
class GameFLViewer;
class SnGroup;
class Manipulator;

#include "util_serializable.h"


class GameFLViewer : public FlViewer , public Serializable
 { 
 public :	
    SnGroup*  _root;
	SnGroup* _lines;

	GameWindow* _mainwin;

	void applyParameters();


   public:
    GameFLViewer ( int x, int y, int w, int h, const char *l=0 );
   ~GameFLViewer ();
    void init ( GameWindow* win ,GsString file);

    virtual void draw ();

    // several event handlers can be re-writen, below is on example; see the
    // base class header file for all possible event methods to be re-written
    virtual int handle_scene_event ( const GsEvent &e );
};
