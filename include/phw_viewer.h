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



class HumanWindow;
class AppViewer;
class SnGroup;
class Manipulator;

#include "util_serializable.h"


class FlManipulatorPopupMenu : public FlPopupMenu
{ public :
AppViewer* viewer;
Manipulator* manip;
bool itemselected;
FlManipulatorPopupMenu ( AppViewer* v);
virtual void selected ( int id );
};


class AppViewer : public FlViewer , public Serializable
 { public :
	
    SnGroup*  _root;
	enum ManipMenuCmd 
	{ 
		CmdActivate,
		CmdDeactivate,
		CmdEdit
	};
	void applyParameters();
	
	
	
	SnGroup* _lines;
    HumanWindow* _mainwin;
	FlManipulatorPopupMenu* manipMenu;
	Manipulator* currentManip;
   public:
    AppViewer ( int x, int y, int w, int h, const char *l=0 );
   ~AppViewer ();
    void init ( HumanWindow* win ,GsString file);
	void getDepthSnapShot(GsImage* img);
	void manip_cmd(ManipMenuCmd c);
	void makeManipMenu(GsVec2 p,Manipulator* manip);

    virtual void draw ();

    // several event handlers can be re-writen, below is on example; see the
    // base class header file for all possible event methods to be re-written
    virtual int handle_scene_event ( const GsEvent &e );
};
