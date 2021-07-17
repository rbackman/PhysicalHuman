#pragma  once

# include <gsim/fl_viewer.h>

#include "phw_events.h"
#include "util_trajectory.h"
#include <fltk/SharedImage.h>
#include "util_serializable.h"

class Channel;
class HumanManager;
class HumanMotionManager;
class HumanMotion;
class HumanWindow;

enum graph_view_parms
{
	graph_viewer_draw_grid,
	graph_viewer_background_color,
	graph_viewer_default_curve_color,
	graph_viewer_default_curve_width,
	graph_viewer_default_point_width,
	graph_viewer_sample_width,
	graph_viewer_draw_bounds
};

class HumanWindowGraphViewer : public FlViewer , public Serializable
 { public:
	fltk::SharedImage* buttonImage;

	HumanWindowGraphViewer ( int x, int y, int w, int h);
	
	void init (HumanWindow* win,HumanMotionManager* mgr,GsString file);
	

	HumanManager* manager;
	HumanWindow* mainwin;
	



	/*!radius of influence for mouse events*/
	float pickprec;   
	/*!width and height of window in pixels*/
	int W,H;	
	/*!reference in screen coords to determine origin of graph, offset from top*/
	int Origin;
	
	/*!pixel size of grid*/
	int gridW;
	int gridH;
	void drawGrid();

	/*most recent mouse position*/
	GsVec2 mouseP;
	GsVec2 mouseStart;
	float viewScale;
	GsVec _view_translate;
	bool _moving;

	curve_state _edit_mode;
	
	//some labels for the selected point
	GsString ptInfoX; 
	GsString ptInfoY;
	
	//transforms from the curve parametric [[0,1],[-1,1]] into [time,angle]
	GsVec2 curveToGrid(GsVec2 input);
	/*!screen space to scene  pixels  pos x = [0,W]=>[0,1] y=[H,0]=>[-1,1]*/
	GsVec2 win2sceneGraph ( GsVec2 input );
	/*!screen space to scene  pixels to pos x = [0,1]=>[0,W] y=[-1,1]=>[H,0]*/
	GsVec2 scene2winGraph ( GsVec2 input);

	void setEditMode(curve_state s){_edit_mode = s;}
	public :
	virtual void draw ();
	virtual int handle ( int ev );
	void overrideCurveLook();

 };


