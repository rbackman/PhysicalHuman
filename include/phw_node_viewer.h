#pragma  once

# include <gsim/fl_viewer.h>

#include "phw_events.h"
#include "util_trajectory.h"
#include <fltk/SharedImage.h>
#include "util_serializable.h"
#include "util_channel.h"

class HumanManager;
class HumanMotionManager;
class HumanMotion;
class HumanWindow;
class Serializable;

struct temp_node
{
	Serializable* obj;
	int parmID;
	int arrID;
	GsVec2 pos;
	int channelType;
};
enum node_viewer_parms
{
	node_viewer_text_size,
	node_viewer_node_size,
	node_viewer_control_size,
	node_viewer_object_size,
	node_viewer_text_offset,
	node_viewer_text_line_offset,
	node_viewer_control_offset,
	node_viewer_control_text_offset,
	node_viewer_feedback_text_offset,
	node_viewer_background_color,
	node_viewer_node_color,
	node_viewer_object_color,
	node_viewer_text_color,
	node_viewer_temp_node_color,
	node_viewer_feedback_color,

};
typedef enum _node_edit_state
{
	node_temp_selected,
	node_connecting,
	node_removing,
	node_moving,
	node_view_moving,
	node_not_editing
}node_edit_state;
class HumanWindowNodeViewer : public FlViewer , public Serializable
 { public:
	//fltk::SharedImage* buttonImage;

	HumanWindowNodeViewer ( int x, int y, int w, int h);
	
	void init (HumanWindow* win,HumanMotionManager* mgr,const GsString& file);
	
	GsArray<Channel*> _channels;

	HumanManager* manager;
	HumanWindow* mainwin;
	GsArray<temp_node> temp_nodes;

	int _edit_index;
	node_edit_state editState;
	int input_removing;
	Channel* lastNodeSelected;

	/*!radius of influence for mouse events*/
	float pickprec;   
	/*!width and height of window in pixels*/
	int W,H;	
	/*!reference in screen coords to determine origin of graph, offset from top*/
	int Origin;
	
	
	/*most recent mouse position*/
	GsVec2 nodeOffset;
	GsVec2 mouseP;
	GsVec2 mouseStart;
	float viewScale;
	GsVec _view_translate;


	
	//some labels for the selected point
	GsString ptInfoX; 
	GsString ptInfoY;
	

	//transforms from the curve parametric [[0,1],[-1,1]] into [time,angle]
	GsVec2 curveToGrid(GsVec2 input);
	/*!screen space to scene  pixels  pos x = [0,W]=>[0,1] y=[H,0]=>[-1,1]*/
	GsVec2 win2sceneGraph ( GsVec2 input );
	/*!screen space to scene  pixels to pos x = [0,1]=>[0,W] y=[-1,1]=>[H,0]*/
	GsVec2 scene2winGraph ( GsVec2 input);


	public :
	virtual void draw ();
	virtual int handle ( int ev );

	void arrange_nodes();
	void detach_node();
	GsVec2 scaleVec( GsVec2 p );
	void drawChannel( Channel* ch );
	void pushChannels(const GsArray<Channel*>& chs);
	void initChannels();
	void addNode(Serializable* sav, int parmID,channel_dof_types dofType,int arrID);
	GsVec2 getOutputDrawPosition(GsVec2 drawP);
	GsVec2 getInputDrawPosition(GsVec2 drawP,int i = -1);
	Channel* getSelectedNode();
};


