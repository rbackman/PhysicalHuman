
# include <gsim/fl.h>
# include <gsim/fl_vars_win.h>

#include <gsim/sn_group.h>
#include <gsim/sn_polyed.h>


# include "phw_viewer.h"
# include "phw_window.h"
#include "ph_manager.h"
#include "util_serializable.h"
#include "ph_human.h"
#include "util_manipulator.h"

# define CHKVAR(s) if(_vars->search(s)<0) { fltk::alert("Missing parameter [%s]!",s); exit(0); }


static FlPopupMenu::Desc ManipMenutems [] = 
{
	{ "&edit", AppViewer::CmdEdit,  FlPopupMenu::Normal, 0 },
	{ "&active", -1, FlPopupMenu::Submenu, 0 },
	{ "&active", AppViewer::CmdActivate, FlPopupMenu::Radio, 1 },
	{ "&inactive",   AppViewer::CmdDeactivate,   FlPopupMenu::Radio, 0 },
	{ 0, -1, FlPopupMenu::Normal, 0 },
	{ 0, -1, FlPopupMenu::Radio, 0 },
};


AppViewer::AppViewer ( int x, int y, int w, int h, const char *l ) : FlViewer ( x, y, w, h, l ),Serializable("AppViewer")
 {
	

	  zoomfactor(0.04f);

	 // we build here an example scene graph for polygon edition
		 _root = new SnGroup;
		

		

		view_all();
		manipMenu =  0; //new FlManipulatorPopupMenu(this);
		 currentManip = 0;;

	  // FlViewer::cmd ( FlViewer::CmdPlanar );
	   FlViewer::root ( _root );
 }
void AppViewer::makeManipMenu(GsVec2 p,Manipulator* manip)
{
	if(manipMenu)
	{
		manipMenu->position((int)p.x,(int)p.y);
		currentManip = manip;
		manipMenu->popup();
	}

}
void AppViewer::applyParameters()
{
	camera().init();

	camera().fovy = pFloat(camera_fovy);
	camera().eye	=  pVec(camera_eye);
	camera().center	=  pVec(camera_center);
	camera().up =  pVec(camera_up);
	camera().znear = pFloat(camera_znear);
	camera().zfar = pFloat(camera_zfar);
	camera().aspect = pFloat(camera_aspect);
	camera().scale = pFloat(camera_scale);
	camera().rotation = pQuat(camera_rotation);
	camera().translation = pVec(camera_translation);

	redraw();


}
AppViewer::~AppViewer ()
 {

 }

void AppViewer::init ( HumanWindow* win ,GsString file)
 {
	loadParametersFromFile(file);

	CHECK_VEC(camera_eye);
	CHECK_VEC(camera_center);
	CHECK_VEC(camera_up);
	CHECK_FLOAT(camera_fovy);
	CHECK_FLOAT(camera_znear);
	CHECK_FLOAT(camera_zfar);
	CHECK_FLOAT(camera_aspect);
	CHECK_FLOAT(camera_scale);
	CHECK_QUAT(camera_rotation);
	CHECK_VEC(camera_translation);
	CHECK_BOOL(camera_follow);

   _mainwin = win;

   applyParameters();
   
 }
void AppViewer::getDepthSnapShot(GsImage* img)
{
	//	draw();
		int vp[4];

		glGetIntegerv ( GL_VIEWPORT, vp );

		int x = vp[0];
		int y = vp[1];
		int W = vp[2];//-x;
		int H = vp[3];//-y;

		GsBuffer<float> zbuffer;
		zbuffer.size(W*H);
		int max_x = 0;
		int min_x = W;
		int max_y = 0;
		int min_y = H;
		float min_z = 10000;
		float max_z = 0;
		float len = 0;
		float zdist;

		glReadPixels ( 0, 0, W,H, GL_DEPTH_COMPONENT, GL_FLOAT, (void*)&zbuffer[0] );

		for ( int i=0; i<W; i++ )
		{ 
			for (int j=0; j<H; j++ )
			{ 
				zdist =  zbuffer[j*W+i];
				if ( zdist>0 && zdist<1 )
				{ 
					if(i<min_x) min_x = i;
					if(i>max_x) max_x = i;
					if(j<min_y) min_y = j;
					if(j>max_y) max_y = j;

					if(zdist > max_z) max_z = zdist;
					if(zdist < min_z) min_z = zdist;
				}
		
			}
		}

		if(min_y > max_y || min_x > max_x)
		{
			phout<<"didn't find any active points";
			return;
		}
		if(min_x<0)min_x=0;
		if(min_y<0)min_y=0;
		float z_dist = max_z - min_z;
		int pt_width = max_x - min_x;
		int pt_height = max_y - min_y;

		img->init(pt_width,pt_height);

		for (int i=min_x;i<max_x;i++)
		{
			for(int j=min_y;j<max_y;j++)
			{
				zdist =  zbuffer[j*W+i];

				if(zdist>0 && zdist<1)
				{
					len = 1.0f - (zdist-min_z)/(z_dist);
					img->ptpixel(j-min_y,i-min_x)->set(len,len,len);
				}
				else
					img->ptpixel(j-min_y,i-min_x)->set(0,0,0);
			}
		}

}
void AppViewer::draw ()
 {
	
	
		setP(camera_fovy,camera().fovy);
		setP(camera_eye,camera().eye);
		setP(camera_center,camera().center);
		setP(camera_scale,camera().scale);
		setP(camera_up,camera().up);
		setP(camera_znear,camera().znear);
		setP(camera_zfar,camera().zfar);
		setP(camera_aspect,camera().aspect);
		setP(camera_rotation,camera().rotation);
		if(pBool(camera_follow))
		{

		}
		else
		{
			setP(camera_translation,camera().translation);
		}

   FlViewer::draw ();
 }


int AppViewer::handle_scene_event ( const GsEvent &e )
 {

	 setP(camera_scale,camera().scale);
	 setP(camera_rotation,camera().rotation);
	 setP(camera_translation,camera().translation);
	

			 _mainwin->handle_viewer_event(e);
			 
	
   // now let the viewer process the remaining events:
   return FlViewer::handle_scene_event ( e );
 }



void AppViewer::manip_cmd( ManipMenuCmd c )
{
	if(currentManip==0)return;
	switch(c)
	{
	case CmdActivate: currentManip->setP(manipulator_active,true); phout<<"edit "<<gsnl; break;
	case CmdDeactivate: currentManip->setP(manipulator_active,false); phout<<"deactivate "<<gsnl; break;
	case CmdEdit: _mainwin->loadParameterEditor(currentManip); phout<<"edit "<<gsnl; break;
	}
	currentManip = 0;
}


FlManipulatorPopupMenu::FlManipulatorPopupMenu( AppViewer* v ) :FlPopupMenu ( v->w(), v->h(), ManipMenutems )
{
	viewer=v;
	itemselected=false;
}

void FlManipulatorPopupMenu::selected( int id )
{
	itemselected=true;
	viewer->manip_cmd ( (AppViewer::ManipMenuCmd)ManipMenutems[id].cmd );
}
