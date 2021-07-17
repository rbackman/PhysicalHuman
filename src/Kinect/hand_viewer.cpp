
# include <gsim/fl.h>
# include <gsim/fl_vars_win.h>
# include <gsim/fl_skeleton_win.h>
#include <gsim/gs_euler.h>

# include "hand_viewer.h"
# include "kinect_main_win.h"
#include "kinect_manager.h"
#include "util_manipulator.h"

# define CHKVAR(s) if(_vars->search(s)<0) { fltk::alert("Missing parameter [%s]!",s); exit(0); }

static void manipCallback ( SnManipulator* mnp,const GsEvent& ev, void* udata )
{
	HandViewer* hv = ((HandViewer*)udata);
	Manipulator* manip = (Manipulator*)mnp;
	hv->setHandMat(manip->mat());
	//manip->evaluate();
}


HandViewer::HandViewer ( int x, int y, int w, int h, const char *l ) : FlViewer ( x, y, w, h, l )
 {
   // we build here an example scene graph for polygon edition
   _root = new SnGroup;
    _realHandPoints = new SnPoints;
   _handPoints = new SnPoints;
   W = w;
    H = h;
   _root->add(_handPoints);
   _root->add(_realHandPoints);
	_image = new GsImage;
	_image->init(w,h);
	depthMode = false;
   FlViewer::root ( _root );
 
	 _handSkel = new KnSkeleton;
	 if(! _handSkel->load("../data/models/RightHand.s"))
	 {
		 phout<<"hand load failed\n";
	 }
	 else
	 {
		 _handSkel->root()->pos()->thaw();
		 _handScene = new KnScene;
		 _handScene->connect(_handSkel);
		 _handScene->set_visibility(1,1,0,0);
		 _handGrp = new SnGroup;
		 _handGrp->separator(true);
		 _handTrans = new SnTransform;
		 _handGrp->add(_handTrans);
		 _handGrp->add(_handScene);
		  _root->add(_handGrp);
		  _handManip = new Manipulator(GsVec(0,0,0),GsVec(0.05f,0.05f	,0.05f));
		  _root->add(_handManip);
		  _handManip->callback(manipCallback,this);
	 }
	
	 view_all();

 }

HandViewer::~HandViewer ()
 {

 }
void HandViewer::init ( KinectMainWin* win )
 {
   _mainwin = win;
	 view_all();
	zoomfactor(0.03f);

}




int HandViewer::handle_scene_event ( const GsEvent &e )
 {
   // window events can be custom-processed here:
   if ( e.button1 )
    {
    }
  
   if(e.type == GsEvent::Push)
   {
	
   }
   // now let the viewer process the remaining events:
   return FlViewer::handle_scene_event ( e );
 }

void HandViewer::makePointCloud(int res)
{
	draw();
	GsArray<GsVec> points;
	GsArray<float> zbuff;
	glGetPointCloud(points,zbuff,res);
	_handPoints->init();
	for (int i=0;i<points.size();i++)
	{
		if(i%res==0)
		{
			_handPoints->push(points[i],2.0);
		}
	}
	_handScene->visible(false);
	redraw();
}

void HandViewer::deletePoints()
{
	_handScene->visible(true);
	_handPoints->init();
	redraw();
}

void HandViewer::addRealHandPoints( GsArray<GsVec>* pts )
{
	phout<<"adding "<<pts->size()<<" points\n";
	_realHandPoints->init();
	for (int i=0;i<pts->size();i++)
	{
		_realHandPoints->push(pts->get(i)*50,GsColor::red,2.0);
	}
	
}




void HandViewer::update()
{
	_handScene->update();
}

void HandViewer::setHandMat( GsMat m )
{
	_handTrans->set(m);
}

bool HandViewer::getHandImage( GsImage* img )
{

	img->init(_image->w(),_image->h());
	img->buffer() = _image->buffer();
	/*
	for (int i=0;i<img->w();i++)
	{
		for(int j=0;j<img->h();j++)
		{
			int myImgX = (int)(i*((float)W)/(float)img->w());
			int myImgY = (int)(j*((float)H)/(float)img->h());
			GsColor* c =_image->ptpixel(myImgX,myImgY);
			img->ptpixel(i,j)->set(c->r,c->g,c->b);
		}
	}*/

	return true;
}

void HandViewer::draw()
{
	FlViewer::draw();
			

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

	_image->init(pt_width,pt_height);

	for (int i=min_x;i<max_x;i++)
	{
		for(int j=min_y;j<max_y;j++)
		{
			zdist =  zbuffer[j*W+i];

			if(zdist>0 && zdist<1)
			{
				len = 1.0f - (zdist-min_z)/(z_dist);
				_image->ptpixel(j-min_y,i-min_x)->set(len,len,len);
			}
			else
				_image->ptpixel(j-min_y,i-min_x)->set(0,0,0);
		}
	}

	if(depthMode)
	{
		glDrawPixels(_image->w(),_image->h(),GL_RGBA,GL_UNSIGNED_BYTE, &_image->r(0,0) );
	}
}


