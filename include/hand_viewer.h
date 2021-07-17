# pragma once

# include <gsim/fl_viewer.h>
# include <gsim/sn_polyed.h>
# include <gsim/sn_sphere.h>
#include <gsim/sn_points.h>
#include <gsim/sn_color_surf.h>
#include <gsim/sn_transform.h>

class KinectMainWin;
class Manipulator;

#include "kinect_manager.h"
#include "util_models.h"


class HandViewer : public FlViewer
 { private :
	KinectManager* _kinect;
    SnGroup*  _root;
	SnPoints* _handPoints;
	SnPoints* _realHandPoints;
	KnSkeleton* _handSkel;
	KnScene* _handScene;
	Manipulator* _handManip;
	SnGroup* _handGrp;
	SnTransform* _handTrans;
    KinectMainWin* _mainwin;

	GsImage* _image;
	int W;
	int H;
   public:
	   	bool depthMode;
    HandViewer ( int x, int y, int w, int h, const char *l=0 );
   ~HandViewer ();

   KnSkeleton* getHandSkel(){return _handSkel;}

   /*make a point cloud from the hand skeleton.. uses depth buffer in hand viewer to generate point cloud*/
   void makePointCloud(int res = 1);

    void init ( KinectMainWin* win );
  
	void deletePoints();
	void addRealHandPoints( GsArray<GsVec>* pts );

	void update();
	void setHandMat( GsMat m );
	void draw();

	virtual int handle_scene_event ( const GsEvent &e );
	bool getHandImage( GsImage* img );
};

