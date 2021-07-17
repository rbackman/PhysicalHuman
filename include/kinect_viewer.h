# pragma once

#include <gsim/fl_viewer.h>
#include <gsim/sn_polyed.h>
#include <gsim/sn_sphere.h>
#include <gsim/sn_points.h>
#include <gsim/sn_color_surf.h>
#include <gsim/sn_transform.h>

class KinectMainWin;

#include "kinect_manager.h"
#include "util_models.h"


class KinectViewer : public FlViewer
{ 
private :
	KinectManager* _kinectManager;
	SnGroup*  _root;
	SnGroup*  _markerGroup;
	
	SnPoints* _markerPoints;
	GsArray<SnColorSurf*> _surfs;
	GsArray<SnTransform*> _surf_trans;
	int _meshNum;
	KinectMainWin* _mainwin;

public:

	SnGroup* _surfGroup;
	KinectViewer ( int x, int y, int w, int h, const char *l=0 );
	~KinectViewer ();

	void init ( KinectMainWin* win );
	void setKinectManager(KinectManager* k);
	
	/*change the given transform for a surface. used to align two kinects*/
	void setTrans(int k,float x,float y,float z,float rx,float ry,float rz);

	/*use given point cloud to create SnColorSurf*/
	void updatePoints(int i, point3D * points);

	/*check if two vertices are within tolerance*/
	bool mdist(int a,int b,int surf);

	virtual int handle_scene_event ( const GsEvent &e );
	SnGroup* getRoot();
};

