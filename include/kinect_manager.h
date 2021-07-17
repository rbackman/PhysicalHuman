# pragma once

#include <gsim/gs_vec.h>
#include <gsim/gs_array.h>


#include "util_models.h"
#include "util.h"
#include "kinect.h"

class KinectMainWin;
class KinectMarker;
class SnPoints;
class Manipulator;
class PointCloud;

#include <gsim/gs_color.h>
#include "util_serializable.h"


enum kinect_manager_parms
{
	kinect_manager_marker_types, /*strings: names of kinds of markers, Hand,Index..etc*/
	kinect_manager_marker_list,  /*list of current markers*/
	kinect_manager_marker_max_v, /* maximum velocity a finger can move in one frame*/
	kinect_manager_marker_sample, /*oversampling for marker position*/
	kinect_manager_marker_color_variance, /*int range of color distance to consider for markers*/
	kinect_manager_use_color, /*bool: use color stream to create mesh*/
	kinect_manager_use_depth, /*bool: use depth stream to create mesh*/
	kinect_manager_use_all_points, /*bool: force all points to be considered for debuging*/
	kinect_manager_track_colors, /*bool: use markers*/
	kinect_manager_draw_markers,
	kinect_manager_draw_mesh, /*bool: update SnColorSurface*/
	kinect_manager_draw_output, /*bool: show final pointcloud after processing*/
	
	kinect_manager_use_clip_box, /*bool only consider points in a box*/
	kinect_manager_clip_box_visible, /*bool show clip manip*/
	kinect_manager_clip_box_size, /*float size of clip box*/
	kinect_manager_clip_box_position, /*vec position of clip box*/
	kinect_manager_hand_color_variance, /*int color variance for hand*/
	kinect_manager_hand_color_proximity, /*bool use color variance or not*/

	kinect_manager_hand_proximity, /*bool: limit pointcloud processing to near the hand marker*/
	kinect_manager_hand_dist, /*float max distance from hand to consider*/
	kinect_manager_range, 
	kinect_manager_cutoff,
	kinect_manager_angle,
	
};

class KinectManager : public Serializable
 { 
   public :
    KinectManager ();
   ~KinectManager();
     void update();
	 bool captureHandCloud(float val);

	 GsBox clipBox;

 private:
   //markers
	 int min_x;
	 int min_y;
	 int max_x;
	 int max_y;

	 //temp vars for pruning
	 int _id;
	 int _min_x;
	 int _min_y;
	 int _max_x;
	 int _max_y;

	point3D points[NUM_POINTS];
	GsImage* _image;

   GsArray<KinectMarker*> _markers;
   KinectMarker* _hand;
   KinectMarker* _currentMarker;

   //fingers
   GsVec _fingerStartVec;
   GsVec _fingerStartPos;
   float _fingerStartDist;

   //clouds
   GsArray<Kinect*> _kinects;
   GsArray<PointCloud*> _clouds;
   GsArray<GsImage*> _images;

   int _currentCloudID;
   SnPoints* _selectedCloudPoints;
 
   SnPoints* _currentCloudPoints;
   SnBox* _clipBoxVis;
   //SnPoints* _
   GsArray<GsPnt> _handPoints;
  
   Manipulator* _clipManip;
   
   //kinect
	int _numKinects;
	int _currentKinectID;

	SnGroup* _grp;
	SnGroup* _marker_grp;

	void prunePoints( GsVec pos,float rad );
	void prunePoints(float min,float max );
	void prunePoints(GsColor col,int var);

	
	void setPointActive( int x, int y, bool p );

	bool _new_data;
 public:	
	 bool getPointActivity( int i );

		 point3D* getPoints(){return &points[0];}
		int init();
		void applyParameters(); 
		bool hasNewData();
		/*for display*/
		SnGroup* getGroup(){return _grp;}
	
		/*kinect handling*/
		Kinect* getKinect(int i){return _kinects[i];}
		Kinect* currentKinect(){return getKinect(_currentKinectID);}
		int getNumKinects(){return _numKinects;}

		/*point cloud access*/
		GsArray<GsPnt>*  getHandPoints(){return &_handPoints;}
		
		PointCloud* currentCloud(){return _clouds[_currentCloudID];}
		GsImage* currentImage(){return _images[_currentCloudID];}
		
		void selectCloud( int cld );
		void findClosestCloud();
		void saveHandCloud(GsString filename);
		float minDepth();
		float maxDepth();
		float cutoff();

		/*marker handling */
		void makeMarker(GsString nme, GsColor cl, GsVec pos );
		KinectMarker* getMarker(GsString name);
		KinectMarker* getMarker( int i );
		KinectMarker* getLastMarker(){if(_markers.size()==0)return 0; else return _markers.top();}
		KinectMarker* getCurrentMarker(){return _currentMarker;}
		int numMarkers(){return _markers.size();}
		int numClouds(){return _clouds.size();}
		void saveCloud(int i);
		void clearMarkers();
		void deleteLastMarker();
		void selectMarker( GsString selected );

		/*marker tracking*/
		void calibrateFingers(); //make current finger position/orientation the origin
		GsQuat getOrientation(); //orientation based on finger calibration
		float getFingerVal(int i); //distance based on finer calibration
		GsVec getFingerPosition(); //position of finger without calibration
		GsVec getLocalMarkerPosition();  //positon based on calibration	
		void initClouds();
		void loadCloudFromFile( GsString files );
		void setClipBox( GsVec pos );
		void setClipManip( GsVec pos );
		void load(GsString filename);
		bool getHandImage(GsImage* img);
		bool getCloudImage( GsImage* img );
		void prunePointsFromClipBox();
		void startPrune();
		void endPrune();
		void validPoint(int i,int j);
		void prunePointsFromActivity();
	
		
};



/*
#define IDC_MYICON                      2
#define IDD_SKELETALVIEWER_DIALOG       102
#define IDS_APP_TITLE                   103
#define IDD_ABOUTBOX                    103
#define IDM_ABOUT                       104
#define IDM_EXIT                        105
#define IDI_SKELETALVIEWER              107
#define IDC_SKELETALVIEWER              109
#define IDD_APP                         110
#define IDR_MAINFRAME                   128
#define IDS_APPTITLE                    129
#define IDS_ERROR_D3DCREATE             130
#define IDS_ERROR_D3DVIDEOTYPE          131
#define IDS_ERROR_NUIINIT               132
#define IDS_ERROR_SKELETONTRACKING      133
#define IDS_ERROR_DEPTHSTREAM           134
#define IDS_STRING135                   135
#define IDS_ERROR_VIDEOSTREAM           135
#define IDC_DEPTHVIEWER                 1001
#define IDC_SKELETALVIEW                1002
#define IDC_VIDEOVIEW                   1003
#define IDC_FPS                         1004
#define IDC_STATIC                      -1
*/