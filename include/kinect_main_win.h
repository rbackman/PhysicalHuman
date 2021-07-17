# pragma once

#include <windows.h>
#include <ole2.h>

# include <gsim/gs_vars.h>
# include <gsim/fl_vars_win.h>
#include <gsim/fl_skeleton_win.h>
# include "kinect_fluid.h"



class FileManager;
class NeuralNet;
class DataGlove;
class DataGloveWin;
class KinectManager;

class KinectMainWin : public KinectFluid
 { private :
int IMG_W;
int IMG_H;
int DIM_OUT;
GsString _buf;
GsImage* _real_hand_image;
GsImage* _synth_hand_image;
GsImage* _diff_hand_image;
FileManager* files;


NeuralNet* _neural_net;

int _record_count;


DataGlove* glove;
DataGloveWin* gloveWin;
KinectManager* kinect;
GsImage* handImage;
GsImage* reducedImage;

   public :
	 
	   FlSkeletonWin* skelWin;

	   KinectMainWin ();
	   ~KinectMainWin ();
	   void makeKinect();
	   void show ();

	   int selectedKinect();
  
   public :
	   virtual void event ( KinectEvent e );

	   void convertDatabase();

	   void updateUI();
	   void update();
	   void setKinectFromUI();
	   void setUIFromKinect();
	   void loadPointCloudFiles(GsString configname);
	   void loadConfigFiles();
	   void refreshImageView();
	   unsigned int calculateDiff( GsImage* i1, GsImage* i2, GsImage* output );
	   void saveImage();
	   bool kinectActive();
	   void propogateImage();
	   void convertToText();
};

