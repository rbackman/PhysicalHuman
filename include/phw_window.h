# pragma once

#include "common.h"
#include "phw_window_fluid.h"
#include "util_serializable.h"
#include "phw_events.h"

class HumanManager;
class SerializableEditor;
class PhysicalHuman;
class Model;
class PhysicalJoint;
class HumanManipulator;
class SnPoints;

class HumanWindow : public HumanWindowFluid ,public Serializable
 { 
 private :
	 GsString _cmd;
	 bool cmdAvailable;
	  GsString _buf;
	  SnPoints* _point_cloud;
	public :
    HumanWindow (HumanManager* mgr, const GsString& file);
   ~HumanWindow ();
    virtual void event ( HumanWindowEvent e );
   
   HumanManager* manager;

   GsArray<SerializableEditor*> editors;

   GsArray<PhysicalJoint*> selectedJoints;
   GsArray<int>  selectedJointIdx;

    void show ();
	void update();
	bool hasCmd(){return cmdAvailable;}
	GsString getCmd()
	{
		if(cmdAvailable)
		{
			cmdAvailable = false;
			return _cmd;
		}
		return "none";
	}

	void loadParameterEditor(Serializable* sav, const GsString& label = " ");
	void loadNodeParameterEditor(Serializable* sav,const GsString& lab);

     
    GsArray<PhysicalJoint*> jointsSelected();

	void loadSceneFiles();
	void loadMotionFiles();
	void loadStateFiles();
		void loadCharacterList();
	
	void loadConfigurationNamed(const GsString& configName,bool forcereload = false);

	void parmEdited();
	void refreshScene(); //probably a bad way to control if the scene is rendered shaded or wireframe
	
	void jointListModified(); //set the joint values based on changes of the UI
	void jointListSelected(); //fill the UI with info on the current selected joint
	void applyEulerAngles();

	GsString selectedState();
	GsString selectedScene();
	GsString selectedController();
	GsString selectedConfig();

		void checkRay(GsLine ray);
		
		void deleteCurves();

		/*!called when channel list is selected*/
		void channelSel();
		/*!called when motion list is selected*/
		GsString motionSelected();
		/*!returns the first channel selected*/
		GsString getSelectedChannel();
		GsArray<Channel*>* getSelectedChannels();

		/*!check to see if joint is selected in graph viewer*/
		bool channelSelected(const char* name);

   public :
   
	HumanManipulator* selectedManipulator();
	void handle_viewer_event( const GsEvent & e );
	void loadPopUpMenu(Manipulator* sav,GsVec2 p);
	void loadControllers();
	void message ( const char* s ) { ui_message->value(s); gsout<<"message:"<<s<<gsnl; }
	void message ( const char* s, int i ) { _buf.setf(s,i); ui_message->value(_buf); }
	void message ( const char* s, float f ) { _buf.setf(s,f); ui_message->value(_buf);}
	void message ( const GsString& s, const GsString& f ) {GsString msg = s; msg<<f; message(msg);}
	void setUIFromCharacter( );
	GsString knMotionSelected();
	void refreshChannelList();
	void segmentMotions();
	void viewCharacter();
	void updateTimeUI();
	void updateSampleUI();
	void depthSnapshot( GsImage& img );
	
	void makePointCloud();
	void selectChannel(const GsString& chName);
	void showErrorWin(const char* error);
	void loadScene( GsString scene );
};

