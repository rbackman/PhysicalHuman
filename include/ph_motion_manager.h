#pragma once
#include "common.h"
#include "util_trajectory.h"
#include "util_serializable.h"
#include "util_channel.h"
#include "ph_controller.h"
class HumanManager;
class HumanMotion;
class Channel;
class VecTrajectory;
class Motion;
class HumanState;
class PhysicalHuman;
class Line;
class TrajectoryChannel;
class Controller;
class Manipulator;
class Box;
enum human_motion_manager_parms
{
	human_motion_manager_playing,
	human_motion_manager_time,
	human_motion_manager_loops,
	human_motion_manager_resets, //reset to the initial state when a motion is selected
	human_motion_manager_default_control_file,

	human_motion_manager_interactive_mode,
	human_motion_manager_show_original_motion,
	human_motion_manager_show_replay,
	human_motion_manager_step_length,
	human_motion_manager_expand_dist,
	human_motion_manager_use_kn_motion_composite,
	human_motion_manager_timewarp_file,
	human_motion_manager_cmu_skel,
	human_motion_manager_draw_snapshots,
	human_motion_manager_snapshot_increment,
	human_motion_manager_start_time,
	human_motion_manager_end_time,
	human_motion_manager_wait_for_stance,
	human_motion_manager_update_when_not_playing,
	human_motion_manager_rbds_max_neighbors,
	human_motion_manager_rbds_support,
	human_motion_manager_load_env,
	human_motion_manager_relative_jump,
	human_motion_manager_edit_motions,
	human_motion_manager_expand_tries,
	human_motion_manager_edit_mode
};
enum skeleton_choice
{
	no_skeleton,
	cmu_skeleton,
	dynoman_skeleton,
};

struct motion_state
{
	HumanState* state;
	float time;
	bool save;
};

class SnapShotVis
{
public:
	GsArray<Box*> boxes;
	SnGroup* g;
	SnapShotVis(PhysicalHuman* h);
	void set(HumanState* s);
};

class HumanMotionManager : public Serializable
{
private:
	//in interactive mode this is where to stop
	float _interactive_time;
	bool _curentIsLeft;

	SnGroup* _motion_lines;

	int _expand_tries;

	HumanManager* manager;
	PhysicalHuman* human;
	GsArray<Controller*> _controllers;
	int _current_controller;
	float _phase;

	skeleton_choice _current_skeleton;
	KnSkeleton* _cmu_kinematic_skeleton;
	KnScene* _cmu_kinematic_scene;
	float _cmu_scale;

	KnSkeleton* _dynoman_kinematic_skeleton;
	KnScene* _dynoman_kinematic_scene;
	GsArray<KnMotion*> _kn_motions;
	int _current_kn_motion;
	KnMotion* getKnMotion(int i);
	KnMotion* currentKnMotion(){ return getKnMotion(_current_kn_motion);}
	KnMotion* openKnMotion(const GsString& fileName );

	//for drawing motions
	VecTrajectory* stanceHandCurve;
	VecTrajectory* swingHandCurve;
	VecTrajectory* rootCurve;
	VecTrajectory* leftFootCurve;
	VecTrajectory* rightFootCurve;
	VecTrajectory* selectedCurve;
	VecTrajectory* comCurve;

	GsVec _swingStanceOffset;
	GsVec _swingStart;
	
	/*!current selected curves*/
	GsArray<Channel*> scvs;
	
	SnGroup* trajGrp;
	bool _verifyingMotions;
	bool _analyzingMotions;
	bool _expandingEnv;
public:
		Manipulator* goal_manip;
	void startExpandingEnv();
	Controller* currentController(){if(_current_controller==-1)return 0; return _controllers[_current_controller];}
	Controller* newController(const GsString&  contName);
	HumanMotion* currentMotion();

	HumanMotionManager(const GsString&  file);
	~HumanMotionManager();
	void init(PhysicalHuman* h,HumanManager* mgr);

///MOTION SELECTING and OPENING
	/*at different points in the phase the stance may be left or right so it should be set here*/
	void connectMotion(Motion* m);
	HumanMotion* getMotion(int i){return _controllers[_current_controller]->getMotion(i);}
	GsString currentMotionName();

	/*loads a controller if needed and returns its index*/
	int loadController(const GsString&  directoryName,bool forceReload = false);

	/*open a motion if needed and set it as the currentMotion*/
	void selectController( const GsString&  motionDirectory,bool forceReload = false);
	/*open a kinematic motion into a HumanMotion */
	int selectKnMotion( const GsString&  selectedM);
	HumanMotion* openMotion(const GsString& dirName);

	void reloadController();

//MOTION EDITING
	void addChannel( const GsString&  n,Serializable* sav, int param ,channel_dof_types dof,int arrayIdx,chanel_modes mode,bool constant);
	void unselectCurves();
	void selectCurve( Channel* ch );
	void mirrorKnMotion();

//MOTION DRAWING
	void drawMotion();
	void updateMotionTrajectories();

	/*basically for the UI phase slider returns 0-1 based on motion time/motion->duration*/
	float getPhase();
	/*sets time to zero and loads motion initial state*/
	void resetAnimation();
	void setPhase(float t);
	void setPhase();
	float update(float dt);

	

//GETTERS



	float duration();
	void duration(float d);
	bool running();
	GsArray<Channel*>* getSelectedChannels();
	float getAnimationTimeStep();
	KnMotion* getSelectedKnMotion();
//SETTERS
	void togglePlaying();
	/*update the animation curves and the animation accordingly*/
	void updateCurves();


	void chooseKnMotion( KnMotion* selectedM );



	GsArray<int> currentMotionChannelIndexes();
	void setRunning( bool param1 );
	float getTime();
	void setTime(float f);
	bool loops();
	void loops(bool p);
	bool resets();
	void resets(bool p);
	
	void deselectMotion();
	void setSelectedMotionChannelIndexes(const GsArray<int>& idxs);
	void applyParameters();
	

		void makeComposite(const GsString&  mname,const GsString&  controlM,const GsArray<bool>& vals,bool allFrames=true,int firstFrame=0,int lastFrame=100);
		void makeTimeWarp();

		HumanMotion* getDefaultControlMotion();
		void refreshControlMotion();
		void moveChannelUp();
		void moveChannelDown();
		void scale_skeleton( KnSkeleton* sk,float sc );
		void updateStateVis();
		int getFrame();
		void setFrame( int f );
		float startTime(){return pFloat(human_motion_manager_start_time);}
		float endTime(){return pFloat(human_motion_manager_end_time);}
		float startFrame();
		float endFrame();
		void scaleMotion( float v );
	
		void setStartState( const GsString&  stateName );

		void makeControlChannel();
		void makeChannelNode( chanel_modes mode );
		
		void updateMotionData();


		////TRAJECTORY EDITING
		/*turn on or off the sampling of the selected channels*/
		void makeSample(bool makeSample);
		/*editing a single control point of a trajectory*/
		void pointEdit(double px, double py,double rx,double ry,double xMin,double xMax,double yMin,double yMax);
		/*randomize curve within sample bounds*/
		void randomize();
		//zero curves
		void initCurves();
		void deleteChannels();
		void deleteCurve(int i);
		void fixShoulders(KnSkeleton* sk);
		TrajectoryChannel* push( GsVec2 mouseP, curve_state s );
		TrajectoryChannel* drag( GsVec2 mouseP );
		void setKey();
		void fitCurve( int points,float tolerance,float mergeDist,float concavity_tolerance );
		void mergeControlPoints( float tolerance );
		void deleteSelectedCurves();

		void interpolatMotion(float pz,float y = 0);
		void interpolatMotion(const GsArray<float>& parms,const GsArray<float>& weights,int motion_id = -1,int num_motions = -1);
		void makeLocalSample(float tol);
		void loadMotionEnvironment();
		void verifyMotions();
		void analyzeMotions();
		bool verifyingMotions(){return _verifyingMotions;}
		bool analyzingMotions(){return _analyzingMotions;}
	
		void forceKeysFromState(int m);
		void setGoalPosition( GsVec pos );
		void motionLinesVisible(bool vis);
		void updateMotionLines(int first = -1,int last=-1);
		void expandEnv();
		void addEnv();
		void saveMotions(int first,int last,bool rename = false);
		void showAllMotionLines();
		void removeDuplicates();
		bool selectClosestFromHull( GsVec _jumpP );
		bool setEnv( GsVec pos );
		void selectMode();
		KnScene* currentKnScene()
		{
			if(_current_skeleton == cmu_skeleton )
				return _cmu_kinematic_scene;
			else if(_current_skeleton == dynoman_skeleton)
				return _dynoman_kinematic_scene;
			else return 0;
		}
		void saveBaseMotion();
		void hideSelectedNodes();
		void hideAllNodes();

		void showAllNodes();
		void viewInputNodes();
		void setInputsVisible(Channel* ch);
		void selectChannel( Channel* ch );
		void viewOutputNodes();
		void setOutputsVisible(Channel* ch);
};
