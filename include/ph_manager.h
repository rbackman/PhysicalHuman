#pragma once

#include "common.h"
#include "util_serializable.h"

class HumanFileManager;
class HumanMotionManager;
class HumanMotionSegmenter;
class HumanStateManager;
class PhysicalHuman;
class ODEWorld;
class Model;
class ODEObject;
class TrajectoryPlanner;
class EnvBuilder;
class ODEBox;
class Client;
class CharBuilder;

enum human_manager_parms
{
	human_manager_simulation_step,
	human_manager_simulation_running,
	human_manager_graphics_active,
	human_manager_scene,
	human_manager_auto_reset_from_hip_height
};



class HumanManager : public Serializable
{
public:
	HumanManager(const GsString& file);
	~HumanManager();
	float envOffY;
	float envOffZ;

private:
	int _oversample_counter;
	int _frame_count;
	int _selectedCharacter;
	bool _animation_step;
	SnGroup*  _groundObjects;
	SnGroup* _root;
	SnGroup* envGrp;
	ODEWorld* _world; 
	EnvBuilder* env_builder;
	CharBuilder* char_builder;
	HumanFileManager* _files;
	
	HumanMotionSegmenter* _motion_segmenter;
	


	Client* client;

	TrajectoryPlanner* _planner;
	GsArray<PhysicalHuman*> _characters;
	GsArray<ODEObject*> _objects;
	GsArray<Serializable*> _serializables;

	bool _has_message;
	GsString _last_message;
	int _message_count;
	bool _waiting_to_stop;

public:
	/*the current character in focus. right now it only supports one character at a time but the basic framework is there to support several characters*/
	PhysicalHuman* selectedCharacter();
	/*access the file manager*/
	HumanFileManager* getFiles();
	/*get the motion manager*/
	HumanMotionManager* getMotionManager();
	/*get the StateManager*/
	HumanStateManager* getStateManager();
	/*get the SceneGraph root node*/
	SnGroup* getRoot(){return _root;}
	/*get the ODE simulation environment*/
	ODEWorld* getWorld();
	/*get the TrajectoryPlanner which randomizes motions to try and discover new ones*/
	TrajectoryPlanner* getPlanner(){return _planner;}
	/*get the motion segmenter which takes in motion capture data and converts it to effector trajectories and composite motions*/
	HumanMotionSegmenter* getMotionSegmenter();
	
	void startClient();
	/*called every simulation step*/
	bool update();
	/*if the manager is active or not*/
	bool isRunning();
	/*called once at begining of application life*/
	void init();
	/*advance the simulation by one animation step.. this also includes _world->pInt(world_simulation_over_sample) simulation steps */
	void step();
	/*if this cycle of the application loop the scene should be redrawn. works at a lower frequency than 
	the simulation step based on _world->pInt(world_simulation_over_sample)*/
	bool animationStep();
	/*look for a serializable object based on its name and type.*/
	Serializable* findSerializable(const GsString& name,const GsString& type);
	/*add a character to the scene*/
	void pushCharacter(PhysicalHuman* guy);
	/*load one of the preset scenes. options are FlatGround or RandomPillars . consider loading this from a file*/
	void loadScene(const GsString& sceneName);
	/*load a skeleton into the scene.*/
	void loadSkeleton(const GsString& filename);
	/*adds a bunch of balls above the character to try and disturb his balance*/
	void dropBalls();
	/*start or stop the simulation*/
	void setRunning( bool val );
	/*cast a ray into the scene and return the closest Model that is collided with. returns 0 if no objects collide with the ray*/
	Serializable* checkRay( const GsLine& ray );
	
	/*get a list of commangs that can be sent to processCmd(GsString cmd)*/
	void listCommands();
	/*takes a string and if it is recognized executes a certain command. use listCommands() to get the acceptable commangs*/
	GsString processCmd( const GsString& cmd );
	/*reset the simulation environment to the default state. use setDefaultState to change the default state*/
	void resetState();
	/*the animation timestep. usually 1/30 or 1/60*/
	float getAnimationTimeStep();
	/*sets the state of the character. the string input is just the name of the state with no directory. the 
	StateManager will either attempt to load from a file or if it is already loaded will just use that*/


	/*the manager can get messages from any component in the system that has a reference to it. these are passed to the window for display*/
	/*true if there is a message waiting for display*/
	bool hasMessage();
	/*the message to display.. may contain several messages concatenated together*/
	GsString getMessage();
	/*send a message to the manager. typically just transfered to the UI but may request a command if it starts with 'cmd'*/
	void message(const GsString& m);
	/*helper function just concatenates strings and sends to message(GsString m) */
	void message(const GsString& m,const GsString& s);
	/*helper function just concatenates string and float and sends to message(GsString m) */
	void message(const GsString& m,float s);
	int numObjects();
	ODEObject* getObject( int i );

	void loadJumpScene(float z,float y);
	/*show or hide the output of the jumps*/
	void toggleEnv();
	void renameMotion(int i);
	EnvBuilder* getEnvBuilder();
	CharBuilder* getCharBuilder();
	ODEBox* addBox(GsVec p,GsVec s,bool dynamic = false);
	void newConfig( const GsString& ans );
	PhysicalHuman* getCharacter( const GsString& configName );
	void setCharacter( const GsString& configName );
	void clearCharacters();

	void showSelectedController();
};