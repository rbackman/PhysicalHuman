# pragma once

#include "common.h"
#include "game_fluid.h"
#include "util_serializable.h"
#include "game_events.h"
#include "game_og_viewer.h"

class HumanManager;
class HumanMotionManager;
enum jump_select_mode
{
	select_closest_mode,
	interpolate_mode,
	move_env_mode,
	env_hull_mode,
};
class GameWindow : public GameWindowFluid ,public Serializable
 { 
 private :
	
	 GsString _cmd;
	 bool cmdAvailable;
	  GsString _buf;
	  	int _interp_motion;

	 GsArray<int> envID;
	 int characterID;
	bool closestMode;
	public :
		GsVec _jumpP;
		GameViewer* og_viewer;
		GsQuat camOrient;
		GsVec camTrans;
		GameWindow (HumanManager* mgr, GsString file);
		~GameWindow ();
		virtual void event ( GameWindowEvent e );
		jump_select_mode _jump_select_mode;
		HumanManager* manager;
		HumanMotionManager* motion_manager;

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

	void handle_viewer_event( const GsEvent & e );
	void viewCharacter();
	void updateEnvironent();
	void updateCharacter();
	void makeJump();
	void play();
	void reset();
	void selectClosest();
};

