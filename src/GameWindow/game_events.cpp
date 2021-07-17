#include <gsim/fl.h>
#include "game_window.h"

#include "ph_manager.h"
#include "ph_human.h"
#include "ph_file_manager.h"
#include "ph_motion_manager.h"
#include "ph_state_manager.h"
#include "ph_motion.h"
#include "m3dconnexion.h"
void GameWindow::event ( GameWindowEvent e )
{
	PhysicalHuman* human = manager->selectedCharacter();
	HumanFileManager* files = manager->getFiles();
	HumanMotionManager* motion_manager = manager->getMotionManager();
	
	switch ( e )
	{
		case evQuit: gs_exit();  break;
		
		case evReset:
			reset();
		break;
		case evPlay:
			play();
		break;
	
		case evSceneAdjust:
			{
				_jumpP.z = (float)ui_jump_distance->value();
				_jumpP.y = (float)ui_jump_height->value();
			
				makeJump();
			}
		break;

		default:
			phout<<"unrecognized event\n";
		break;
	}

}
