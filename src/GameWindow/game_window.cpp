
# include <gsim/fl.h>
# include "game_window.h"


#include "util_serializable_editor.h"
#include "ph_motion_manager.h"
#include "ph_manager.h"
#include "ph_mod_manip.h"
#include "ph_mod_ik.h"
#include "ph_file_manager.h"
#include "ph_motion.h"
#include "ph_state_manager.h"
#include "util_channel_traj.h"
#include "ode_object.h"
#include "util_models.h"
#include "m3dconnexion.h"

#include <gsim/gs_scandir.h>

GameWindow::GameWindow (HumanManager* mgr,GsString file) : Serializable("GameWindow")
 {
	 manager  = mgr;
	 motion_manager = manager->getMotionManager();
	// ui_viewer->init ( this , file );	
	 cmdAvailable = false;
	 _interp_motion = -1;
	 characterID = -1;
	 closestMode = true;
	 _jump_select_mode = select_closest_mode;
 }
GameWindow::~GameWindow ()
{
}
int cnt = 0;
void GameWindow::update()
{
	fltk::check();
	og_viewer->update();
	og_viewer->render();
}

void GameWindow::show ()
 {
	 PhysicalHuman* human = manager->selectedCharacter();
	 og_viewer = new GameViewer("../data/ogre/",this);
	 og_viewer->init(false);
	 og_viewer->showAxis(false);
	
	 _jumpP.z = motion_manager->currentController()->getDescriptorValue("distance");
	 _jumpP.x = motion_manager->currentController()->getDescriptorValue("height");
	 ui_jump_distance->value(_jumpP.z);
	 ui_jump_height->value(_jumpP.y);
	 updateCharacter();
	 updateEnvironent();
	// ui_window->show();
}

void GameWindow::handle_viewer_event( const GsEvent & e )
{
	if(e.type == GsEvent::Push)
	{
		Serializable* s = manager->checkRay(e.ray);
		if (s)
		{
			
			if(s->type()=="PhysicalJoint")
			{
				manager->selectedCharacter()->setEditJoint((PhysicalJoint*)s);
			}
			else
			{
				
			}
		}
	}
}


void GameWindow::viewCharacter()
{
	PhysicalHuman* human = manager->selectedCharacter();
// 	if(human->pBool(human_show_collision_geo))
// 	{
// 		ui_viewer->view_node(human->getCollisionGroup(),ui_viewer->pFloat(camera_fovy));
// 	}
// 	else
// 		ui_viewer->view_node(human->getVisSkelScene(),ui_viewer->pFloat(camera_fovy));
}
void GameWindow::updateCharacter()
{
	if(characterID>0)
		og_viewer->removeAgent(characterID);

	characterID = og_viewer->add(manager->selectedCharacter()->getVisSkel());
	og_viewer->getAgent(characterID)->setCastShadow(true);
	og_viewer->getAgent(characterID)->setReceiveShadow(false);
	og_viewer->getAgent(characterID)->setScale(GsVec(100,100,100));
}
void GameWindow::updateEnvironent()
{
	if(manager->numObjects()!=envID.size())
	{
		for (int i=0;i<envID.size();i++)
		{
			og_viewer->removeAgent(envID[i]);
		}
		
		for(int i=0;i<manager->numObjects();i++)
		{
			
			int id = og_viewer->add(manager->getObject(i)->getModel()->getModel());
			og_viewer->getAgent(id)->setPosition(manager->getObject(i)->getPosition()*100);
			og_viewer->getAgent(id)->setReceiveShadow(true);
			og_viewer->getAgent(id)->setCastShadow(false);

			og_viewer->getAgent(id)->setScale(GsVec(100,100,100));
			envID.push(id);
		}
	}
	else
	{
		
		for(int i=0;i<manager->numObjects();i++)
		{

			og_viewer->getAgent(envID[i])->setPosition(manager->getObject(i)->getPosition()*100);
		
		}
	}
}

void GameWindow::makeJump()
{
	//phout<<"jump p "<<_jumpP<<gsnl;
	switch(_jump_select_mode)
	{
	case select_closest_mode:
		motion_manager->currentController()->selectClosest(_jumpP);
		manager->loadScene("MotionJump");
		break;
	case interpolate_mode:
		motion_manager->interpolatMotion(_jumpP.z,_jumpP.y);
		manager->loadScene("MotionJump");
		break;
	case move_env_mode:
		motion_manager->setGoalPosition(_jumpP);
		break;
	case env_hull_mode:
		motion_manager->selectClosestFromHull(_jumpP);
		break;
	}
		
	
	
	updateEnvironent();
}

void GameWindow::play()
{
	motion_manager->setRunning(true);
	motion_manager->setTime(0);
}

void GameWindow::reset()
{
	motion_manager->resetAnimation();
	motion_manager->setRunning(false);
	manager->resetState();
	if(motion_manager->currentController())
		motion_manager->loadMotionEnvironment();
	else
		manager->loadScene("FlatGround");

	updateEnvironent();
}

void GameWindow::selectClosest()
{
	closestMode = !closestMode;
	if(closestMode)
	{
		motion_manager->currentController()->selectClosest(_jumpP);
	}
	manager->loadScene(motion_manager->currentController()->getMotionScene());
	updateEnvironent();
}

