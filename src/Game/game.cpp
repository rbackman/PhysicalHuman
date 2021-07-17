#include "game.h"
#include "game_window.h"

#include "ode_world.h"
#include "ph_human.h"
#include "ode_sphere.h"
#include "ode_box.h"
#include "ph_mod_root.h"
#include "ph_mod_com.h"
#include "ph_mod_contact.h"
#include "ph_mod_ref.h"
#include "ph_mod_gravity.h"
#include "ph_mod_puppet.h"
#include "ph_mod_ik.h"
#include "ph_state_manager.h"
#include "util_curve.h"
#include "ph_file_manager.h"
#include "ph_controller.h"
#include "ph_motion_manager.h"
#include "ph_motion_segmenter.h"
#include "ph_motion.h"
#include "ph_manager.h"
#include "util.h"
#include "util_manipulator.h"
#include "ph_trajectory_planner.h"
#include "m3dconnexion.h"

#ifdef USE_KINECT
#include "kinect_manager.h"
#include "kinect_main_win.h"
bool Game::openKinect()
{
	bool ret = false;
	
		
	if(kinect == 0)
	{
		kinectWin = new KinectMainWin;

		kinect = new KinectManager(kinectWin);
	}

	if(kinect->init()==0)
	{
		if(kinectWin)kinectWin->ui_window->hide();	
		//delete kinect;
		//kinect = 0;
		ret = false;
	}
	else
	{
		kinectWin->show();
		ret = true;
	}
	
	
	
	
	return ret;
}
#endif

Game::Game (GsString dir)
{
	#ifdef USE_KINECT
	trackManip = 0;
	kinectWin = 0;
	kinect = 0;
	frameCount = 0;
	track = false;
	kinect_sample_delay = 20;
	#endif

	updateCount=0;
	manager = new HumanManager(dir);


	manager->init();
	
	mainwin = new GameWindow(manager,manager->getFiles()->getPrefFile());
	//mainwin->ui_viewer->_root->add(manager->getRoot());

	PhysicalHuman* human = new PhysicalHuman(manager,"Man");
	manager->pushCharacter(human);

	//manager->loadScene("RandomPillars");
	//manager->loadScene("FlatGround");
	manager->getStateManager()->loadDefaultState();
	manager->getMotionManager()->selectController("..\\data\\motions\\Jump\\");
	manager->getMotionManager()->currentController()->selectMotion(0);
	manager->loadScene(manager->getMotionManager()->currentController()->getMotionScene());

	human->setP(human_show_visual_geo,true);
	human->setP(human_show_collision_geo,false);
	human->setP(human_show_skeleton,false);
	human->setP(human_show_controllers,false);
	human->applyParameters();

	manager->step();
	mainwin->show();

}



void Game::update()
{
	#ifdef USE_KINECT
		frameCount++;

		if(frameCount % kinect_sample_delay==0)
		{
			if(kinect )
			{
				kinect->update();
				if (track)
				{
					trackManip->translation(kinect->getLocalMarkerPosition());
					trackManip->rotation(kinect->getOrientation());
				}
			}
		}
	
	#endif

		if (manager->isRunning())
		{
			if(manager->update())
			{
				mainwin->update();
				//og_viewer->wait();
			}
		}
		else
		{
			manager->update();
			mainwin->update();
		}

		
		if(mainwin->hasCmd())
		{
			//mainwin->message(
				processCmd(mainwin->getCmd());
		}
}
void Game::listCommands()
{
	phout<<"\n\n\n\n\n\n";
	phout<<"------Game------"<<gsnl;
	phout<<"command list: "<<gsnl;
	phout<<"help:		you may have figured that out ;-)"<<gsnl;
	#ifdef USE_KINECT
	phout<<"track:		have the currently selected manipulator track the kinect"<<gsnl;
	phout<<"stop_track: stop tracking the kinect"<<gsnl;
	phout<<"kinect:		load the kinect"<<gsnl;
	#endif

	phout<<"edit 'ObjectType:Object':  tries to load an object "<<gsnl;
	phout<<"plan start:  start the planner with the motion selected "<<gsnl;
	phout<<"plan stop:  stop the planner "<<gsnl;
}
GsString Game::processCmd( GsString cmd )
{
	GsString response;
	cmd.trim();
	if(cmd.search("interp")==0)
	{
		cmd.substring(6,cmd.len());
		cmd.trim();
		GsStrings val;
		val.parse(cmd);
		GsArray<float> vals;
		GsArray<float> weights;
		for (int i=0;i<val.size();i++)
		{
			vals.push() = (float)atof(val[i]);
			weights.push() = 1.0f;
		}
		phout<<"interpolating with all parms weight 1\n";
		manager->getMotionManager()->interpolatMotion(vals,weights);
		
			GsString dir = manager->getMotionManager()->currentController()->getDirectoryName();
			dir<<manager->getMotionManager()->currentMotion()->getMotionName()<<".motion";
			manager->getFiles()->saveMotionTrajectories(dir,manager->getMotionManager()->currentController()->currentMotion());
		
	}
#ifdef USE_KINECT
	else  if (cmd == "track")
	{

		if(kinect)
		{	
			track = true;
			//trackManip = mainwin->selectedManipulator();
			response<<"tracking manip "<<trackManip->name();
		}
		else
			response<<"do 'kinect' cmd first\n";
	}
	else if (cmd == "stop_track")
	{
		response<<"stop tracking";
		track = false;
		trackManip =0;
		
	}
	else if (cmd == "kinect")
	{
		
		if(openKinect())
		{
			response<<"kinect open";
		}
		else
		{
			response<<"no kinect or closed.";
		}
	}
#endif

	response<<":";
	//phout<<"response: "<<response;
	return response;
}

int main ( int argc, char** argv )
{
	GsString dataDir = "..";
	dataDir<<SLASH<<"data";
	Game* game = new Game(dataDir);

	int devices = m3d_init ();
	bool foundMouse = true;
	if ( devices==0 ) {phout<<"failed to load 3dmouse\n";  foundMouse = false;}
	else m3d_sensitivity (200,8000);

	Ogre::Camera* cam = game->mainwin->og_viewer->getCamera();
	Ogre::Vector3 v = cam->getPosition();
	Ogre::Quaternion q = cam->getOrientation();
	game->mainwin->camTrans = GsVec(v.x,v.y,v.z);
	game->mainwin->camOrient = GsQuat(q.w,q.x,q.y,q.z);

	while(true)
	{
		if(foundMouse)
		{
			
			float v[6] = {0,0,0,0,0,0}; char but[8] = {0,0,0,0,0,0,0,0};
			const float ts=4.0f;
			const float rs=0.3f;
			GsQuat camOrient = game->mainwin->camOrient;
			GsVec camTrans = game->mainwin->camTrans;

			// read input:
			if ( m3d_read(v,but) ) // could read
			{
				//phout<<" vals:\t"; for ( i=0; i<6; i++ ) phout << v[i] <<"\t"; phout<<gsnl;

				GsVec dv ( v[0]*ts, -v[2]*ts, v[1]*ts );
				dv = camOrient.apply(dv);

				camTrans = camTrans + dv;
				GsVec reul = GsVec(v[3],-v[5],v[4]);
				reul = camOrient.apply(reul)*rs;

				GsQuat qx ( GsVec::i, reul.x );
				GsQuat qy ( GsVec::j, reul.y);
				GsQuat qz ( GsVec::k, reul.z );
				
				camOrient = (qx * qy * qz) * camOrient;
				
				//quat2mat ( q, ui_viewer->transf()->get() );
				//ui_viewer->transf()->get().setrans(t);
				game->mainwin->og_viewer->getCamera()->setOrientation(Ogre::Quaternion(camOrient.w,camOrient.x,camOrient.y,camOrient.z));
				game->mainwin->og_viewer->getCamera()->setPosition(Ogre::Vector3(camTrans.x,camTrans.y,camTrans.z));
				game->mainwin->camTrans = camTrans;
				game->mainwin->camOrient = camOrient;
			}
		}

		game->update();

	}
	return 0;
}


