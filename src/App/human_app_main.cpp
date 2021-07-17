#include "human_app_main.h"
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

#include "ph_motion_manager.h"
#include "ph_motion_segmenter.h"
#include "phw_window.h"
#include "ph_manager.h"
#include "util.h"
#include "ph_trajectory_planner.h"
//#include "m3dconnexion.h"


AppMain::AppMain (const GsString& dir)
{
	
	
	updateCount=0;
	manager = new HumanManager(dir);

	manager->init();
	mainwin = new HumanWindow(manager,manager->getFiles()->getPrefFile());
	mainwin->ui_ode_run->value(manager->isRunning());
	mainwin->ui_viewer->_root->add(manager->getRoot());
	
	mainwin->loadConfigurationNamed("man");
	mainwin->loadScene("FlatGround");

//	manager->getMotionManager()->selectController("..\\data\\motions\\Jump\\");
//	manager->getMotionManager()->currentController()->selectMotion(0);
//	manager->getMotionManager()->loadMotionEnvironment();

//	manager->getStateManager()->loadDefaultState();
		
	manager->step();
	mainwin->show();

	
// 	int devices = m3d_init ();
// 	foundMouse = true;
// 	if ( devices==0 ) {phout<<"failed to load 3dmouse\n";  foundMouse = false;}
// 	else m3d_sensitivity (200,8000);

}


void AppMain::savePrefs()
{
	GsOutput f;
	f.open("../data/pref.dat");
	f<<"#config file for physical humanoid\n" ;
	f<<"#Robert Backman rbackman@ucmerced.edu\n#2/27/2011\n\n";

	f<< manager->toString();
	//f<<toString();
	f<< mainwin->toString();
	f<<manager->getMotionManager()->toString();
	f<<manager->getMotionSegmenter()->toString();
	f<<manager->getStateManager()->toString();
	f<<manager->getFiles()->toString();
	f<<manager->getWorld()->toString();
	f<<manager->getPlanner()->toString();
	f<<mainwin->ui_viewer->toString();
	f<<mainwin->ui_node_viewer->toString();
	f<<mainwin->ui_graph_viewer->toString();

	f<<"\nend";
	f.close();

}



void AppMain::update()
{

// 		trackManip->translation(kinect->getLocalPosition());
// 		trackManip->rotation(kinect->getOrientation());
	
	
		if (manager->isRunning())
		{
			if(manager->update())
			{
				updateCount++;
				if(manager->pBool(human_manager_graphics_active))
					mainwin->update();
				else if(updateCount%30==0)
				{
					mainwin->update();
				}
				
			}
		}
		else
		{
			manager->update();
			mainwin->update();
		}

		
		if(mainwin->hasCmd())
		{
			mainwin->message(processCmd(mainwin->getCmd()));
		}

// 		GsCamview* cam = &mainwin->ui_viewer->camera();
// 		if(foundMouse)
// 		{
// 
// 			float v[6] = {0,0,0,0,0,0}; char but[8] = {0,0,0,0,0,0,0,0};
// 			const float ts=4.0f;
// 			const float rs=0.3f;
// 			GsQuat camOrient =  cam->rotation;
// 			GsVec camTrans = cam->translation;
// 
// 			// read input:
// 			if ( m3d_read(v,but) ) // could read
// 			{
// 				//phout<<" vals:\t"; for ( i=0; i<6; i++ ) phout << v[i] <<"\t"; phout<<gsnl;
// 
// 				GsVec dv ( v[0]*ts, -v[2]*ts, v[1]*ts );
// 				dv = camOrient.apply(dv);
// 
// 				camTrans = camTrans + dv;
// 				GsVec reul = GsVec(v[3],-v[5],v[4]);
// 				reul = camOrient.apply(reul)*rs;
// 
// 				GsQuat qx ( GsVec::i, reul.x );
// 				GsQuat qy ( GsVec::j, reul.y);
// 				GsQuat qz ( GsVec::k, reul.z );
// 
// 				camOrient = (qx * qy * qz) * camOrient;
// 
// 				//quat2mat ( q, ui_viewer->transf()->get() );
// 				//ui_viewer->transf()->get().setrans(t);
// 				cam->rotation = camOrient;
// 				cam->translation = camTrans;
// 				
// 			}
// 		}

}
void AppMain::listCommands()
{
	phout<<"\n\n\n\n\n\n";
	phout<<"------AppMain------"<<gsnl;
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
GsString AppMain::processCmd( const GsString& cm )
{
	GsString response;
	GsString cmd = cm;
	cmd.trim();
	if(cmd.search("edit")==0)
	{
		cmd.substring(4,cmd.len());
		cmd.trim();
	
	
		int delim = cmd.search(":");
		if(delim!=-1)
		{	
			GsString type = cmd;
			GsString name = cmd;
			
			type.substring(0,delim-1);
			name.substring(delim+1,name.len()-1);
			type.trim();
			name.trim();
			Serializable* s = manager->findSerializable(name,type);
			if(s)
			{
				mainwin->loadParameterEditor(s,cmd);
				response = "Loading Editor for object: ";
				response<<name<<" type:"<<type;
			}
			else
			{
				response = "Could not load editor for object: ";
				response<<name<<" type:"<<type;
			}
				
		}
		else 
		{
				response<<"Edit what? should be 'edit ObjectType:ObjectName'";
		}
		
	}
	else if(cmd.search("node")==0)
	{
		cmd.substring(4,cmd.len());
		cmd.trim();

		if(cmd.search("traj")>=0)
		{
			manager->getMotionManager()->makeChannelNode(channel_trajectory);
			mainwin->refreshChannelList();
			return "made new trajectory";
		}


		int delim = cmd.search(":");
		if(delim!=-1)
		{	
			GsString type = cmd;
			GsString name = cmd;

			type.substring(0,delim-1);
			name.substring(delim+1,name.len()-1);
			type.trim();
			name.trim();
			Serializable* s = manager->findSerializable(name,type);
			if(s)
			{
				mainwin->loadNodeParameterEditor(s,cmd);
				response = "Loading Editor for object to add nodes: ";
				response<<name<<" type:"<<type;
			}
			else
			{
				response = "Could not load editor for object: ";
				response<<name<<" type:"<<type;
			}

		}
		else 
		{
			response<<"node what? should be 'node ObjectType:ObjectName'";
		}

	}
	else if(cmd.search("interp")==0)
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
	}
	else if(cmd == "save")
	{
		response<<" app saved ";
		savePrefs();
	}
	else if(cmd == "help")
	{
		response<<"commands listed";
		listCommands();
		manager->listCommands();
	}
	else if(cmd == "plan stop")
	{
		manager->getPlanner()->stop();
		mainwin->loadMotionFiles();
	}
	else if(cmd == "plan start")
	{
		manager->getPlanner()->start();
	
	}
	else if(cmd == "hello")
	{
		response<<"app says hi";
	}
	else if(cmd.search("human")==0)
	{
		cmd.substring(6,cmd.len());
		response<<manager->processCmd(cmd);
	}
#ifdef USE_KINECT
	else  if (cmd == "track")
	{

		if(kinect)
		{	
			track = true;
			trackManip = mainwin->selectedManipulator();
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
	AppMain* App = new AppMain(dataDir);
	while(true)
	{
		App->update();
	}
	return 0;
}


