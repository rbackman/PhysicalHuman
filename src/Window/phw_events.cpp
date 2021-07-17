#include "ph_manager.h"
#include "phw_window.h"
#include "ph_mod.h"
#include "ph_human.h"
#include "ph_joint.h"
#include <gsim/fl.h>
#include "ph_mod_root.h"
#include "ph_mod_com.h"
#include "ph_mod_ref.h"
#include "ph_mod_contact.h"
#include "ph_mod_gravity.h"
#include "ph_mod_balance.h"
#include "ph_mod_ik.h"
#include "ph_mod_puppet.h"
#include "ph_mod_virtual.h"
#include "ph_file_manager.h"
#include "ph_motion_manager.h"
#include "ph_motion_segmenter.h"
#include "ph_state_manager.h"
#include "util_channel_traj.h"
#include "phw_node_viewer.h"
#include "ph_trajectory_planner.h"
#include <gsim/fl_skeleton_win.h>
#include "ph_motion.h"
#include "ph_env_builder.h"
#include "ph_char_builder.h"

void HumanWindow::refreshChannelList()
{
	   HumanMotionManager* motion_manager = manager->selectedCharacter()->getMotionManager();
	 ui_channelList->clear();
	 ui_motion_duration->value(motion_manager->duration());
	 HumanMotion* m = motion_manager->currentMotion();
	 if(m)
	 {
		 for(int j=0;j<m->numChannels();j++)
		 {
			 Channel* ch = m->getChannel(j);
			
			 if(ch)	
			 {
				 if(ui_trajectories_vis->value())
				 {
					 if(!ch->isTrajectory())
						 continue;
				 }

				 if(ch->getObject())
				 {
					 GsString channelName =  ch->name();
					 channelName<<"/Output/";
					 GsString objectName = channelName;
					 objectName << ch->getObject()->name();	
					 objectName<<"/";
					 objectName<<ch->getParameterName();
					 objectName<<"/";
					 objectName<<channelTypeToString(ch->getChannelType());

					 ui_channelList->add(objectName,0,0);
				 }
				 else
				 {
					
					 for(int k=0;k<m->numChannels();k++)
					 {
						 if(k!=j)
						 {
							 Channel* chI = m->getChannel(k);
							 if(chI->hasInput(ch))
							 {
								 GsString channelName =  ch->name();
								 channelName<<"/Output/"<<chI->name();
								  ui_channelList->add(channelName,0,0);
							 }
						 }
					 }

					
				 }
				for(int i=0;i<ch->numInputs();i++)
				{
					GsString contName = ch->name();
					if(ch->getInput(i))
						contName<<"/Inputs/"<<ch->getInput(i)->name();
					else
						contName<<"/Inputs/0";
					ui_channelList->add(contName,0,0);
				}

			 }
		 }
		 GsArray<int> idxs = motion_manager->currentMotionChannelIndexes();
		 for (int i=0;i<idxs.size();i++)
		 {
			 if(idxs[i]<ui_channelList->size())
					ui_channelList->select(idxs[i]);
		 }

		 ui_node_viewer->initChannels();
		
		 ui_node_viewer->pushChannels(motion_manager->currentMotion()->getChannels());
		
	}
	
}
GsString HumanWindow::motionSelected()
{
		   HumanMotionManager* motion_manager = manager->selectedCharacter()->getMotionManager();

	int controller_index =  ui_motionList->value();
	if(controller_index == -1)
		return "none";

	GsString selected_controller =  ui_motionList->child(controller_index)->label();
	GsString controller_directory = manager->getFiles()->getMotionDirectory(manager->selectedCharacter()->characterName());
	controller_directory<<selected_controller<<SLASH;
	GsArray<int> idxs = motion_manager->currentMotionChannelIndexes();
	static bool save_selection = false;
	int level = ui_motionList->focus_level();
	if(level==1)
	{
		/*selected a single motion within the controller*/
		fltk::Widget* w = ui_motionList->goto_focus();
		if(w)
		{
			GsString name = w->label(); 
			gsout<<"select motion "<<name<<gsnl;
			motion_manager->currentController()->selectMotion(name);
			motion_manager->goal_manip->visible(false);
			if(motion_manager->pBool(human_motion_manager_resets))
			{
				manager->resetState();
			}
			if(motion_manager->pBool(human_motion_manager_load_env))
			{

				motion_manager->loadMotionEnvironment();
			}
			/*load all the descriptors into the interpolate ui for this motion*/
			GsArray<float>* parms = motion_manager->currentMotion()->pFloatArray(motion_descriptor_avg);
		
			for (int i=0;i<parms->size();i++)
			{
				ui_parm_edit[i]->value(parms->get(i));
			}
			ui_interp_window->redraw();

			/*keep the channels selected from the previous motion*/
			motion_manager->setSelectedMotionChannelIndexes(idxs);
			ui_graph_viewer->redraw();

			HumanMotion* m = motion_manager->currentMotion();
			if(m)
			{
				manager->selectedCharacter()->redraw();
				
				motion_manager->updateMotionLines();
				refreshChannelList();
			}

			updateTimeUI();
			setUIFromCharacter();
			ui_node_viewer->initChannels();
			if(motion_manager->currentMotion())
			{
				ui_node_viewer->pushChannels(motion_manager->currentMotion()->getChannels());
			}
			save_selection = true;
		}
	}
	else //selected a controller
	{
		save_selection = false;
		/*selected a controller so load all the motions if necessary*/
		gsout<<"load controller "<<controller_directory<<gsnl;
		motion_manager->selectController(controller_directory);
		Controller* c = motion_manager->currentController();
		//load the parameters and weights into the interpolate  UI view->windows->interpolate
		GsArray<float>* weights = motion_manager->currentController()->pFloatArray(controller_parameter_weights);
		GsStrings* names = motion_manager->currentController()->pStringArray(controller_parameters);

		for (int i=0;i<names->size();i++)
		{
			ui_parm_name[i]->value(names->get(i));
			ui_parm_edit[i]->value(0);
			ui_parm_weight[i]->value(weights->get(i));
		}

	}


	if(motion_manager->resets())
	{
		motion_manager->setP(human_motion_manager_time,0.0f);
		motion_manager->setRunning(false);
		/*look for the initial state, if it exists it is a subdirectory within the controller directory named 'initial'*/
		if(manager->getFiles()->subDirectoryExists(controller_directory,"initial"))
		{
			manager->getStateManager()->selectCharacterState("initial");
		}
		else
		{
			/*if no initial state was found just load whatever the last state selected was*/
			manager->getStateManager()->loadDefaultState();
		}
		
	}
	



	return selected_controller;
}
GsString HumanWindow::knMotionSelected()
{
	HumanMotionManager* motion_manager = manager->selectedCharacter()->getMotionManager();

	ui_channelList->clear();
	int motionIndex =  ui_kn_motion_list->value();
	if(motionIndex == -1)
		return "none";

	GsString selectedM =  ui_kn_motion_list->child(motionIndex)->label();
	
	if(selectedM=="cmu")
	{
		int idx = ui_kn_motion_list->focus_index()[0];
		int level = ui_kn_motion_list->focus_level();
		if(level==2)
		{
			
			fltk::Widget* w = ui_kn_motion_list->goto_focus();
			selectedM<<SLASH<<w->parent()->label()<<SLASH<<w->label();
			//phout<<"file "<<selectedM<<gsnl;
		}
		else
		{
			return "none";
		}
	}

	motion_manager->selectKnMotion(selectedM);

	KnMotion* m = motion_manager->getSelectedKnMotion();
	if(m)
	{
		m->apply_frame(0);
		manager->selectedCharacter()->redraw();
	
		if(motion_manager->getSelectedKnMotion())
		{
			ui_motion_duration->value( m->duration());
		}
	}
	
	ui_node_viewer->initChannels();

	updateTimeUI();
	return selectedM;
}

void HumanWindow::event ( HumanWindowEvent e )
{
	PhysicalHuman* human = manager->selectedCharacter();
	HumanFileManager* files = manager->getFiles();
	HumanMotionSegmenter* segmenter = manager->getMotionSegmenter();
	HumanMotionManager* motion_manager = manager->getMotionManager();

	Controller* current_controller = 0;
	if(motion_manager)
		current_controller = motion_manager->currentController();
	EnvBuilder* env_builder = manager->getEnvBuilder();
	CharBuilder* char_builder = manager->getCharBuilder();
	switch ( e )
	{
	
	case evCreateNode:
		{
			Serializable* s = manager->findSerializable(ui_node_create_serializable->value(),ui_node_create_serializable_type->value());
			int parmIndex = s->getParameterIndex(ui_node_create_parameter->value());
			channel_dof_types dof = stringToChannelType(ui_node_create_dof->value());
			int dof_index = (int)ui_node_create_parameter_index->value();
			bool constantVal = ui_constant_value->value();
			GsString ch_name = ui_node_create_name->value();
			if(ui_empty_node->value())
			{
				ui_node_viewer->addNode(s,parmIndex,dof,dof_index);
				message("adding temp node to node viewer",s->name());
			}
			else if(ui_trajectory_node->value())
			{
				motion_manager->addChannel(ch_name,s,parmIndex,dof,dof_index,channel_trajectory,constantVal);
			}
			else if(ui_add_node->value())
			{
				motion_manager->addChannel(ch_name,s,parmIndex,dof,dof_index,channel_additive,constantVal);
			}
			else if(ui_feedback_node->value())
			{
				motion_manager->addChannel(ch_name,s,parmIndex,dof,dof_index,channel_feedback,constantVal);
				
			}
			else if(ui_modulate_node->value())
			{
					motion_manager->addChannel(ch_name,s,parmIndex,dof,dof_index,channel_modulate,constantVal);
			}
			else if(ui_mult_node->value())
			{
				motion_manager->addChannel(ch_name,s,parmIndex,dof,dof_index,channel_multiply,constantVal);
			}
			else
			{
				message("must select the type of node to create");
				return;
			}
			ui_node_create_window->hide();
			refreshChannelList();
		}
		break;
	case evCharacterListSelected:
		loadConfigurationNamed(selectedConfig());
		break;
	case evLoadCharacter:
			loadConfigurationNamed( selectedConfig(),true);
		break;
	case evClearCharacters:
		manager->clearCharacters();
		break;
	case evSaveConfig:		
		files->saveConfigurationNamed(human,human->name());
		loadCharacterList();
		break;
	case evCreateConfig:
		if(!human->getGroup()->visible())
		{
			//human->setControllerVis(true);
			human->setP(human_show_collision_geo,true);
			human->setP(human_show_visual_geo,false);
			human->setP(human_show_skeleton,false);
			human->getGroup()->visible(true);
			char_builder->createCharacter();
			files->saveConfigurationNamed(human,human->characterName());
			human->setP(human_show_collision_geo,true);
			human->applyParameters();
			manager->getFiles()->saveStateNamed(human,"initial");
			manager->getStateManager()->selectCharacterState("initial",true);
		}
		break;
	case  evNewConfig:
	{	

		GsString ans = "newCharacer";
		if(fl_string_input("new character","enter name for new character",ans) )
		{
			manager->newConfig(ans);
			human = manager->selectedCharacter();
			loadScene("FlatGround");
			ui_character_list->add_leaf(ans);
			ui_character_list->select(ui_character_list->children()-1);
			loadStateFiles();
			setUIFromCharacter();
		}
	}
	break;
	case evEditConfig:
		{
			if(human->getGroup()->visible())
			{
				//human->setControllerVis(false);
				human->getGroup()->visible(false);
				manager->setRunning(false);

				ui_ode_run->value(false);
				char_builder->editCharacter(manager->selectedCharacter());
				human->setP(human_show_collision_geo,false);
				human->applyParameters();
			}
			
		}
	break;
	case evConnectKinect:

		manager->startClient();

		break;
	case evSaveCurrentScene:
		files->saveScene(manager->pString(human_manager_scene),manager);
		break;
	case evSaveNewScene:
		{
			GsString s = manager->pString(human_manager_scene);
			if(fl_string_input("new scene name","enter scene name",s))
			{
				files->saveScene(s,manager);

			}
		}
	break;
		case evEnvDuplicate:
			env_builder->duplicateBlock();
	 break;
	case evEnvAddBlock:
		env_builder->addBlock();
	break;
	case  evEnvMake:
		env_builder->makeEnv();
	break;
	case evEnvEdit:
		env_builder->editEnv();
		setUIFromCharacter();
		manager->setRunning(false);
		break;
	case evEnvResize:
		
		{
			env_builder->set_size(GsVec(ui_env_size[0]->value(),ui_env_size[1]->value(),ui_env_size[2]->value()));
			
			EnvManipulator* manip = env_builder->currentManip();
			manip->dynamic = ui_env_dynamic->value();

			if(manip->uniqueProperties!=ui_env_unique->value())
			{
				manip->uniqueProperties = ui_env_unique->value();
				if(manip->uniqueProperties)
				{
					ui_env_mu->show();
					ui_env_bounce->show();
					ui_env_density->show();
					ui_env_mu->value(env_builder->pFloat(env_manager_friction));
					manip->friction = env_builder->pFloat(env_manager_friction);
					ui_env_bounce->value(env_builder->pFloat(env_manager_bounce));
					manip->bounce = env_builder->pFloat(env_manager_bounce);
					ui_env_density->value(env_builder->pFloat(env_manager_density));
					manip->density = env_builder->pFloat(env_manager_density);
				}
				else
				{
					ui_env_mu->hide();
					ui_env_bounce->hide();
					ui_env_density->hide();
				}
			}

			if(manip->uniqueProperties)
			{

				manip->friction = (float)ui_env_mu->value();
				manip->bounce = (float)ui_env_bounce->value();
				manip->density = (float)ui_env_density->value();
			}
		
		}
	break;
	case evShowSampleBounds:
		manager->getPlanner()->toggleBoundLines();
	break;
	case evPrintState:
		phout<<"stancePoint(): "<<human->contact_module()->stancePoint()<<gsnl;
		break;
	case evGoalSelectMode:
		motion_manager->selectMode();
		break;
	case evStartExpandingCurrent:
		{
		int id =	motion_manager->currentController()->currentMotionID();
		motion_manager->setP(human_motion_manager_edit_motions,id);
		motion_manager->setP(human_motion_manager_edit_motions,id,1);
		motion_manager->startExpandingEnv();
		}
		break;
	case evUpdateAllLines:
		{
			motion_manager->updateMotionLines(0,current_controller->numMotions()-1);
		}break;
	case evEditSample:
		{
			for(int i=0;i<4;i++)
			manager->getPlanner()->setP(trajectory_planner_sample_bounds, (float)ui_sample_bounds[i]->value(),i) ;
			manager->getPlanner()->setP(trajectory_planner_seed, (int)ui_sample_bounds[4]->value()) ;
			gs_rseed((int)ui_sample_bounds[4]->value());
		}
		break;
	case evForceKeys:
		{
			GsString ans = "c";
			if(fl_string_input("all or current","enter 'a' or 'c' depending what motions to rename",ans) )
			{
				if(ans == "a")
				{
					for (int i=0;i<current_controller->numMotions();i++)
					{
						motion_manager->forceKeysFromState(i);
					}
				}
				else
				{
					motion_manager->forceKeysFromState(current_controller->currentMotionID());
				}

			}
			ui_graph_viewer->redraw();
		}
		break;


	case evSetControllerScene:
		{
			if(current_controller)
				current_controller->setP(controller_scene,selectedScene());
			manager->resetState();
		}
		break;
	case evEnvOffset:
		{
			 manager->envOffY = (float)ui_env_offset[0]->value();
			 manager->envOffZ = (float)ui_env_offset[1]->value();
		}
		break;
	case evSetFromEnvDesc:
		motion_manager->currentController()->setDescriptorFromEnv();
		break;
	case evSetEnvDescFromAnalysis:
		motion_manager->currentController()->setEnvFromDescriptor();
		break;
	case evShowMotionOutputs:
		manager->toggleEnv();
		break;
	case evPrintMotionDescriptors:
		{
			if(current_controller)
			{
				int numParms = current_controller->sizeOfParameter(controller_parameters);
				for (int i=0;i<numParms;i++)
				{
					phout<<current_controller->pString(controller_parameters,i)<<" "<<current_controller->getDescriptorValue(current_controller->pString(controller_parameters,i))<<gsnl;
				}
			}
		}
		break;
	case evPlanStart:
		manager->getPlanner()->start();
	break;
	case evPlanStop:
		manager->getPlanner()->stop();
		loadMotionFiles();
	break;
	case evMotionEditWindow:
		ui_motion_edit_window->show();
		break;
	case evMotionEnv:
		{
			motion_manager->loadMotionEnvironment();
		}
		break;

	case evSaveWithParmNames:
		{
			
			GsString ans = "c";
			if(fl_string_input("all or current","enter 'a' or 'c' depending what motions to rename",ans) )
			{
				int first = 0;
				int last = 0;
				if(ans == "a")
				{
					first = 0;
					last = current_controller->numMotions()-1;
				}
				else
				{
					first = current_controller->currentMotionID();
					last = first;
				}
				for (int i=first;i<=last;i++)
				{
				
					manager->renameMotion(i);
				
				}
				
			}

			loadMotionFiles();
		}
		break;
	case evSaveEditingMotions:
		if(current_controller)
		{
			motion_manager->saveMotions(motion_manager->pInt(human_motion_manager_edit_motions),motion_manager->pInt(human_motion_manager_edit_motions,1));
			loadMotionFiles();
		}
		break;
	case evSaveAllMotions:
		
		if(current_controller)
		{
			motion_manager->saveMotions(0,current_controller->numMotions()-1);
			motion_manager->saveBaseMotion();
			loadMotionFiles();
		}
		
			
	
		break;
	case evResetParms:
		{
			if(current_controller)
			{
				current_controller->initializeAnalysis();
			}
		}
		break;
	case evConfigureBounds:
		{
			float buf = 0.1f;
				if(fl_value_input("resample the sample bounds for the controller","buffer size ",buf))
				{
					motion_manager->currentController()->configureBounds(buf);
					ui_graph_viewer->redraw();
				}
		}
		break;
	case evSaveBounds:
	{	
		GsString dir = motion_manager->currentController()->getDirectoryName();
		GsString shortName = dir;
		directory_short_name(shortName);
		dir<<shortName<<".sample";
		manager->getFiles()->saveBounds(dir,motion_manager->currentController()->currentMotion());
	}
		break;
	case  evNextPoint:
		{
			if(getSelectedChannels()->size()==1)
			{
				Channel* ch =	getSelectedChannels()->get(0);
				if(ch->isTrajectory())
				{
						Trajectory* cv = ((TrajectoryChannel*) ch)->getCurve();
						if(cv->selectionState == CURVE_POINT_SELECTED)
							cv->selection++;
						else 
							cv->selectionState = CURVE_POINT_SELECTED;

						if(cv->selection==cv->numPoints())
							cv->selection=0;
						
						
				}
				updateSampleUI();
			}
		}
		ui_graph_viewer->redraw();
		break;

	case evSaveBaseMotionFile:
		{
			motion_manager->saveBaseMotion();
		}break;
	case evToggleBounds:
		ui_graph_viewer->toggleBool(graph_viewer_draw_bounds);
		ui_graph_viewer->redraw();
	break;
	case 	 evEditParm:
		{
		 motion_manager->setP(human_motion_manager_rbds_max_neighbors,(int)ui_rbds_height->value());
		 motion_manager->setP(human_motion_manager_rbds_support,(float)ui_rbds_support->value());
		
			if(ui_parm_live->value())
			{
				int numParms = current_controller->sizeOfParameter(controller_parameters);
				GsArray<float> parms;
				GsArray<float> weights;
				for (int i=0;i<numParms;i++)
				{
					parms.push((float)ui_parm_edit[i]->value());
					weights.push((float)ui_parm_weight[i]->value());
				}
				motion_manager->interpolatMotion(parms,weights,motion_manager->currentController()->currentMotionID());
				ui_graph_viewer->redraw();
				loadScene(current_controller->getMotionScene());	
			}
			
		}
		break;
	case evSelectClosestMotion:
		{
			int numParms = current_controller->sizeOfParameter(controller_parameters);
			GsArray<float> parms;
			GsArray<float> weights;
			for (int i=0;i<numParms;i++)
			{
				parms.push((float)ui_parm_edit[i]->value());
				weights.push((float)ui_parm_weight[i]->value());
			}
			motion_manager->currentController()->selectClosest(parms,weights);
			ui_graph_viewer->redraw();
		}
		break;
	case evMakeMotionEnv:
		{
			loadScene(current_controller->getMotionScene());
		}
		break;
	case evSetMotion:
		{
			motion_manager->setP(human_motion_manager_rbds_max_neighbors,(int)ui_rbds_height->value());
			motion_manager->setP(human_motion_manager_rbds_support,(float)ui_rbds_support->value());
				int numParms = current_controller->sizeOfParameter(controller_parameters);
			GsArray<float> parms;
			GsArray<float> weights;
			GsString nme;
			for (int i=0;i<numParms;i++)
			{
				nme<<ui_parm_edit[i]->value()<<"_";
				parms.push((float)ui_parm_edit[i]->value());
				weights.push((float)ui_parm_weight[i]->value());
			}
			motion_manager->interpolatMotion(parms,weights);
			GsString dir = current_controller->getDirectoryName();
			dir<<nme<<".motion";

			manager->getFiles()->saveMotion(dir,current_controller->currentMotion());

			loadMotionFiles();
		}
		break;
	case evInterpMotion:
	{
		ui_interp_window->show();
	}break;
	case evAnalyzeMotion:
		{
			motion_manager->resetAnimation();
			manager->resetState();
			motion_manager->currentController()->initializeAnalysis();
			motion_manager->currentController()->startAnalysis();
			motion_manager->setRunning(true);
		}break;
	case evReanalyzeMotions:
		{
			motion_manager->analyzeMotions();
		}
		break;
	case evVerifyMotion:
		motion_manager->verifyMotions();
	break;
	case  evPopGraphviewer:
		ui_graph_window->show();
		
		ui_graphviewer_group->remove_all();
		//ui_graph_viewer);
		ui_popviewer_group->add(ui_graph_viewer);
		ui_graph_viewer->parent(ui_popviewer_group);
		ui_popviewer_group->redraw();
	break;
	case evEmbedGraphviewer:
		ui_popviewer_group->remove_all(); //(ui_graph_viewer);
		ui_graphviewer_group->add(ui_graph_viewer);
		ui_graph_viewer->parent(ui_graphviewer_group);
		ui_graphviewer_group->redraw();
		//ui_graph_window->hide();
	break;
	case evShowAllNodes:
		motion_manager->showAllNodes();
		break;
	case evHideAllNodes:
		motion_manager->hideAllNodes();
	break;
	case evHideSelectedNodes:
		motion_manager->hideSelectedNodes();
		break;
	case evViewInputNodes:
		motion_manager->hideAllNodes();
		motion_manager->viewInputNodes();
	break;
	case evViewOutputNodes:
		motion_manager->hideAllNodes();
		motion_manager->viewOutputNodes();
		break;
	case evViewConnectedNodes:
		motion_manager->hideAllNodes();
		motion_manager->viewOutputNodes();
			motion_manager->viewInputNodes();
		break;
	case evDetachNode:
		ui_node_viewer->detach_node();
		break;
	case evMakeInv:
		motion_manager->makeChannelNode(channel_inverse);
		refreshChannelList();
		break;
	case  evMakeMod:
		motion_manager->makeChannelNode(channel_modulate);
		refreshChannelList();
	break;
	case evMakeMult:
		motion_manager->makeChannelNode(channel_multiply);
		refreshChannelList();
	break;
	case evMakeAdd:
		motion_manager->makeChannelNode(channel_additive);
		refreshChannelList();
	break;
	case evMakeTraj:
		motion_manager->makeChannelNode(channel_trajectory);
		refreshChannelList();
		break;

	case evMakeSwitch:
		motion_manager->makeChannelNode(channel_switch);
		refreshChannelList();
		break;
	case evDrawNodes:
		if(!ui_node_window->visible())
		{
			motion_manager->showAllNodes();
			ui_node_window->show();
		}
		ui_node_viewer->redraw();
		break;
	case evArrangeNodes:
		ui_node_viewer->initChannels();
		
		
		ui_node_viewer->pushChannels(motion_manager->currentMotion()->getChannels());
		
		
		ui_node_viewer->arrange_nodes();


		ui_graph_viewer->redraw();
		break;
	case evAddControlChannel:
		{
			motion_manager->makeControlChannel();
			refreshChannelList();
		}
		break;
	case evScaleMotion:
		{
			float v =0;
			if(fl_value_input("scale value","enter scale factor for motion",v))
			{
				motion_manager->scaleMotion(v);
				ui_graph_viewer->redraw();
			}
			
		}
		break;
	case evFitView:
		ui_graph_viewer->_view_translate = GsVec2(0,0);
		ui_graph_viewer->viewScale = 1.0f/motion_manager->duration();
		break;
	case evResetView:
		ui_graph_viewer->_view_translate = GsVec2(0,0);
		ui_graph_viewer->viewScale = 1.0f;
		break;
	case evRefreshChannels:
		refreshChannelList();
		break;
	case evTimeEdit:
	

		if(ui_show_time->value())
		{
			motion_manager->setP(human_motion_manager_end_time,(float)ui_last_time->value());
			motion_manager->setP(human_motion_manager_start_time,(float)ui_first_time->value());
			motion_manager->setPhase((float)ui_current_time->value()/motion_manager->duration());

		}
		else
		{
			motion_manager->setP(human_motion_manager_end_time,(float)(ui_last_frame->value()-1)*manager->getAnimationTimeStep());
			motion_manager->setP(human_motion_manager_start_time,(float)(ui_first_frame->value()-1)*manager->getAnimationTimeStep());
			motion_manager->setFrame((int)ui_current_frame->value());
		}
		if(motion_manager->currentMotion())
		{
			human->ik_module()->solve(false);
			if(manager->pBool(human_manager_graphics_active))
				human->redraw();
			ui_graph_viewer->redraw();
		}
		else if(motion_manager->getSelectedKnMotion())
		{
			//human->redraw();
			human->ik_module()->matchToSkeleton();
		}

		break;
	case evDrawSnapShots:
		motion_manager->toggleBool(human_motion_manager_draw_snapshots);
		motion_manager->applyParameters();
		break;
	case evOpenCompositeWindow:
		ui_group_name->value(motion_manager->currentMotionName());
		ui_composite_window->show();
		break;
	case  evMakeComposite:
		{
			GsArray<bool> vals;
			for(int i=0;i<33;i++)
			{
				vals.push(ui_comp[i]->value());
			}
			GsString mname = manager->getFiles()->getControlFile( ui_control_motion->value());
			mname <<".motion";
			
			
			motion_manager->makeComposite(ui_group_name->value(),mname,vals,ui_all_frames->value(),(int)ui_first_frame_comp->value(),(int)ui_last_frame_comp->value());
			loadMotionFiles();
			refreshChannelList();
			ui_composite_window->hide();
			message("made composite");
		}
	break;
	case evMakeTimeWarp:
		motion_manager->makeTimeWarp();
		refreshChannelList();
	break;
	case evViewCharacter:
		viewCharacter();
		break;

	
	case  evEditMotion:
		{
			Motion* m = motion_manager->currentMotion();
				if(m)
				loadParameterEditor(m);
		}
		break;
	case  evMakeLinear:
		{
			for(int i=0;i<motion_manager->getSelectedChannels()->size();i++)
			{
				Channel* ch = motion_manager->getSelectedChannels()->get(i);
				if(ch->isTrajectory())
					((TrajectoryChannel*)ch)->setCurveType(TRAJ_LINEAR);

			}
			motion_manager->updateCurves();
			ui_graph_viewer->redraw();
		}
		break;
	case	evMakeBezier:
		for(int i=0;i<motion_manager->getSelectedChannels()->size();i++)
		{
			Channel* ch = motion_manager->getSelectedChannels()->get(i);
			if(ch->isTrajectory())
				((TrajectoryChannel*)ch)->setCurveType(TRAJ_BEZIER);

		}
		motion_manager->updateCurves();
		ui_graph_viewer->redraw();
		break;
case evMakeStep:
	for(int i=0;i<motion_manager->getSelectedChannels()->size();i++)
	{
		Channel* ch = motion_manager->getSelectedChannels()->get(i);
		if(ch->isTrajectory())
			((TrajectoryChannel*)ch)->setCurveType(TRAJ_STEP);

	}
	motion_manager->updateCurves();
	ui_graph_viewer->redraw();
			break;
	case  evMakeChannelAdditive:
			if(ui_node_viewer->getSelectedNode())
			{
				ui_node_viewer->getSelectedNode()->setChannelMode(channel_additive);
			}
			else
			{
				for(int i=0;i<motion_manager->getSelectedChannels()->size();i++)
				{
					Channel* ch = motion_manager->getSelectedChannels()->get(i);
					ch->setChannelMode(channel_additive);

				}
			}
			motion_manager->updateCurves();
			ui_graph_viewer->redraw();
		
		break;
	case evMakeChannelInverse:
		{
			if(ui_node_viewer->getSelectedNode())
			{
				ui_node_viewer->getSelectedNode()->setChannelMode(channel_inverse);
			}
			else
			{
				for(int i=0;i<motion_manager->getSelectedChannels()->size();i++)
				{
					Channel* ch = motion_manager->getSelectedChannels()->get(i);
					ch->setChannelMode(channel_inverse);

				}
			}
			motion_manager->updateCurves();
			ui_graph_viewer->redraw();
		}break;
	case evMakeChannelFeedback:
		if(ui_node_viewer->getSelectedNode())
		{
			ui_node_viewer->getSelectedNode()->setChannelMode(channel_feedback);
		}
		else
		{
			for(int i=0;i<motion_manager->getSelectedChannels()->size();i++)
			{
				Channel* ch = motion_manager->getSelectedChannels()->get(i);
				ch->setChannelMode(channel_feedback);

			}
		}
		motion_manager->updateCurves();
		ui_graph_viewer->redraw();
		break;
		case 	evMakeChannelEvent:
			message("evMakeChannelEvent not implemented");
// 			for(int i=0;i<motion_manager->getSelectedChannels()->size();i++)
// 			{
// 				Channel* ch = motion_manager->getSelectedChannels()->get(i);
// 				ch->setP(channel_control_mode,control_event);
// 				ch->setCurveType(TRAJ_STEP);
// 			}
// 				
// 				motion_manager->updateCurves();
// 				ui_graph_viewer->redraw();
// 		
		break;
		case evMakeChannelIdler:
			message("evMakeChannelIdler not implemented");
// 			for(int i=0;i<motion_manager->getSelectedChannels()->size();i++)
// 			{
// 				Channel* ch = motion_manager->getSelectedChannels()->get(i);
// 				ch->setP(channel_control_mode,control_idler);
// 				ch->setCurveType(TRAJ_STEP);
// 			}
// 
// 			
// 				motion_manager->updateCurves();
// 				ui_graph_viewer->redraw();
// 		
		break;
	case evMakeChannelScale:
		if(ui_node_viewer->getSelectedNode())
		{
			ui_node_viewer->getSelectedNode()->setChannelMode(channel_multiply);
		}
		else
		{
			for(int i=0;i<motion_manager->getSelectedChannels()->size();i++)
			{
				Channel* ch = motion_manager->getSelectedChannels()->get(i);
				ch->setChannelMode(channel_multiply);

			}
		}
		motion_manager->updateCurves();
		ui_graph_viewer->redraw();

		break;

	case evChannelRename:
		{
			if(ui_node_viewer->getSelectedNode())
			{
				
				GsString s = ui_node_viewer->getSelectedNode()->name();
				if(fl_string_input("new channel name","enter channel",s))
				{
					ui_node_viewer->getSelectedNode()->setName(s);
					motion_manager->updateMotionData();
					refreshChannelList();
				}
			}
			else
			{
				if(motion_manager->getSelectedChannels()->size()==1)
				{
					GsString s = motion_manager->getSelectedChannels()->get(0)->name();
					if(fl_string_input("new channel name","enter channel",s))
					{
						phout<<s<<" entered\n";


						motion_manager->getSelectedChannels()->get(0)->setName(s);
						motion_manager->updateMotionData();
						
						refreshChannelList();

					}
				}
			}
		}
		break;
	case evEditChannel:
		{
			for(int it=0;it<motion_manager->getSelectedChannels()->size();it++)
			{
				loadParameterEditor(motion_manager->getSelectedChannels()->get(it));
			}
		}	break;
	
	case evGainAdjust:
		manager->selectedCharacter()->setGainMult((float)ui_p_mult->value(),(float)ui_d_mult->value(),(float)ui_psw_mult->value(),(float)ui_dsw_mult->value());
		break;
	case  evMovePoint:
		ui_curve_edit_mode->label("move");
		ui_graph_viewer->setEditMode(CURVE_MOVE_POINT);
		ui_graph_menu->redraw();
		break;
	case  evAddPoint:
		ui_curve_edit_mode->label("add");
		ui_graph_viewer->setEditMode(CURVE_ADD_POINT);
		ui_graph_menu->redraw();
		break;
	case  evDeletePoint:
		ui_curve_edit_mode->label("delete");
		ui_graph_viewer->setEditMode(CURVE_REMOVE_POINT);
		ui_graph_menu->redraw();
		break;
	case  evStraightenPoint:
		ui_curve_edit_mode->label("straighten");
		ui_graph_viewer->setEditMode(CURVE_STRAIGHTEN_TANGENT);
		ui_graph_menu->redraw();
		break;
	case  evFreePoint:
		ui_curve_edit_mode->label("free");
		ui_graph_viewer->setEditMode(CURVE_FREE_TANGENT);
		ui_graph_menu->redraw();
		break;
	case evEditBounds:
		ui_curve_edit_mode->label("bounds");
		ui_graph_viewer->setEditMode(CURVE_EDIT_BOUNDS);
		ui_graph_menu->redraw();
		break;
		
			

	case  evFlattenPoint:
		ui_curve_edit_mode->label("flatten");
		ui_graph_viewer->setEditMode(CURVE_FLATTEN_TANGENT);
		ui_graph_menu->redraw();
		break;

	case evStateCapture:
		if(manager->getStateManager()->getSelectedState())
			manager->getStateManager()->getSelectedState()->capture(human);

	break;
	case	evDrawMotion: motion_manager->drawMotion(); break;

	case evPrintMotion:
		if (motion_manager->currentMotion())
		{
			phout<<motion_manager->currentMotion()->toString(true);
		}
		break;

	case evMotionSel: motionSelected(); break;
	case evReloadController: motion_manager->reloadController(); break;
	case evPhaseSlider:
		
		motion_manager->setPhase((float)ui_graph_phase->value()); 
		if(motion_manager->currentMotion())
		{
			human->ik_module()->solve(false);
			if(manager->pBool(human_manager_graphics_active))
				human->redraw();
			ui_graph_viewer->redraw();
		}
		else if(motion_manager->getSelectedKnMotion())
		{
			//human->redraw();
			human->ik_module()->matchToSkeleton();
		}

		updateTimeUI();

		break;
	case 	 evMoveChannelUp:
		motion_manager->moveChannelUp();
		refreshChannelList();
		break;
	case evMoveChannelDown:
		motion_manager->moveChannelDown();
		refreshChannelList();
			break;
	case evDeleteChannels:	motion_manager->deleteChannels(); refreshChannelList(); ui_graph_viewer->redraw(); break;
	case evInitCurves:	motion_manager->initCurves(); ui_graph_viewer->redraw(); break;
	case evMakeSample: 
		motion_manager->makeSample(ui_makeSample->value()); 
		updateSampleUI();
		ui_graph_viewer->setP(graph_viewer_draw_bounds,true);
		ui_graph_viewer->redraw(); 
		break;
	case evPointEdit: 
		motion_manager->pointEdit(ui_curve_pt[0]->value(),ui_curve_pt[1]->value(), ui_pt[0]->value(),ui_pt[1]->value(),ui_pt[2]->value(),ui_pt[3]->value(),ui_pt[4]->value(),ui_pt[5]->value()); 
		ui_graph_viewer->redraw();
		break;
	case evRandomize: 
		motion_manager->randomize();
		ui_graph_viewer->redraw(); break;
	case evMakeLocalSample:
		{
			float v = 0.1f;
			if(	fl_value_input("make local sample bounds for this motion","enter tolerance to sample around existing points",v))
			{
				motion_manager->makeLocalSample(v);
				ui_graph_viewer->redraw(); 
			}	
		}break;
	case evRemoveMotion:
		ui_kn_motion_list->deselect();
		ui_motionList->deselect();

		motion_manager->deselectMotion();
		refreshChannelList();
		break;
	case evToggleCameraFollow:
		ui_viewer->toggleBool(camera_follow);
		break;
	case evSwitchStance:
		{
			if(human)
			{
				human->flipStanceState();
				motion_manager->currentMotion()->setStance(human->getStanceState());
				motion_manager->connectMotion(motion_manager->currentMotion());
				motion_manager->setPhase((float)ui_graph_phase->value());
		
				motion_manager->updateMotionTrajectories();
			}	
		}
		break;
	case evLoadPlanEditor:
			loadParameterEditor(manager->getPlanner(),"Planner");

	break;
	case evLoadGraphViewerEditor:
		loadParameterEditor(ui_graph_viewer,"GraphViewer");
		break;
	case evEditSelected:
		loadParameterEditor(human->getEditJoint(),human->getEditJoint()->name());
		break;
	case  evForceForward:
		human->addForce(GsVec(0.0f,0.0f,(float)ui_ext_force->value()),(float)ui_ext_force_duration->value(),human->getEditJoint()->getBody());
		break;
	case evForceBackward:
		human->addForce(GsVec(0.0f,0.0f,-(float)ui_ext_force->value()),(float)ui_ext_force_duration->value(),human->getEditJoint()->getBody());
		break;
	case	evForceRight:
		human->addForce(GsVec((float)ui_ext_force->value(),0.0f,0.0f),(float)ui_ext_force_duration->value(),human->getEditJoint()->getBody());
		break;
	case	evForceLeft:
		human->addForce(GsVec(-(float)ui_ext_force->value(),0.0f,0.0f),(float)ui_ext_force_duration->value(),human->getEditJoint()->getBody());
		break;
	case  evSimbiconAdjust:
		human->balance_module()->setP(balance_simbicon_gain_d,GsVec((float)ui_sim_d_x->value(),0.0f,(float)ui_sim_d_z->value()));
		human->balance_module()->setP(balance_simbicon_gain_v,GsVec((float)ui_sim_v_x->value(),0.0f,(float)ui_sim_v_z->value()));
		human->balance_module()->setP(balance_simbicon_root_scale,(float)ui_sim_root_scale->value());
	break;

	case evStepOde:
		manager->step();
		motion_manager->setP(human_motion_manager_playing,true);
		update();
	break;
	case evMergePoints:
		{
			motion_manager->mergeControlPoints((float)ui_curve_merge_tolerance->value());
			ui_graph_viewer->redraw();
		}break;
	case evCurveFit:
		{
			motion_manager->fitCurve((int)ui_curve_fit_points->value(),(float)ui_curve_fit_tolerance->value(),(float)ui_curve_merge_tolerance->value(),(float)ui_curve_conc_tolerance->value());
			
			refreshChannelList();

		}break;
	case 	evMotionResets:
		{
			motion_manager->resets(ui_motion_resets->value());
		}break;

	case	evMotionLoops:
		{
			motion_manager->loops(!motion_manager->loops());
			
		}break;

	case evCmdLine:
		{
			_cmd = ui_cmd_line->value();
			cmdAvailable = true;
			message(_cmd);
		}
		break;
	case evSegmentMotion:
		{
			segmentMotions();
		}break;
	case evSavePref:
		manager->processCmd("save");
		break;
	case evGeneralizeMotion:
		if(motion_manager->currentMotion())
		{
			motion_manager->currentMotion()->generalizeNames(STANCE_LEFT);
			refreshChannelList();
		}
		break;
	case evTranslateSegment:
		{
			if(motion_manager->getSelectedKnMotion())
				translateMotionToOrigin(motion_manager->getSelectedKnMotion(),human->reference_module()->pVec(reference_skeleton_position));

		}break;
	case evOrientSegment:
			{
				if(motion_manager->getSelectedKnMotion())
					orientMotionToOrigin(motion_manager->getSelectedKnMotion());

			}break;
		case evMirrorSegment:
		{
			if(motion_manager->getSelectedKnMotion())
				motion_manager->getSelectedKnMotion()->mirror("Left","Right");

		}break;
		case evKnMotionListSelected:
		{
			knMotionSelected();
		}break;
		case evStanceAdjust:
			{
				human->contact_module()->setStanceSwingRatio((float)ui_stanceSwingRatio->value());
				human->contact_module()->setP(contact_toe_heel_ratio,(float)ui_stanceToeHeel->value());
				human->contact_module()->setP(contact_stance_offset,GsVec((float)ui_contact_offset->value(),0.0f,(float)ui_contact_offset_z->value()));
				human->setP(human_desired_heading_delta, (float)ui_desired_heading->value());
				human->balance_module()->setP(balance_jcom_velocity_desired,GsVec(0.0f,0.0f,(float)ui_vel_desired_z->value()));
				human->setP(human_desired_v_scale,(float)ui_desired_v_scale->value());
			}break;
		case evControllerMatchFrame:
			{
				Module* c = human->getModule(selectedController());
				if(c->manipController())
				{
					((ManipulatorModule*)c)->matchFrameToHuman();
					((ManipulatorModule*)c)->evaluate();
				}
			}break;
		case evControllerMatchHm:
			{
				Module* c = human->getModule(selectedController());
				if(c->manipController())
				{
					((ManipulatorModule*)c)->matchToHuman();
				}

			}break;
		case 	evControllerMatchSk:
			{
			
				Module* c = human->getModule(selectedController());
				if(c->manipController())
				{
					((ManipulatorModule*)c)->matchToSkeleton();
				}
			}break;
		case  evControllerListSelected:
			{
				ui_manip_browser->remove_all();
				Module* c = human->getModule(selectedController());
				ui_controll_active->value(c->pBool(module_active));
				ui_controll_visible->value(c->pBool(module_visible));
				if(c->manipController())
				{
					ui_manip_group->show();
					ManipulatorModule* mc = (ManipulatorModule*)c;
					for(int i=0;i<mc->numManips();i++)
					{
						ui_manip_browser->add_leaf(mc->getManip(i)->name());
					}
					ui_manip_browser->add_leaf(mc->getFrameManip()->name());
				}
				else
				{
					ui_manip_group->hide();
				}
			}break;
		case evManipListSelected:
			{
				Manipulator* m = selectedManipulator();

				if(m)
				{
					ui_manip_active->value(m->pBool(manipulator_active));
					ui_manip_visible->value(m->pBool(manipulator_visible));
				}
				else
				{
					phout<<" no manip\n";
				}


			}break;

		case evManipMatchHm:
			{
				HumanManipulator* m = selectedManipulator();

				if(m)
				{
					PhysicalJoint* j = human->joint(m->name());
					if(j)
						m->match(j);
				}
			}break;
		case evManipMatchSk:
			{
				HumanManipulator* m = selectedManipulator();

				if(m)
				{
					KnJoint* j = human->skref()->joint(m->name());
					if(j)
						m->match(j);
				}
			}break;
		case evManipActive:
			{
				HumanManipulator* m = selectedManipulator();

				if(m)
				{
					m->setP(manipulator_active,ui_manip_active->value());
					m->applyParameters();
				}
			}break;
		case evManipVisible:
			{
				HumanManipulator* m = selectedManipulator();
				if(m)
				{
					m->setP(manipulator_visible,ui_manip_visible->value());
					m->applyParameters();
				}
			}break;
		case evManipEdit:
			{
				HumanManipulator* m = selectedManipulator();

				if(m)
				{
					loadParameterEditor(m);
				}
			}break;
		case evEditModule:
			{
				Module* c = human->getModule(selectedController());
				if(c)
					loadParameterEditor(c);
			}
			break;
		case evEditController:
			loadParameterEditor(motion_manager->currentController());
			break;
		case   evControllerActive:
			{
				Module* c = human->getModule(selectedController());
				if(c)
				{
					c->setP(module_active,ui_controll_active->value());
					c->applyParameters();
				}
			}
			break;
		case   evControllerVisible:
			{
				Module* c = human->getModule(selectedController());
				if(c)
				{
					c->setP(module_visible,ui_controll_visible->value());
					c->applyParameters();
				}
			}
			break;
		case evLoadSkeleton: 
			{
				GsInput ip;
				if(fl_open(ip,"find skeleton file ","*.s","../../models"))
				{
					phout<<"selected "<<ip.filename()<<gsnl;
				}
				return;
			//	manager->loadSkeleton(ui_skeleton_to_load->value()); 
				human = manager->selectedCharacter();
				ui_jointList->remove_all();
				for(int i=0;i<human->numJoints() ;i++)
				{
					ui_jointList->add_leaf(human->joint(i)->name());
				}
			}
			break;
		case evEditNodeViewer:
			loadParameterEditor(ui_node_viewer);
			break;
		case evJointParm:
			if(selectedJoints.size()==1)
			{
				loadParameterEditor(selectedJoints.get(0));
			}
			break;
		case evJointBody:
			if(selectedJoints.size()==1)
			{
				loadParameterEditor(selectedJoints.get(0)->getBody());
			}
			break;
		case evSkelWin:
			{
				
				FlSkeletonWin* win = new FlSkeletonWin;
				win->add(human->skref());
				win->show();
				human->ik_module()->setP(manip_module_match,true);
				human->puppet_module()->setP(manip_module_match,true);
				
			}
			break;
		case evLoadMotionManagerEditor:  loadParameterEditor(motion_manager); break;
		case  evLoadMainWinEditor: loadParameterEditor(this); break;
		case  evLoadViewerEditor: loadParameterEditor(ui_viewer); break;
		case 	evLoadManagerEditor:
			loadParameterEditor(manager); 
		break;
		case		evToggleVis:
			human->toggleBool(human_show_visual_geo);
			if(human->pBool(human_show_visual_geo))
			{	
				human->setP(human_show_collision_geo,false);
				human->setP(human_show_skeleton,false);
			}
			human->applyParameters();
			
		break;
		case evToggleSkeletonAxis:
			human->toggleBool(human_show_axis);
			human->applyParameters();
			break;
		case evToggleSkeletonDraw:
			human->toggleBool(human_show_skeleton);
			if(human->pBool(human_show_skeleton))
			{
				human->setP(human_show_collision_geo,false);
				human->setP(human_show_visual_geo,false);
			}
			human->applyParameters();
			break;
		case evDrawOriginalMotion:
		motion_manager->toggleBool(human_motion_manager_show_original_motion);
		motion_manager->applyParameters();
		break;
		case evToggleHeadingDraw:
			human->toggleBool(human_show_heading);
			human->applyParameters();
			break;
		case evToggleCollisionDraw:
			human->toggleBool(human_show_collision_geo);
			if(human->pBool(human_show_collision_geo))
			{
				human->setP(human_show_visual_geo,false);
				human->setP(human_show_skeleton,false);
			}
			human->applyParameters();
			break;
		case evToggleHumanDraw:
			human->getGroup()->visible(!human->getGroup()->visible());
		break;
		case  evLoadFileManagerEditor: loadParameterEditor(manager->getFiles()); break;
		case  evLoadODEWorldEditor: loadParameterEditor(manager->getWorld()); break;
		case  evLoadHumanEditor: loadParameterEditor(human); break;
		case evStateListSelected:
			{
				manager->getStateManager()->selectCharacterState(selectedState()); 
				message("loading state ",selectedState());
				setUIFromCharacter();
			}
		break;
		case	evStateReload:
			{
	
				manager->getStateManager()->selectCharacterState(selectedState(),true);
				//manager->step();
				message("reloading state ",selectedState());
				setUIFromCharacter();
			}
			break;
		case evSceneBrowse:
			loadScene(selectedScene());
			break;
		case evNodeVal:
			{
				if(ui_node_viewer->getSelectedNode())
				{
					ui_node_viewer->getSelectedNode()->setControlVal((float)ui_node_val->value());
					motion_manager->updateMotionTrajectories();
				}
			}
			break;
		case evRemoveDuplicates:
			motion_manager->removeDuplicates();
			break;
		case evOverrideCurveLook:
			ui_graph_viewer->overrideCurveLook();
			break;
		case evChannelSelect:
		{	//curve editor window events
			GsArray <Channel*>* cvs = getSelectedChannels();
			if(cvs->size()==1)
			{
				ui_channelMin->activate();
				ui_channelMax->activate();
				ui_channelRest->activate();
				ui_channelVal->activate();
				ui_channelVal->activate();
				ui_channelRep->activate();
				ui_channel_active->activate();

				ui_channelVal->value(cvs->get(0)->currentChannelVal());
				ui_channelMin->value (cvs->get(0)->cmin());
				ui_channelRest->value(cvs->get(0)->crest());
				ui_channelMax->value (cvs->get(0)->cmax());
				ui_channelRep->value (cvs->get(0)->creps());
				ui_channel_active->value(cvs->get(0)->pBool(channel_active));
			}
			else
			{
				ui_channel_active->deactivate();
				ui_channelVal->deactivate();
				ui_channelMin->deactivate();
				ui_channelMax->deactivate();
				ui_channelRest->deactivate();
				ui_channelRep->deactivate();
				message("more than one channel selected: ",cvs->size());
			}
			channelSel();

		}
			break;
		case evChannelEdit:
			{
				GsArray <Channel*>* cvs = getSelectedChannels();
				if(cvs->size()==1)
				{
					Channel* ch = cvs->get(0);
					ch->setP(channel_active,ui_channel_active->value());
					ch->setP(channel_range,(float)ui_channelMin->value(),0);
					ch->setP(channel_range,(float)ui_channelRest->value(),1);
					ch->setP(channel_range,(float)ui_channelMax->value(),2);
					ch->setCurrentChannelVal((float)ui_channelVal->value());
					ch->applyParameters();
					
				}
				else
				{
				}

				segmenter->setP(segment_motion_reduction_sample_points,(int)ui_curve_fit_points->value());
				segmenter->setP(segment_reduction_merge_distance,(float)ui_curve_merge_tolerance->value());
				segmenter->setP(segment_motion_reduction_slope_tolerance,(float)ui_curve_fit_tolerance->value());
				segmenter->setP(segment_motion_reduction_conc_tolerance,(float)ui_curve_conc_tolerance->value());


				if(motion_manager->currentMotion())
				{
					motion_manager->duration((float)ui_motion_duration->value());
				}

			}
			break;
		case evToggleInteractive:
			{
				gsout<<"disabled interactive mode\n";
				//motion_manager->setP(human_motion_manager_interactive_mode,!motion_manager->pBool(human_motion_manager_interactive_mode));
			}break;
		case evStateSnapshot:
			{

				ui_state_browser->add(manager->getStateManager()->snapshot()->getDirectory());
				ui_state_browser->select(ui_state_browser->children()-1);
			}
			break;
		case evMotionCaptureState:
			{
				gsout<<"disabled motion manager captureState\n";
				//motion_manager->captureState(true);
			}break;
		case evHideMotionEnv:
			{
				motion_manager->motionLinesVisible(false);
			}break;
		case evShowMotionEnv:
			{
				
				motion_manager->motionLinesVisible(true);
			}
			break;
		case evShowAllMotionEnv:
			{
				motion_manager->showAllMotionLines();
			}break;
		case evExpandEnv:
			{
				motion_manager->expandEnv();
			}break;
		case evAddEnv:
			{
				motion_manager->addEnv();
				motion_manager->loadMotionEnvironment();
				
			}break;
		
		case evStartExpandingEnv:
			{
				motion_manager->startExpandingEnv();
				
			}break;
		case evReduceMotionEnv:
			{
				motion_manager->currentMotion()->updateEnvHull();
				motion_manager->loadMotionEnvironment();
				motion_manager->goal_manip->visible(false);
			}break;
		case evStateRename:
			{
				showErrorWin("evStateRename is probably old check that the directory is still good");

				GsString dir = manager->getStateManager()->getSelectedState()->getDirectory();

				GsString fname = dir;
				remove_path(fname);
				GsString newN = fname;
				if(fl_string_input("new state name","enter new name for state",newN))
				{
					manager->getStateManager()->loadState(dir);
					manager->getStateManager()->deleteState(dir);
					files->deleteDirectory(dir);
					files->saveStateNamed(manager->selectedCharacter(),newN);
				}
				loadStateFiles();
			}break;
		case evDeleteMotion:
			{
				if(motion_manager->currentMotion())
				{
					GsString s = motion_manager->currentMotion()->getMotionName();
					GsString file = motion_manager->currentController()->getDirectoryName();
					file<<s<<".motion";
					files->deleteFile(file);
					loadMotionFiles();
				}
			}
			break;
		case evDeleteConfig:	

			break;
		case evDeleteState:	
			{
				GsString dir_name = manager->getStateManager()->getSelectedState()->getDirectory();
				
				GsString fileName = files->getStateFile(dir_name,"human");
				files->deleteFile(fileName);
				fileName = files->getStateDirectory(human->characterName());
				fileName << files->getStateFile(dir_name,"ik");
				files->deleteFile(fileName);
				fileName = files->getStateDirectory(human->characterName());
				fileName << files->getStateFile(dir_name,"joints");
				files->deleteFile(fileName);
				fileName = files->getStateDirectory(human->characterName());
				fileName << files->getStateFile(dir_name,"virtual");
				files->deleteFile(fileName);
				loadStateFiles();
			}
			break;

		case evSetControllerInitialState:
		
			if(motion_manager->currentMotion())
			{
				files->saveMotionStateNamed(human,"initial",current_controller->name());

				//files->saveMotionStateNamed(human,motionDirectory,"initial");
				//motionDirectory<<"initial";
			//	gsout<<"motion state name is "<<motionDirectory<<gsnl;
				GsString initalState = files->getMotionDirectory(human->characterName());
				initalState<<current_controller->name()<<SLASH<<"initial";
				manager->getStateManager()->loadState(initalState,true);
				//motion_manager->setStartState("none");
				setUIFromCharacter();
			}
		break;
		case evMotionPlay:
		
			if(motion_manager->pBool(human_motion_manager_interactive_mode))
			{
				if(motion_manager->resets())
				{
					motion_manager->resetAnimation();
				}
			//	motion_manager->setP(human_motion_manager_interactive_mode,false);
				motion_manager->setRunning(true);
				manager->setRunning(true);
				manager->setP(human_manager_graphics_active,true);
			}
			else
			{
				motion_manager->togglePlaying();
				manager->setP(human_manager_graphics_active,true);
				if(motion_manager->running())
				{
					if(motion_manager->resets())
					{
						motion_manager->resetAnimation();
					}
					motion_manager->setP(human_motion_manager_time,0.0f);
				}
			}
			
			break;
		case evMotionReset:
			motion_manager->resetAnimation();
			break;
		case evGraphWinKey:
			motion_manager->setKey();
			ui_graph_viewer->redraw();
			break;
		case evNewController:
			{

				GsString newControllerName = "NewController_";
				newControllerName<<randomString();
				if(fl_string_input("new controller name","enter controller name",newControllerName))
				{
					Controller* cont = motion_manager->newController(newControllerName);
					GsString dirName = files->makeControllerDirectory(manager->selectedCharacter()->characterName(),cont->name());
					gsout<<"dir name is "<<dirName<<gsnl;
					cont->setDirectoryName(dirName);
					
					HumanMotion* newMotion = new HumanMotion();
					
					newMotion->setMotionName(newControllerName);
					cont->pushMotion(newMotion);

					GsString baseName = dirName;

					baseName<<newControllerName<<".base";
					gsout<<"base name is "<<baseName<<gsnl;
					files->saveBaseMotion(baseName,newMotion);
					files->saveMotionTrajectories(files->getMotionFile(manager->selectedCharacter()->characterName(),newControllerName,newControllerName),newMotion);
					
					loadMotionFiles();

					for (int i=0;i<ui_motionList->children();i++)
					{
						if(ui_motionList->child(i)->label() == newControllerName)
						{
							ui_motionList->value(i);
						}
					}
					motionSelected();
				}
			}
			break;
		case evSaveMotion: 
			{
				
				if(!motion_manager->currentController())
				{
					manager->message("need to make or select a controller first");
					return;
				}

				HumanMotion* newMotion = motion_manager->currentMotion();
				GsString motionName = "NewMotion_";
				motionName<<randomString();
				if(fl_string_input("new motion name","enter motion name",motionName))
				{
					if(newMotion==0)
					{
						newMotion = new HumanMotion();
						message("new empty motion made");
						newMotion->setMotionName(motionName);
						motion_manager->currentController()->pushMotion(newMotion);
					}
					else
					{
						if(motionName!=newMotion->getMotionName())
						{
							GsString s = motion_manager->currentController()->getDirectoryName();
							s<<newMotion->getMotionName()<<".motion";
							phout<<"deleting directory "<<s<<gsnl;
							files->deleteFile(s);
							phout<<"changing motion name to "<<motionName<<gsnl;
							newMotion->setMotionName(motionName);
						}
					}
					GsString s = motion_manager->currentController()->getDirectoryName();
					s<<newMotion->getMotionName()<<".motion";
					gsout<<"new motion made "<<s<<gsnl;
					files->saveMotionTrajectories(s,newMotion);
					loadMotionFiles();
				}
			}
		break;
		case evDraw:
			manager->toggleBool(human_manager_graphics_active);
			break;
		case evRunOde: manager->setRunning(!manager->isRunning());  break;
		case evStop:
			manager->setRunning(false);  
			motion_manager->setP(human_motion_manager_playing,false);
			break;
		case evStart:
			manager->setRunning(true);  
		motion_manager->setP(human_motion_manager_playing,true);
			break;
	
		case evQuit: files->saveConfigurationNamed(human,"quit");  gs_exit();  break;
		case evDropBalls: manager->dropBalls(); break;
		case evResetState: 
			motion_manager->resetAnimation();
			manager->getStateManager()->reset();
			motion_manager->setRunning(false);
			//manager->getWorld()->reset();
		//	if(motion_manager->currentController() && motion_manager->pBool(human_motion_manager_resets))
			if(manager->pString(human_manager_scene)=="Editing")
			{
				env_builder->makeEnv();
			}
			else
				loadScene(manager->pString(human_manager_scene));
			setUIFromCharacter();
			updateTimeUI();
		break;
		case evOdeReset: 
			{
				loadScene("FlatGround");
				manager->resetState();
			
				motionSelected();	
				manager->getMotionManager()->setPhase(0);
			}break;
		
		case evStateNew:
			{
				GsString s = "new_state_";
				s<<randomString();
				if(fl_string_input("new state name","enter state",s))
				{
					files->saveStateNamed(human,s);
					loadStateFiles();
					
				}
			}
		break;
		case evSaveState:		
			{
				files->saveStateToDirectory(human,manager->getStateManager()->getSelectedState()->getDirectory());
				loadStateFiles();
			}
			break;

		
		case  evJointListSelect:  jointListSelected();   break;
		case evEulerJointAdjust: applyEulerAngles(); break;
		case  evIndividualJointAdjust: jointListModified();  break;
		case evSaveDepthBuffer:
			{
				GsImage img;
				ui_viewer->getDepthSnapShot(&img);
				img.save("depth.png");
			}
		break;
		case evMakePointCloud:
			{

				makePointCloud();
			}break;
		default:
			message("unknown event ",e); 
		break;
}

ui_viewer->redraw();

}
#include "ph_human.h"
#include "ph_motion_segmenter.h"
void HumanWindow::setUIFromCharacter(  )
{
	HumanMotionManager* motion_manager = manager->selectedCharacter()->getMotionManager();
	PhysicalHuman* human = manager->selectedCharacter();
	HumanMotionSegmenter* segmenter = manager->getMotionSegmenter();
	ui_curve_fit_points->value( segmenter->pInt(segment_motion_reduction_sample_points));
	ui_curve_merge_tolerance->value(segmenter->pFloat(segment_reduction_merge_distance));
	ui_curve_fit_tolerance->value(segmenter->pFloat(segment_motion_reduction_slope_tolerance));
	ui_curve_conc_tolerance->value(segmenter->pFloat(segment_motion_reduction_conc_tolerance));
	ui_p_mult->value(human->getGainPMult());
	ui_d_mult->value(human->getGainDMult());
	ui_psw_mult->value(human->getGainMultStance());
	ui_dsw_mult->value(human->getGainMultSwing());
	ui_stanceSwingRatio->value(human->contact_module()->pFloat(contact_stance_swing_ratio));
	ui_stanceToeHeel->value(human->contact_module()->pFloat(contact_toe_heel_ratio));
	ui_contact_offset->value(human->contact_module()->pVec(contact_stance_offset).x);
	ui_contact_offset_z->value(human->contact_module()->pVec(contact_stance_offset).z);
	ui_sim_root_scale->value(human->balance_module()->pFloat(balance_simbicon_root_scale));
	
	ui_sim_v_x->value(human->balance_module()->pVec(balance_simbicon_gain_v).x);
	ui_sim_v_z->value(human->balance_module()->pVec(balance_simbicon_gain_v).z);
	ui_sim_d_x->value(human->balance_module()->pVec(balance_simbicon_gain_d).x);
	ui_sim_d_z->value(human->balance_module()->pVec(balance_simbicon_gain_d).z);
	ui_vel_desired_z->value(human->balance_module()->pVec(balance_jcom_velocity_desired).z);
	ui_desired_heading->value(human->pFloat(human_desired_heading_delta));
	ui_selected_joint->value(human->getEditJoint()->name());
	ui_desired_v_scale->value(human->pFloat(human_desired_v_scale));

	 ui_motion_label->value(motion_manager->currentMotionName());
	 ui_state_label->value(manager->getStateManager()->currentStateName());
	 ui_rbds_height->value(motion_manager->pInt(human_motion_manager_rbds_max_neighbors));
	 ui_rbds_support->value(motion_manager->pFloat(human_motion_manager_rbds_support));
	 
	for(int i=0;i<4;i++)
	ui_sample_bounds[i]->value(manager->getPlanner()->pFloat(trajectory_planner_sample_bounds,i));

	ui_sample_bounds[4]->value(manager->getPlanner()->pInt(trajectory_planner_seed));
	if(manager->getEnvBuilder()->currentManip())
	{
		GsVec boxSize = manager->getEnvBuilder()->currentManip()->size();
		ui_env_size[0]->value(boxSize.x);
		ui_env_size[1]->value(boxSize.y);
		ui_env_size[2]->value(boxSize.z);
		EnvManipulator* manip = manager->getEnvBuilder()->currentManip();
		ui_env_dynamic->value(manip->dynamic);
		ui_env_unique->value(manip->uniqueProperties);
		if(manip->uniqueProperties)
		{
			
			ui_env_mu->show();
			ui_env_bounce->show();
			ui_env_density->show();

			ui_env_mu->value(manip->friction);
			ui_env_bounce->value(manip->bounce);
			ui_env_density->value(manip->density);
		}
		else
		{
			ui_env_mu->hide();
			ui_env_bounce->hide();
			ui_env_density->hide();
		}
	}
	
}

void HumanWindow::segmentMotions()
{
	phout<<"fix segment motions\n";
	/*
	PhysicalHuman* human = manager->selectedCharacter();
	HumanFileManager* files = manager->getFiles();
	HumanMotionManager* motion_manager = manager->getMotionManager();
	HumanMotionSegmenter* segmenter = manager->getMotionSegmenter();

	ui_motion_group_list->remove_all();
	segmenter->processMotions("man_walks.bvh");


	GsArray<kn_motion_segment*>* kn_segs = &segmenter->kn_segments;
	GsArray<HumanMotion*>* mo_segs = &segmenter->motion_segments;
	//GsArray<HumanMotion*>* simple_segs = &segmenter->simple_segments;
	HumanMotion* default_control = motion_manager->getDefaultControlMotion();
	if(kn_segs->size() != mo_segs->size() )
	{
		message("the groups aren't the same size");
		return;
	}
	//phout<<"saving "<<kn_segs->size()<< " groups\n	";
	motion_manager->initMotionGroups();

	for (int i=0;i<kn_segs->size();i++)
	{
		HumanMotion* cm = new HumanMotion(*default_control);
		int id = motion_manager->makeGroupNamed(kn_segs->get(i)->motion->name(),kn_segs->get(i)->motion,mo_segs->get(i),cm);
		files->saveMotionGroup(motion_manager->getMotionGroup(id));
	}

	loadMotionFiles();
	*/
}
