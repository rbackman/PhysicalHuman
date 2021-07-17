
# include <gsim/fl.h>
# include "phw_window.h"


#include "util_serializable_editor.h"
#include "ph_motion_manager.h"
#include "ph_manager.h"
#include "ph_mod_manip.h"
#include "ph_mod_ik.h"
#include "ph_file_manager.h"
#include "ph_motion.h"
#include "ph_state_manager.h"
#include "util_channel_traj.h"
#include "ph_trajectory_planner.h"
#include <gsim/gs_scandir.h>
#include "ph_env_builder.h"
#include "ph_char_builder.h"

#include <gsim/sn_points.h>
#include <gsim/gs_ogl.h>

GsString HumanWindow::selectedScene()
{
return ui_scene_browser->child(ui_scene_browser->value())->label();
}
GsString HumanWindow::selectedState()
{
return ui_state_browser->child(ui_state_browser->value())->label();
}

void HumanWindow::loadParameterEditor(Serializable* sav,const GsString& label)
{
	for(int i=0;i<editors.size();i++)
	{
		if(editors.get(i)->getSerializable() == sav ) 
		{
			 editors.get(i)->show();
			return;
		}
	}

	SerializableEditor* editor = new SerializableEditor(sav);
	editor->ui_serializable_editor->label(label);
	editor->show();
	editors.push(editor);
}
void HumanWindow::loadNodeParameterEditor( Serializable* sav ,const GsString& lab)
{
	SerializableEditor* editor = new SerializableEditor(sav,true);
	editor->ui_serializable_editor->label(lab);
	editor->show();
	editors.push(editor);
}
HumanManipulator* HumanWindow::selectedManipulator()
{
	Module* c = manager->selectedCharacter()->getModule(selectedController());
	if(c)
	{
		if(c->manipController())
		{
			GsString s = "selectedManipulator():";
			s<<c->name();
			message(s);
			return ((ManipulatorModule*)c)->getManip(ui_manip_browser->value());

		}
	}
	return 0;
}

 HumanWindow::HumanWindow (HumanManager* mgr,const GsString& file) : Serializable("HumanWindow")
 {
	 manager  = mgr;


	GsString testMes = "hey there";
	// testMes<<48<<" yo";
	// message(testMes);
	// message(file.pt());
	_point_cloud = 0;;
	 ui_viewer->init ( this , file );	
	
	 ui_graph_viewer->init(this,manager->getMotionManager(),file);
	 ui_node_viewer->init(this,manager->getMotionManager(),file);
	 cmdAvailable = false;
	loadSceneFiles();
	loadCharacterList();

 }
HumanWindow::~HumanWindow ()
 {
 }

void HumanWindow::selectChannel(const GsString& chName)
{
	//ui_channelList->clear_selected();

	for (int i=0;i<ui_channelList->size();i++)
	{
		if(ui_channelList->child(i)->label() == chName)
		{
			ui_channelList->select(i); //  child(i)->set_selected();
		}
		else
		{
			ui_channelList->select(i,false);
		}
	}
}
GsArray<Channel*>* HumanWindow::getSelectedChannels()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();

	motion_manager->unselectCurves();
	if(motion_manager->currentMotion())
	{
		fltk::Browser* b =  ui_channelList;
		HumanMotion* m = motion_manager->currentMotion();
		if(m)
		{
			for(int it=0;it<b->children();it++)
			{

				Channel* cv =   m->getChannel(b->child(it)->label()); 
				if(b->child(it)->selected())
				{

					if(cv)
					{
						cv->showCurve();
						cv->showNode();
						motion_manager->selectCurve(cv);
					}
				}
				else
				{
					if(cv)
					{
						cv->hideCurve();
						cv->hideNode();
					}
				}
			}
		}
	}
	return motion_manager->getSelectedChannels();
}

GsString HumanWindow::getSelectedChannel()
{
	fltk::Browser* b =  ui_channelList;
	for(int it=0;it<b->children();it++)
	{
		if(b->child(it)->selected())
			return GsString(b->child(it)->label());
	}
	phout<<"nothing selected"<<gsnl;
	return "NULL";
}

void HumanWindow::deleteCurves()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;
	motion_manager->deleteSelectedCurves();
	 ui_channelList->clear();
refreshChannelList();
}
bool HumanWindow::channelSelected(const char* name)
{
	fltk::Browser* b =  ui_channelList;
	for(int it=0;it<b->children();it++)
	{
		if(b->child(it)->selected())
		{
			if(GsString::compare(name,b->child(it)->label()) ==0)
			{
				return true;
			}
		}

	}
	return false;
}



void HumanWindow::channelSel()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;
	//just update the selected curve (scvs) array and redraw
	for (int i=0;i<motion_manager->currentMotion()->numChannels();i++)
	{
		if(motion_manager->currentMotion()->getChannel(i)->isTrajectory())
		{
			((TrajectoryChannel*)motion_manager->currentMotion()->getChannel(i))->getCurve()->selectionState = CURVE_NOT_SELECTED;
		}
	}
	ui_curve_pt[0]->deactivate();
	ui_curve_pt[1]->deactivate();
	if(getSelectedChannels()->size()==1)
	{
		if(getSelectedChannels()->get(0)->isTrajectory())
		{
			ui_curve_pt[0]->activate();
			ui_curve_pt[1]->activate();
			Trajectory* cv = ((TrajectoryChannel*)getSelectedChannels()->get(0))->getCurve();
			if(cv->selectionState == CURVE_POINT_SELECTED)
			{
				ui_curve_pt[0]->value(cv->getPoint(cv->selection).x);
				ui_curve_pt[1]->value(cv->getPoint(cv->selection).y);
			}
			else
			{
				ui_curve_pt[0]->value(0);
				ui_curve_pt[1]->value(0);
			}
		}
	
	}

	ui_graph_viewer->draw();
}


void HumanWindow::jointListSelected()
{
	   message("jointListSelected()");
	int count = 0;
	GsString name;

	PhysicalHuman* human = manager->selectedCharacter();

	selectedJoints.remove(0,selectedJoints.size());
	selectedJointIdx.remove(0,selectedJointIdx.size());	
	if(!human)
	{
			ui_jointList->remove_all();
			return;
	}

	for(int i=0;i<ui_jointList->children();i++)
		   {
			   if(ui_jointList->child(i)->selected())
			   {	
				   selectedJointIdx.push(i);
				   count++;
				   name = ui_jointList->child(i)->label();
					PhysicalJoint* j = human->joint(name);
					if(j)
					{
						selectedJoints.push(j);
						ui_multiSelect->value(j->name());
						ui_jointGrav->value(j->pBool(joint_grav_comp));
						ui_jointJX->value(j->pVec(joint_vf_scale).x);
						ui_jointJY->value(j->pVec(joint_vf_scale).y);
						ui_jointJZ->value(j->pVec(joint_vf_scale).z);
						ui_jointMaxT->value(j->pFloat(joint_max_t));
						ui_char_frame->value(j->pBool(joint_char_frame));
						ui_jointP->value(j->pFloat(joint_gain_p));
						ui_jointD->value(j->pFloat(joint_gain_d));
						ui_jointPD->value(j->pBool(joint_use_pd));
						ui_joint_scale[0]->value(j->pVec(joint_pd_scale).x);
						ui_joint_scale[1]->value(j->pVec(joint_pd_scale).y);
						ui_joint_scale[2]->value(j->pVec(joint_pd_scale).z);
					}
		
			   }

		}
	if(count>1)
	{
		ui_joint_type->value(" ");
		ui_multiSelect->value("Multiple Joints");
	}
	else if(count==1)
	{
		PhysicalJoint* j = human->joint(name);
		KnJoint* jn = human->skref()->joint(name);
		if(j&&jn)
		{
			ui_jointRX->value(GS_TODEG( jn->euler()->value(0)));
			ui_jointRY->value(GS_TODEG( jn->euler()->value(1)));
			ui_jointRZ->value(GS_TODEG( jn->euler()->value(2)));	
			ui_joint_type->value(j->jointTypeName());
		}
	}
	
	//ui_multiSelect->redraw();
}

void HumanWindow::refreshScene()
{
	
	for(int i=0;i<editors.size();i++)
	{
		delete editors.get(i);
	}editors.remove(0,editors.size());

	PhysicalHuman* human = manager->selectedCharacter();

	if(human->pBool(human_models_shaded)) 
		ui_viewer->cmd(FlViewer::CmdAsIs);
	else 
		ui_viewer->cmd(FlViewer::CmdLines);

	for(int i=0;i<selectedJointIdx.size();i++)
	{
		ui_jointList->select( selectedJointIdx.get(i) );
	}
}


#include "ph_mod_ref.h"
void HumanWindow::update()
{
	fltk::check();
	
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

	if(manager->getPlanner()->running())
	{
		ui_num_found->show();
		ui_num_found->value(manager->getPlanner()->_numFound);
	}
	else
	{
		ui_num_found->hide();
	}
	
	PhysicalHuman* human = manager->selectedCharacter();

	for(int i=0;i<editors.size();i++)
	{
		editors.get(i)->update();
		if(editors.get(i)->wantsChannel())
		{
			GsString nname = editors.get(i)->getSerializable()->name();
			nname<<"_"<<channelTypeToString(editors[i]->getSelectedDof());

			ui_node_create_window->show();
			ui_node_create_name->value(nname);
			Serializable* s = editors.get(i)->getSerializable();

			ui_node_create_serializable->value(s->name());
			ui_node_create_serializable_type->value(s->type());
			ui_node_create_parameter->value(s->getParameter(editors.get(i)->getParamaterIdx())->name);
			ui_node_create_parameter_index->value(editors.get(i)->getDofArrayIdx());
			ui_node_create_dof->value(channelTypeToString(editors[i]->getSelectedDof()));

			ui_node_create_window->show();
		
			//ui_node_viewer->addNode(editors.get(i)->getSerializable(),editors.get(i)->getParamaterIdx(),editors[i]->getSelectedDof(),);

			
		}
	}
	if (ui_viewer->pBool(camera_follow))
	{
		viewCharacter();
	}
	if(manager->getEnvBuilder()->manipClicked())
	{
		setUIFromCharacter();
	}
	if(manager->getCharBuilder()->manipClicked())
	{
		setUIFromCharacter();
	}

	if(motion_manager->verifyingMotions() && motion_manager->currentController())
	{
		ui_status_bar->maximum(motion_manager->currentController()->numMotions());
		ui_status_bar->minimum(0);
		if(motion_manager->currentController()->currentMotionID()>0)
			ui_status_bar->position(motion_manager->currentController()->currentMotionID());
		if(!motion_manager->currentController()->verifyingAllMotions())
		{
			
			motion_manager->setRunning(false);
	
			GsStrings choices;
			GsStrings* _motions_to_delete = &motion_manager->currentController()->_motions_to_delete;
			choices.push("Delete All Files");
			choices.push("keep files");
			for (int i=0;i<_motions_to_delete->size();i++)
			{
				GsString ms = "Delete ";
				ms<<_motions_to_delete->get(i);
				choices.push(ms);
			}
			int choice = fl_choice_input("delete all files?","delete all files?",choices);
			if(choice != -1)
			{
				if(choice == 0)
				{
					for (int i=0;i<_motions_to_delete->size();i++)
					{
						GsString s = _motions_to_delete->get(i);
						GsString file = motion_manager->currentController()->getDirectoryName();
						file<<s<<".motion";
						manager->getFiles()->deleteFile(file);
					}
					return;
				}
				if(choice == 1)
				{

				}
				else
				{
					GsString s = _motions_to_delete->get(choice);
					GsString file = motion_manager->currentController()->getDirectoryName();
					file<<s<<".motion";
					manager->getFiles()->deleteFile(file);
				}
			}
			
		}
		//loadMotionFiles();

	}
	if(motion_manager->analyzingMotions())
	{
		ui_status_bar->maximum(motion_manager->currentController()->numMotions());
		ui_status_bar->minimum(0);
		ui_status_bar->position(motion_manager->currentController()->currentMotionID());
		if(!motion_manager->currentController()->analyzingAllMotions())
		{

			motion_manager->setRunning(false);
			phout<<"done analyzing so should rename all the motions with new parm names";

			GsString ans = "all";
			if(fl_string_input("rename all","enter all to rename all files",ans))
			{
				for (int i=0;i<motion_manager->currentController()->numMotions();i++)
				{
					manager->renameMotion(i);
				}
			}			
			

		}
		loadMotionFiles();

	}
	if(motion_manager->running())
	{
		if(ui_show_time->value())
		{
			ui_current_time->value(motion_manager->getTime());
		}
		else
		{
			ui_current_frame->value(motion_manager->getFrame());
		}
	}

	if(motion_manager->pBool(human_motion_manager_interactive_mode))
	{
		ui_graph_phase->when(fltk::WHEN_RELEASE_ALWAYS);
	}
	else
	{
		ui_graph_phase->when(fltk::WHEN_CHANGED);
		if(manager->getMotionManager()->running())
			ui_graph_phase->value(manager->getMotionManager()->getPhase()); 
	}

	ui_interactive_edit->value(motion_manager->pBool(human_motion_manager_interactive_mode));
	ui_ode_run->value(manager->isRunning());
	ui_draw->value(manager->pBool(human_manager_graphics_active));
	ui_motion_loops->value(motion_manager->loops());
	ui_motion_resets->value(motion_manager->resets());
	if(manager->hasMessage())
	{
		message(manager->getMessage());
	}
	if(ui_feedback->value())
	{
		if(selectedJoints.size()==1)
		{
			PhysicalJoint* j = selectedJoints.get(0);
			KnJoint* jt = human->skref()->joint(j->name());
			if(jt)
			{
				ui_jointRX->value(jt->euler()->value(0));
				ui_jointRY->value(jt->euler()->value(1));
				ui_jointRZ->value(jt->euler()->value(2));
			}
		}
		
	}
	if(manager->pBool(human_manager_graphics_active))
	{
		ui_graph_viewer->redraw();
		if(ui_graph_viewer->visible())
			ui_node_viewer->redraw();
		ui_viewer->redraw();
		
	}
}
GsArray<PhysicalJoint*> HumanWindow::jointsSelected()
{
	return selectedJoints;

	
}
void HumanWindow::applyEulerAngles()
{
	PhysicalHuman* human = manager->selectedCharacter();
	 if(selectedJoints.size()==1)
	{
		GsString name = selectedJoints.get(0)->name();
		PhysicalJoint* j = human->joint(name);

		GsQuat  q =	vecToQuat(GsVec(GS_TORAD( ui_jointRX->value()),GS_TORAD( ui_jointRY->value()),GS_TORAD( ui_jointRZ->value())),rotation_type_to_gs_euler_order(j->rotationOrder()));

		human->skref()->joint(name)->quat()->value(q);
		human->ik_module()->matchToSkeleton();
		human->redraw();
	}
}

void HumanWindow::jointListModified()
{
	PhysicalHuman* human = manager->selectedCharacter();
	 if(!human)return;

	 GsString name;
	
	 
	 for(int i=0;i<selectedJoints.size();i++)
	 {
		 PhysicalJoint* j = selectedJoints.get(i);

		 j->setP(joint_grav_comp,ui_jointGrav->value());
		 j->setP(joint_char_frame,ui_char_frame->value());
		 j->setP(joint_vf_scale,GsVec((float)ui_jointJX->value(),(float)ui_jointJY->value(),(float)ui_jointJZ->value()));
		 j->setP(joint_pd_scale,GsVec((float)ui_joint_scale[0]->value(),(float)ui_joint_scale[1]->value(),(float)ui_joint_scale[2]->value()));
		 j->setP(joint_max_t,(float)ui_jointMaxT->value());
		 j->setP(joint_gain_p,(float)ui_jointP->value());
		 j->setP(joint_gain_d,(float)ui_jointD->value());
		 j->setP(joint_use_pd,ui_jointPD->value());
	 }
	
}
void HumanWindow::show ()
 {
	setUIFromCharacter();
	ui_window->show();
	ui_channelMin->deactivate();
	ui_channelMax->deactivate();
	ui_channelRest->deactivate();
	ui_channelVal->deactivate();
	ui_channelRep->deactivate();
	ui_channel_active->deactivate();
	ui_env_offset[0]->value(manager->envOffY);
	ui_env_offset[1]->value(manager->envOffZ);
}



GsString HumanWindow::selectedController()
{
	return GsString(ui_controller_browser->child(ui_controller_browser->value())->label());	
}

GsString HumanWindow::selectedConfig()
{

	return GsString(ui_character_list->child(ui_character_list->value())->label());
}



void HumanWindow::handle_viewer_event( const GsEvent & e )
{
	
	if(e.type == GsEvent::Push)
	{
		if(ui_select_mode->value())
		{
			Serializable* s = manager->checkRay(e.ray);
			if (s)
			{

				if(s->type()=="PhysicalJoint")
				{
					manager->selectedCharacter()->setEditJoint((PhysicalJoint*)s);
					ui_selected_joint->value(s->name());
				}
				else
				{
					phout<<"loading editor for "<<s->name()<<gsnl;
					loadParameterEditor(s);
				}
			}
		}
		else if(ui_pick_mode->value())
		{

			GsVec2 mp = e.mouse;
			mp.x = (mp.x+1)/2.0f*ui_viewer->w();
			mp.y = ui_viewer->h() - (mp.y+1)/2.0f*ui_viewer->h();
			GsVec p = glUnProject((int)mp.x,(int)mp.y);
			//phout<<"HumanWindow::handle_viewer_event()::clicked in scene at position "<<p<<gsnl;
			/*
			Ball* b = new Ball(p,0.01f);
			manager->getRoot()->add(b->getGrp());
			*/
		}
	}
}
void HumanWindow::loadControllers()
{
	PhysicalHuman* human = manager->selectedCharacter();
	ui_controller_browser->remove_all();
	
	if(human)
	{
		for(int i=0;i<human->numModules();i++)
		{
			ui_controller_browser->add_leaf(human->getModule(i)->name());
		}	
	}
}
void HumanWindow::loadStateFiles()
{
	if(!manager->selectedCharacter())
	{
		gsout<<"must load character before states\n";
		return;
	}

	ui_state_browser->remove_all();
	GsStrings files;
	GsStrings dir;
	GsStrings ext;
	ext.push("state");
	GsString motion_dir = manager->getFiles()->getStateDirectory(manager->selectedCharacter()->characterName());
	gs_scandir(motion_dir,dir,files,ext);
	
	for (int i=0;i<dir.size();i++)
	{
		GsString s = dir.get(i);
		s.replace(motion_dir,"");
		ui_state_browser->add(s);
	}

	for (int i=0;i<ui_state_browser->children();i++)
	{
		//gsout<<"browser name "<<ui_state_browser->child(i)->label()<<" state name "<<manager->getStateManager()->getSelectedState()->stateShortName()<<gsnl;
		if(ui_state_browser->child(i)->label() == manager->getStateManager()->getSelectedState()->stateShortName())
			ui_state_browser->select(i);
	}
}
void HumanWindow::loadCharacterList()
{
	ui_character_list->remove_all();
	GsStrings dir;
	gsout<<"loading characters from "<<manager->getFiles()->getCharacterDirectory()<<gsnl;
	manager->getFiles()->getDirectories(manager->getFiles()->getCharacterDirectory(),dir);
	for (int i=0;i<dir.size();i++)
	{
		GsString d = dir[i];
		remove_path(d);
		//gsout<<"adding character "<<d<<gsnl;
		ui_character_list->add_leaf(d);
	}

	
}
void HumanWindow::loadSceneFiles()
{
	ui_scene_browser->remove_all();
	GsStrings files;
	GsStrings dir;
	GsStrings ext;
	ext.push("scene");
	GsString scene_dir = manager->getFiles()->getSceneDirectory();
	gs_scandir(scene_dir,dir,files,ext);

	for (int j=0;j<files.size();j++)
	{
 			GsString m = files[j];
 			remove_extension(m);
 			remove_path(m);
 			ui_scene_browser->add(m,0,0);
	}
	for (int i=0;i<manager->getFiles()->sizeOfParameter(files_scene_list);i++)
	{
		ui_scene_browser->add(manager->getFiles()->pString(files_scene_list,i),0,0);
	}
// work on later to load scene directories
//  	for (int i=0;i<dir.size();i++)
//  	{
//  		GsString s = dir.get(i);
//  		s.replace(scene_dir,"");
//  
//  
//  		//ui_motionList->add(s);
//  		GsStrings motions;
//  		GsStrings mdir;
//  
//  		gs_scandir(dir.get(i),mdir,motions,ext);
//  		for (int j=0;j<motions.size();j++)
//  		{
//  			GsString m = motions[j];
//  			remove_extension(m);
//  			remove_path(m);
//  			GsString name = s;
//  			name<<"/"<<m;
//  			ui_motionList->add(name,0,0);
//  		}
//  
//  	}
	ui_motionList->deselect(0);
}
void HumanWindow::loadMotionFiles()
{
	if(!manager->selectedCharacter())
	{
		gsout<<"no character loaded\n";
		return;
	}
	{
		ui_motionList->remove_all();
		GsStrings files;
		GsStrings dir;
		GsStrings ext;
		ext.push("motion");
		GsString motion_dir = manager->getFiles()->getMotionDirectory(manager->selectedCharacter()->characterName());
		gs_scandir(motion_dir,dir,files,ext);
		
		for (int i=0;i<dir.size();i++)
		{
			GsString s = dir.get(i);
			s.replace(motion_dir,"");
			

			//ui_motionList->add(s);
			GsStrings motions;
			GsStrings mdir;
		
			gs_scandir(dir.get(i),mdir,motions,ext);
			for (int j=0;j<motions.size();j++)
			{
				GsString m = motions[j];
				remove_extension(m);
				remove_path(m);
				GsString name = s;
				name<<"/"<<m;
				ui_motionList->add(name,0,0);
			}
			
		}
		ui_motionList->deselect(0);
	}
	
	{
		ui_kn_motion_list->remove_all();
		GsStrings files;
		GsStrings dir;
		GsStrings ext;
		ext.push("bvh");
		GsString motion_dir =  manager->getFiles()->getKinematicsDirectory();
		gs_scandir(motion_dir,dir,files,ext);
	
		for (int i=0;i<files.size();i++)
		{
			GsString s = files.get(i);
			s.replace(motion_dir,"");
			//phout<<s<<gsnl;
			ui_kn_motion_list->add(s);
		}
		ui_kn_motion_list->deselect(0);
	}

	{
		GsStrings files;
		GsStrings dir;
		GsStrings ext;
		ext.push("bvh");
		GsString cmu_dir = manager->getFiles()->getKinematicsFile("cmu/");
		gs_scandir(cmu_dir,dir,files,ext);
		for (int i=0;i<dir.size();i++)
		{
			GsString subd = dir[i];
			GsString dirName = subd;
			dirName.replace(cmu_dir,"");
			GsStrings files_sub;
			GsStrings dir_sub;
			gs_scandir(subd,dir_sub,files_sub,ext);
			for(int j=0;j<files_sub.size();j++)
			{
				GsString fileName = files_sub[j];
				fileName.replace(subd,"");
				GsString listName = "cmu/";
				listName<<dirName;
				listName<<fileName;
				ui_kn_motion_list->add(listName,0,0);
			}
		}
		for (int i=0;i<files.size();i++)
		{
			phout<<files.get(i)<<gsnl;
		}
	}

	
}

void HumanWindow::loadConfigurationNamed( const GsString& configName ,bool forcereload)
{
	PhysicalHuman* character = manager->getCharacter(configName);
	
	
	if(character)
	{
		if(forcereload)
		{
			manager->getRoot()->remove(character->getGroup());
			character->loadConfiguration(configName);
			manager->setCharacter(configName);
			manager->resetState();
			manager->getRoot()->add(character->getGroup());
		}
		else
		{
			manager->setCharacter(configName);
		}
	}
	else
	{
		
		if(!ui_mult_characters->value())
		{
			manager->clearCharacters();
		}

		message("loading configuration:",configName);
		character = new PhysicalHuman(manager,configName);
		character->init();
		manager->pushCharacter(character);
		manager->getStateManager()->loadDefaultState();
	}
	
	manager->showSelectedController();
	
	ui_jointList->remove_all();
	for(int i=0;i<character->numJoints() ;i++)
	{
		ui_jointList->add_leaf(character->joint(i)->name());
	}

	setUIFromCharacter();
	loadControllers();
	loadMotionFiles();
	loadStateFiles();
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;
	ui_control_motion->value(motion_manager->pString(human_motion_manager_default_control_file));
	for(int i=0;i<ui_character_list->size();i++)
	{
		if(ui_character_list->child(i)->label() == configName)
			ui_character_list->select(i);
	}
	ui_character_list->redraw();
}

void HumanWindow::viewCharacter()
{
	PhysicalHuman* human = manager->selectedCharacter();
	if(human->pBool(human_show_collision_geo))
	{
		ui_viewer->view_node(human->getCollisionGroup(),ui_viewer->pFloat(camera_fovy));
	}
	else
		ui_viewer->view_node(human->getVisSkelScene(),ui_viewer->pFloat(camera_fovy));
}

void HumanWindow::updateTimeUI()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

	ui_current_time->value(motion_manager->getTime());
 	ui_first_time->value(motion_manager->startTime());
 	ui_last_time->value(motion_manager->endTime());
	
	ui_current_frame->value(motion_manager->getFrame());
 	ui_first_frame->value(motion_manager->startFrame());
 	ui_last_frame->value(motion_manager->endFrame());

}

void HumanWindow::updateSampleUI()
{

	if(getSelectedChannels()->size()==1)
	{
		Channel* ch =	getSelectedChannels()->get(0);
		int foundSample = -1;
		if(ch->isTrajectory())
		{
			Trajectory* cv = ((TrajectoryChannel*) ch)->getCurve();
			ui_pt[0]->activate();
			ui_pt[1]->activate();
			ui_makeSample->activate();
			if(cv->selectionState == CURVE_POINT_SELECTED)
			{
				ui_curve_pt[0]->value(cv->getPoint(cv->selection).x);
				ui_curve_pt[1]->value(cv->getPoint(cv->selection).y);
			}
			if(((TrajectoryChannel*) ch)->samples())
			{
				
				
				for(int j=0;j<cv->numSamplePoints();j++)
				{
					if(cv->sample(j)->index == cv->selection)
					{
						foundSample = j;
					}
				}
			}
			
			if(foundSample==-1)
			{
				ui_makeSample->value(0);
				for(int k=0;k<6;k++)
				{
					ui_pt[k]->hide();
				}
			}
			else
			{
				ui_makeSample->value(1);
				for(int k=0;k<6;k++)
				{
					ui_pt[k]->activate();
					ui_pt[k]->show();
				}

				ui_pt[0]->value(cv->sample(foundSample)->rest.x);
				ui_pt[1]->value(cv->sample(foundSample)->rest.y);
				ui_pt[2]->value(cv->sample(foundSample)->min.x);
				ui_pt[3]->value(cv->sample(foundSample)->max.x);
				ui_pt[4]->value(cv->sample(foundSample)->min.y);
				ui_pt[5]->value(cv->sample(foundSample)->max.y);

			}


			
		}
		else
		{
			ui_makeSample->deactivate();
		}
	}
}

void HumanWindow::depthSnapshot( GsImage& img )
{

# ifdef GS_OPENGL
		int vp[4];

		glGetIntegerv ( GL_VIEWPORT, vp );

		int x = vp[0];
		int y = vp[1];
		int w = vp[2];//-x;
		int h = vp[3];//-y;

		img.init ( w, h );

	//	glReadBuffer ( GL_FRONT );
	//	glPixelStorei ( GL_UNPACK_ALIGNMENT, 1 );
	//	glPixelStorei ( GL_PACK_ALIGNMENT, 1 );

		
		GsBuffer<float> data; // array of rgba values
		data.size(w*h);

		glReadPixels( 0, 0, w, h,
			GL_DEPTH_COMPONENT, // GLenum format
			GL_FLOAT, //GLenum type
			(void*)&data[0] //GLvoid *pixels   
			);

		float min = 1000;
		float max = -1000;

		for (int i=0;i<data.size();i++)
		{
			if (data[i]<1 && data[i]>0)
			{
				if(data[i] >max)
					max = data[i];
				if(data[i]<min)
					min = data[i];
			}
		}
		phout<<"min "<<min<<" max "<<max<<gsnl;
		float range = max-min;
		if(range == 0)
		{
			phout<<"range is zero so no image for you\n";
			return;
		}
		for (int i=0;i<w;i++)
		{
			for(int j=0;j<h;j++)
			{
				int id = j*w+i;
				if(id<data.size())
				{
					float dist =  (data[id]-min)/range;
					//if(dist!=0 && dist != 1)
					//	dist = dist/range;

					img.ptpixel(h,j)->set(dist,dist,dist);
	//				img.pixel(h,i).set(dist,dist,dist);
				}
				else
				{
					phout<<i<<" "<<" "<<j<<" out of bounds\n";
				}
				
			}
		}
	
		img.vertical_mirror ();

		// Restore default values:
	//	glPixelStorei ( GL_UNPACK_ALIGNMENT, 4 );
	//	glPixelStorei ( GL_PACK_ALIGNMENT, 4 );
# endif
	
}

void HumanWindow::makePointCloud()
{
	if(_point_cloud==0)
	{
		_point_cloud = new SnPoints;
		manager->getRoot()->add(_point_cloud);
	}
	_point_cloud->init();
	GsArray<GsVec> points;
	GsArray<float> zbuff;
	glGetPointCloud(points,zbuff);
	for (int i=0;i<points.size();i++)
	{
		_point_cloud->push(points[i]);
	}
}

void HumanWindow::showErrorWin(const char* error )
{
	ui_error_message->value(error);
	ui_error_win->show();
}

void HumanWindow::loadScene( GsString scene )
{
	manager->loadScene(scene);
	for (int i=0;i<ui_scene_browser->children();i++)
	{
		if(ui_scene_browser->child(i)->label() == scene)
		{
			ui_scene_browser->select(i);
		}
	}
}

