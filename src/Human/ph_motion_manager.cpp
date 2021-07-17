#include "ph_motion_manager.h"
#include "util_channel.h"

#include "ph_motion.h"
#include "util_models.h"
#include "ph_manager.h"
#include "ph_file_manager.h"
#include "ph_human.h"
#include "ph_state_manager.h"
#include "util_models.h"
#include "ph_controller.h"
#include "ph_mod_com.h"
#include "util_channel_traj.h"
#include <gsim/kn_posture.h>
#include <gsim/gs_scandir.h>
#include "ph_mod_contact.h"
#include "ph_mod_ref.h"
#include "util_manipulator.h"
void copyCmuJointAngles( KnSkeleton* _dest,KnSkeleton* _origin )
{
	copyJointAngles(_dest,_origin);
	_dest->joint("Spine")->rot()->value(_origin->joint("LowerBack")->rot()->value()*_origin->joint("Spine")->rot()->value() );
	_dest->joint("Neck")->rot()->value(_origin->joint("Neck")->rot()->value()*_origin->joint("Neck1")->rot()->value() );
	_dest->root()->pos()->value(_dest->root()->pos()->value()-GsVec(0.0f,0.12f,0.0f));
	_dest->joint("LeftUpLeg")->rot()->value(_dest->joint("LeftUpLeg")->rot()->value() * GsQuat(GsVec(0,0,1),GS_TORAD(20)));
	_dest->joint("RightUpLeg")->rot()->value(_dest->joint("RightUpLeg")->rot()->value() * GsQuat(GsVec(0,0,1),GS_TORAD(-20)));
	_dest->joint("LeftToeBase")->rot()->value(GsQuat());
	_dest->joint("RightToeBase")->rot()->value(GsQuat());
}


HumanMotionManager::HumanMotionManager(const GsString&  file):Serializable("HumanMotionManager")
{
	_phase = 0.0f;
	loadParametersFromFile(file);
	CHECK_BOOL(human_motion_manager_playing);
	CHECK_FLOAT(human_motion_manager_time);
	CHECK_BOOL(human_motion_manager_loops);
	CHECK_BOOL(human_motion_manager_resets);
	CHECK_STRING(human_motion_manager_default_control_file);
	CHECK_BOOL(human_motion_manager_interactive_mode);
	CHECK_BOOL(human_motion_manager_show_original_motion);
	CHECK_BOOL(human_motion_manager_show_replay);
	CHECK_FLOAT(human_motion_manager_step_length);
	CHECK_FLOAT(human_motion_manager_expand_dist);
	CHECK_BOOL(human_motion_manager_use_kn_motion_composite);
	CHECK_STRING(human_motion_manager_timewarp_file);
	CHECK_STRING(human_motion_manager_cmu_skel);
	CHECK_BOOL(human_motion_manager_draw_snapshots);
	CHECK_FLOAT(human_motion_manager_snapshot_increment);
	CHECK_FLOAT(human_motion_manager_start_time);
	CHECK_FLOAT(human_motion_manager_end_time);
	CHECK_BOOL(human_motion_manager_wait_for_stance);
	CHECK_BOOL(human_motion_manager_update_when_not_playing);
	CHECK_INT(human_motion_manager_rbds_max_neighbors);
	CHECK_FLOAT(human_motion_manager_rbds_support);
	CHECK_BOOL(human_motion_manager_load_env);
	CHECK_BOOL(human_motion_manager_relative_jump);
	CHECK_INT(human_motion_manager_edit_motions);
	CHECK_INT(human_motion_manager_expand_tries);
	CHECK_BOOL(human_motion_manager_edit_mode);

	_interactive_time=0;


	goal_manip = new Manipulator("TrajectoryGoalManip","TrajectoryGoalManip",file);
	goal_manip->visible(false);
	
	stanceHandCurve = 0;
	swingHandCurve = 0;
	rootCurve = 0;
	leftFootCurve = 0;
	rightFootCurve = 0;
	comCurve=0;
	_current_controller = -1;
	
	_verifyingMotions = false;
	_analyzingMotions = false;
	_expandingEnv = false;
	
	trajGrp = 0;

	_curentIsLeft = true;
	_cmu_kinematic_skeleton = 0;
	_cmu_kinematic_scene = 0;
	_dynoman_kinematic_skeleton = 0;
	_dynoman_kinematic_scene = 0;
	_cmu_scale = 1.0f;
	_current_skeleton = no_skeleton;

	_motion_lines = new SnGroup;
	
}
static void manipCallback ( SnManipulator* mnp,const GsEvent& ev, void* udata )
{
	HumanManager* mngr = ((HumanManager*)udata);
	Manipulator* manip = ((Manipulator*)mnp);
	//phout<<"manip pos  "<<manip->globalPosition()<<gsnl;
	if(mngr->getMotionManager()->setEnv(manip->globalPosition()))
		manip->setColor(GsColor::green);
	else
		manip->setColor(GsColor::red);

	//manip->evaluate();
}
void HumanMotionManager::selectMode()
{
	goal_manip->visible(true);
	setP(human_motion_manager_edit_mode,false);
}
void HumanMotionManager::init(PhysicalHuman* hum,HumanManager* mgr)
{
	human = hum;
	manager= mgr;

	manager->getRoot()->add(_motion_lines);
	goal_manip->callback(manipCallback,manager);
	manager->getRoot()->add(goal_manip);
}

void HumanMotionManager::resetAnimation()
{
	setP(human_motion_manager_time,startTime());
}

void HumanMotionManager::applyParameters()
{
	if(_current_skeleton == dynoman_skeleton)
	{
		if(_dynoman_kinematic_scene)
		{
			_dynoman_kinematic_scene->set_visibility(pBool(human_motion_manager_show_original_motion),0,0,0);
		}
	}
	else if(_current_skeleton == cmu_skeleton)
	{
		if (_cmu_kinematic_scene)
		{
			_cmu_kinematic_scene->set_visibility(pBool(human_motion_manager_show_original_motion),0,0,0);
		}
	}
}
float HumanMotionManager::update(float dt)
{
	if(!running())
		return getTime();

	float t = getTime() + dt;
	
	setTime(t);
	_phase = getPhase();
	currentController()->update();


	if(_verifyingMotions || _analyzingMotions || _expandingEnv)
	{
		
		if(currentController()->needsReset())
		{
			if(_expandingEnv)
			{
				if(!currentMotion()->tempEnv()) 
				{
					int last = pInt(human_motion_manager_edit_motions,1);
					if(last>=currentController()->numMotions())
						last = currentController()->numMotions()-1;
					
					currentController()->selectMotion(gs_random(pInt(human_motion_manager_edit_motions),last));
					addEnv();
					_expand_tries =0;
				}
				
				expandEnv();
				_expand_tries++;
			}
			setP(human_motion_manager_time,0.0f);
			resetAnimation();			
			manager->getStateManager()->selectCurrentState();
			loadMotionEnvironment();
			currentController()->initializeAnalysis();
			currentController()->startAnalysis();
			
			
		}
		else
		{
			if(!currentMotion())
			{
				
				
			}
			else if(currentMotion()->falls())
			{
				currentController()->needsReset(true);
			}
		}
	}
	else
	{
		if(currentController()->needsReset())
		{
			setP(human_motion_manager_time,0.0f);
			manager->getStateManager()->selectCurrentState();
			loadMotionEnvironment();
			//setRunning(false);
		}
		if(currentMotion())
		{
			if(!currentMotion()->active())
			{
				if(currentMotion()->_tempEnv)
				{
					goal_manip->visible(false);
				}
				if(currentController()->needsSave())
				{
					//GsString dir = currentController()->getDirectoryName();
					//dir<<currentMotion()->getMotionName()<<".motion";
					//manager->getFiles()->saveMotionTrajectories(dir,currentMotion());
					//phout<<"saved analyzed motion "<<dir<<gsnl;
				}
				//setRunning(false);
			}
		}
	}

	return getTime();
}
void HumanMotionManager::setPhase(float t)
{
	if(currentMotion() )
	{
		_phase = t;
			setP(human_motion_manager_time,t*currentMotion()->duration());
			currentMotion()->setTime(getTime());
			currentMotion()->update();
	}
	if(currentKnMotion())
	{
		currentKnMotion()->apply(currentKnMotion()->duration()*t);
		currentKnScene()->update();
	}

}
void HumanMotionManager::saveMotions(int first,int last,bool rename)
{
	int numParms = currentController()->sizeOfParameter(controller_parameters);
	if(last>currentController()->numMotions()-1)
		last = currentController()->numMotions()-1;

	for (int j=first;j<=last;j++)
	{
		HumanMotion* m = currentController()->getMotion(j);
		GsString s = currentController()->getDirectoryName();
		s<<m->getMotionName()<<".motion";

		if(rename)
		{
				
			manager->getFiles()->deleteFile(s);

			GsString newName;//=current_controller->pString(controller_name);
			int numParms = currentController()->sizeOfParameter(controller_parameters);
			for (int i=0;i<numParms;i++)
			{
				newName<<abs((int)(100*m->pFloat(motion_descriptor_avg,i)));
				if(i<numParms-1)
					newName<<"_";
			}
			s = currentController()->getDirectoryName();
			s<<newName<<".motion";
		}
		manager->getFiles()->saveMotionTrajectories(s,m);
	}
}
void HumanMotionManager::interpolatMotion(float z,float y)
{
	GsArray<float> fs;
	GsArray<float> ws;

	int distId = currentController()->getDescriptorIndex("distance");
	int heightID = currentController()->getDescriptorIndex("height");
	
	for (int i=0;i<heightID+1;i++)
	{
		fs.push(0);
		ws.push(0);
	}

	fs.set(distId,z);
	ws.set(distId,3);
	fs.set(heightID,y);
	ws.set(heightID,1);

	interpolatMotion(fs,ws,currentController()->currentMotionID());
	currentMotion()->setP(motion_descriptor_env,z,0);
	currentMotion()->setP(motion_descriptor_env,y,1);

// 	if(currentController())
// 	{
// 		if(currentController()->interpolateMotion(parm))
// 		{
// 			currentController()->currentMotion()->activate();
// 			currentController()->startAnalysis();
// 			connectMotion(currentController()->currentMotion());
// 
// 			GsString dir = currentController()->getDirectoryName();
// 			dir<<(int)parm<<".motion";
// 			manager->getFiles()->saveMotion(dir,currentController()->currentMotion());
// 		}
// 	}
}

void HumanMotionManager::interpolatMotion(const GsArray<float>& parms,const GsArray<float>& weights,int motion_id,int num_motions)
{
	if(currentController())
	{
		HumanMotion* new_motion;
		if(motion_id==-1)
		{
			new_motion = new HumanMotion(currentController()->getMotion(0));
			new_motion->setP(motion_success,0);
			GsString name = "new_motion_";
			name<<randomString();
			new_motion->setMotionName(name);
			connectMotion(new_motion);
			currentController()->pushMotion(new_motion);
			currentController()->selectMotion(name);
		}
		else
		{
			new_motion = currentController()->getMotion(motion_id);
			//phout<<"over writing this motion\n";
		}
		
		if(num_motions == -1)
			num_motions = pInt(human_motion_manager_rbds_max_neighbors);
		currentController()->interpolateMotion(new_motion,parms,weights,pFloat(human_motion_manager_rbds_support),num_motions);
	}
}

#include "ph_mod_ik.h"

void HumanMotionManager::setTime(float t)
{
	setP(human_motion_manager_time,t);

	if(currentMotion())
	{
		currentMotion()->setTime(t);
		
		if(!currentMotion()->active())
		{
			if(pBool(human_motion_manager_loops))
				setP(human_motion_manager_time,0.0f);
			else
			{
				manager->message("Done Playing.");
				return;
			}
			if(pBool(human_motion_manager_resets))
			{
				resetAnimation();
			}
			else
			{
				setRunning(false);
			}
		}
	}
}
void HumanMotionManager::makeSample(bool makeSample)
{
	bool done = false;
	for(int i=0;i<scvs.size() && !done;i++)
	{
		if(scvs.get(i)->isTrajectory())
		{
			TrajectoryChannel* ch = (TrajectoryChannel*)scvs.get(i);
			Trajectory* cv = ch->getCurve();
			if(cv->selectionState == CURVE_POINT_SELECTED)
			{
				done = true;
				int sel = cv->selection;
				int sampleIdx = -1;
				for(int j=0;j<ch->getCurve()->numSamplePoints()&& sampleIdx ==-1;j++)
				{
					if(ch->getCurve()->sample(j)->index == sel)
					{
						sampleIdx = j;
						if(!makeSample)
						{
							ch->removeSample(sampleIdx);
							if(ch->getCurve()->numSamplePoints()==0)
								ch->setP(trajectory_channel_sample,false);
							return;
						}
					}
				}
				//no sample limit found so make one if 
				if(makeSample)
				{
					sample_data p;
					p.index = sel;
					p.rest = cv->getPoint(cv->selection);
					p.max=GsVec2(0.15f,0.15f);
					p.min = GsVec2(-0.15f,-0.15f);
					ch->addSample(p);
					ch->setP(trajectory_channel_sample,true);
					ch->copyFromTrajectory();
				}
			}
		}
	}
	
}
void HumanMotionManager::initCurves()
{
	for(int i=0;i<scvs.size();i++)
	{
		if(scvs.get(i)->isTrajectory())
		{
			TrajectoryChannel* ch = (TrajectoryChannel*)scvs.get(i);
			ch->initCurve();
		}
	}
}

void HumanMotionManager::pointEdit(double px, double py,double rx,double ry,double xMin,double xMax,double yMin,double yMax)
{
	bool done = false;
	for(int i=0;i<scvs.size() && !done;i++)
	{
		if(scvs.get(i)->isTrajectory())
		{
			TrajectoryChannel* ch = (TrajectoryChannel*)scvs.get(i);

			if(ch->getCurve()->selectionState == CURVE_POINT_SELECTED)
			{
				done = true;
				int sel = ch->getCurve()->selection;
		
				if(ch->samples())
				{
					int sampleIdx = -1;

					for(int j=0;j<ch->getCurve()->numSamplePoints()&& sampleIdx ==-1;j++)
					{
						if(ch->getCurve()->sample(j)->index == sel)
						{
							sampleIdx = j;
						}
					}
					if(sampleIdx==-1)
					{
						//no sample limit found so make one if 

					}
					else
					{
						//phout<<"move point "<<px<<" "<<py<<gsnl;
						ch->getCurve()->sample(sampleIdx)->min = GsVec2(xMin,yMin);
						ch->getCurve()->sample(sampleIdx)->max = GsVec2(xMax,yMax);
						ch->getCurve()->sample(sampleIdx)->rest = GsVec2(rx,ry);
						ch->getCurve()->setPoint(sel,GsVec2(px,py));
						ch->getCurve()->update();
						ch->copyFromTrajectory();
						return;
					}
				}
			
				
				ch->getCurve()->setPoint(sel,GsVec2(px,py));
				ch->getCurve()->update();
				ch->copyFromTrajectory();
				
			}
		
		}
	}
}
void HumanMotionManager::randomize()
{
	if(currentMotion())
	{
		for(int j=0;j<currentMotion()->numChannels();j++)
		{
			currentMotion()->getChannel(j)->randomize();
		}
		updateCurves();
	}
	
}
int HumanMotionManager::selectKnMotion( const GsString&  name )
{
	
	GsString filename = manager->getFiles()->getKinematicsFile(name);
	phout<<"filename = "<<filename<<gsnl;

	GsString shortName = name;
	remove_path(shortName);
	remove_extension(shortName);
	for (int i=0;i<_kn_motions.size();i++)
	{
		if (_kn_motions[i]->name() == shortName)
		{
			chooseKnMotion(_kn_motions[i]);

			if(trajGrp)
				trajGrp->visible(false);
			manager->message("motion already loaded:",shortName);
			return i;
		}
	}

	_kn_motions.push() = openKnMotion(filename);
	
	if(!_kn_motions.top())
	{
		_kn_motions.pop();
	}
	else
	{
		
		KnMotion* knm = _kn_motions.top();
		if(filename.search("cmu")>=0)
		{
			GsString nameT = "cmu_";
			nameT<<shortName;
			shortName = nameT;
			phout<<"shortName = "<<shortName<<gsnl;
			knm->name(shortName);
		}
		chooseKnMotion(knm);
		if(_current_skeleton == cmu_skeleton)
		{
			scale_motion(knm,_cmu_scale);
			
			knm->apply_frame(1);
			_cmu_kinematic_skeleton->update_global_matrices();
			GsVec cmuPos = _cmu_kinematic_skeleton->joint("LeftFoot")->gcenter() + _cmu_kinematic_skeleton->joint("RightFoot")->gcenter();
			cmuPos = cmuPos/2.0f;
			translate_motion(knm,-cmuPos);
			orientMotionToOrigin(knm);
			knm->remove_frame(0);
			knm->apply_frame(0);
			_cmu_kinematic_scene->update();
		}
		else
		{
			
			knm->apply_frame(0);
			scale_motion(knm,0.01f);
			_dynoman_kinematic_scene->update();
		}

		_kn_motions.top()->name(shortName);
		_current_kn_motion = _kn_motions.size()-1;
		
		manager->message("motion loaded:",shortName);
		if(trajGrp)
			trajGrp->visible(false);
	}

	return _kn_motions.size()-1;
	
	return 0;
}
HumanMotion* HumanMotionManager::openMotion(const GsString&  dirname)
{
	HumanMotion* motion = new HumanMotion();

	if(motion->load(dirname))
	{
		GsString motionName = dirname;
		motion->applyParameters();
		remove_extension(motionName);
		remove_path(motionName);
		motion->setMotionName(motionName);
		_motion_lines->add(motion->env_lines);
		
	}
	else
	{
		phout<<"could not open motion "<<dirname<<gsnl;
		delete motion;
		motion = 0;
	}
	return motion;
}

void HumanMotionManager::removeDuplicates()
{
	GsStrings motionsToDelete;

	for (int i=0;i<currentController()->numMotions()-1;i++)
	{
		HumanMotion* m1 = currentController()->getMotion(i);
		for (int j=i+1;j<currentController()->numMotions();j++)
		{
			if(i!=j)
			{
				HumanMotion* m2 = currentController()->getMotion(j);
				if(m1->numChannels()!=m2->numChannels())
					continue;
				bool looking = true;
				for(int k=0;k< m1->numChannels() && looking;k++)
				{
					if(m1->getChannel(k)->isTrajectory())
					{
						Trajectory*  c1 = ((TrajectoryChannel*)m1->getChannel(k))->getCurve();
						Trajectory*  c2 = ((TrajectoryChannel*)m2->getChannel(k))->getCurve();
						if(c1->numPoints()!=c2->numPoints())
						{
							looking = false;
						}
						else
						{
							for(int l=0;l<c1->numPoints() && looking;l++)
							{
								if(c1->getPoint(l)!=c2->getPoint(l))
								{
									looking = false;
								}
							}
						}
					}
				}
				if(looking)
				{
					phout<<m1->getMotionName()<<" is the same as "<<m2->getMotionName()<<gsnl;
					if(m1->numEnv()>m2->numEnv())
					{
						if(!arrayContains(&motionsToDelete,m2->getMotionName()))
							motionsToDelete.push(m2->getMotionName());
					}
					else
					{
						if(!arrayContains(&motionsToDelete,m1->getMotionName()))
							motionsToDelete.push(m1->getMotionName());
					}
				}
			}
		}
	}
	for (int i=0;i<motionsToDelete.size();i++)
	{
		GsString s = motionsToDelete[i];
		GsString file = currentController()->getDirectoryName();
		file<<s<<".motion";
		manager->getFiles()->deleteFile(file);
	}
}
int HumanMotionManager::loadController(const GsString&  dirname,bool forceReload)
{
	for (int i=0;i<_controllers.size();i++)
	{
		//phout<<"saved name "<<_controllers[i]->getDirectoryName()<<" this name "<<dirname<<gsnl;
		if (_controllers[i]->getDirectoryName()==dirname)
		{
			if(forceReload)
			{
				delete _controllers[i];
				_controllers.remove(i);
			}
			else
			{
				return i;
			}
		}
	}
	GsStrings control_files;
	gs_scandir(dirname,control_files,"control");
	Controller* _controller = 0;
	if(control_files.size()>0)
	{
		_controller = new Controller(manager->selectedCharacter(), control_files[0]);
	}
	else
	{
		_controller = new Controller(manager->selectedCharacter());
	}
	_controller->setDirectoryName(dirname);
	_controllers.push(_controller);

	GsStrings motion_files;
	gs_scandir(dirname,motion_files,"motion");
	if (motion_files.size()==0)
	{
		phout<<"error did not find any motion files to load in directory "<<dirname<<gsnl;
	}
	
	GsStrings base_file;
	HumanMotion* base_motion =0;
	gs_scandir(dirname,base_file,"base");
	GsStrings sample_file;

	gs_scandir(dirname,sample_file,"sample");

	if (base_file.size()!=0)
	{
		base_motion = openMotion(base_file[0]);
		connectMotion(base_motion);
	}
	
	for (int i=0;i<motion_files.size();i++)
	{
		
		HumanMotion* motion = 0;
		if(base_motion ==0)
			motion = openMotion(motion_files[i]);
		else
		{

			/*first just load the base parameters such as channel names and any repeted variable*/
			motion = new HumanMotion(base_motion);
			
		//	connectMotion(motion);

			/*then load the trajectories that are unique to this motion*/
			motion->loadDataFromFile(motion_files[i]);
			
			/*then load the sample data which should be consistent throughout the motions*/
			if(sample_file.size()!=0)
				motion->loadDataFromFile(sample_file[0]);

			GsString motionName = motion_files[i];
			remove_extension(motionName);
			remove_path(motionName);
			motion->setMotionName(motionName);
			
			
		}
		if(motion==0)
		{
			phout<<"could not loadMotion "<<dirname<<gsnl;
			return -1;
		}
		else
		{
			motion->init();
			connectMotion(motion);
			
			_controller->pushMotion(motion);
		}
	}

	//_controller->selectMotion(0);
	
	return _controllers.size()-1;
}
/*at different points in the phase the stance may be left or right so it should be set here*/
void HumanMotionManager::connectMotion(Motion* m)
{
	for(int j=0;j<m->numChannels();j++)
	{
		Channel* ch = m->getChannel(j);
		if(ch->controlsObject()) //if it doesn't control object it is probably a control node
		{
			ch->setObject(manager->findSerializable(ch->pString(channel_object_name),ch->pString(channel_object_type)));
			if(m->isHumanMotion())
			{
				if(((HumanMotion*)m)->isMirrored())
				{
					if(ch->isTrajectory())
					{
						TrajectoryChannel* tch = (TrajectoryChannel*)ch;

						TrajectoryChannel* comp = ((HumanMotion*)m)->findComplement(tch);
						if(comp)
							tch->getCurve()->complement = comp->getCurve();
					}
				}
			}
		}
		ch->clearInputs();

		for(int i=0;i<ch->pStringArray(channel_control_list)->size();i++)
		{
			GsString inputName = ch->pString(channel_control_list,i);
			if(inputName != "NULL")
			{
				Channel* inp = m->getChannel(inputName);
				if(inp)
				{
					ch->pushInput(inp);
				}
				else
				{
					phout<<"could not find input "<<inputName<<gsnl;
				}
			}
			else
			{
					ch->pushInput(0);
			}

		}
	}
}

void HumanMotionManager::togglePlaying()
{
	setRunning(!running());	
}
void HumanMotionManager::setPhase()
{
	setPhase(_phase);
}
void HumanMotionManager::updateCurves()
{
	//phout<<"HumanMotionManager::updateCurves()\n";
	if(!currentMotion())
		return;

	for(int j=0;j<scvs.size();j++)
	{ 
		Channel* ch = scvs.get(j);
		ch->duration(currentMotion()->duration());
		if(scvs.get(j)->isTrajectory())
		{
			TrajectoryChannel* tch = (TrajectoryChannel*)scvs.get(j);
			tch->copyFromTrajectory();
			tch->getCurve()->update();  
		}
	}
	updateMotionTrajectories();
	if(!running())
	{
		setPhase();
	}
}
bool HumanMotionManager::running()
{
	return pBool(human_motion_manager_playing);
}
HumanMotionManager::~HumanMotionManager()
{
	for(int i=0;i<_controllers.size();i++)
	{
		delete _controllers[i];
	}

	if(stanceHandCurve){delete stanceHandCurve; stanceHandCurve = 0;}
	if(swingHandCurve){delete swingHandCurve;swingHandCurve=0;}
	if(rootCurve){delete rootCurve; rootCurve=0;}
	if(leftFootCurve){delete leftFootCurve; leftFootCurve=0;}
	if(rightFootCurve){delete rightFootCurve; rightFootCurve=0;}
	if(comCurve){delete comCurve; comCurve=0;}
}

void HumanMotionManager::unselectCurves()
{
	scvs.size(0);
}

void HumanMotionManager::selectCurve(Channel* ch)
{
	scvs.push(ch);
}

GsArray<Channel*>* HumanMotionManager::getSelectedChannels()
{
	return &scvs;
}

float HumanMotionManager::getAnimationTimeStep()
{
	return manager->getAnimationTimeStep();
}

void HumanMotionManager::selectController( const GsString&  motionDirectory,bool forceReload)
{
	setRunning(false);
	setP(human_motion_manager_time,0.0f);
	_current_controller = loadController(motionDirectory,forceReload);
	setP(human_motion_manager_start_time,0.0f);
	setP(human_motion_manager_end_time,duration());
}

void HumanMotionManager::mirrorKnMotion()
{
	if(getSelectedKnMotion())
			getSelectedKnMotion()->mirror("Left","Right");
}


KnMotion* HumanMotionManager::getSelectedKnMotion()
{
	if(currentMotion())
	{
		return currentMotion()->getKnMotion();
	}
	return 0;
}

KnMotion* HumanMotionManager::openKnMotion( const GsString&  fileName )
{
	KnMotion* km = new KnMotion;
	km->ref();
	if(!km->load(fileName))
	{
		km->unref();
		return 0;
	}
	else
	{
		return km;
	}

}

GsArray<int> HumanMotionManager::currentMotionChannelIndexes()
{
	GsArray<int> idxs;
	if(!currentMotion())
		return idxs;

	for (int i=0;i<scvs.size();i++)
	{
		for(int j=0;j<currentMotion()->numChannels();j++)
		{
			if(scvs[i]->name() == currentMotion()->getChannel(j)->name())
			{
				idxs.push(j);
			}
		}
	}
	return idxs;
}
void HumanMotionManager::setSelectedMotionChannelIndexes(const GsArray<int>& idxs)
{
	if(!currentMotion())
		return ;

		scvs.size(0);
		for(int j=0;j<idxs.size();j++)
		{
			Channel* ch = currentMotion()->getChannel(idxs[j]);
			if(ch)
			{
				ch->showCurve();
				ch->showNode();
				scvs.push(ch);
			}
		}


}

void HumanMotionManager::setRunning( bool rng )
{
	
	setP(human_motion_manager_playing,rng);

	if(rng)
	{
		if(!currentMotion())
		{
			manager->message("There is no motion selected to play.");
			setP(human_motion_manager_playing,false);
		}
		else
		{
			
			GsString mes = "Start playing selected motion:";
			mes<<currentMotionName();
			manager->message(mes);
			
		}
	}
	else
	{
		if(_expandingEnv)
		{
			if(!currentMotion()->tempEnv())
				currentMotion()->removeEnv();
			currentMotion()->updateEnvHull();
			currentMotion()->updateEnvLines();
		}
		_verifyingMotions = false;
		_analyzingMotions = false;
		_expandingEnv = false;
	}

}
float HumanMotionManager::duration()
{
	if(currentMotion())
	{
		return currentMotion()->duration();
	}
	
	return 1.0f;
}
void HumanMotionManager::duration(float d)
{
	if(currentMotion())
	{
		currentMotion()->duration(d);
	}
}
float HumanMotionManager::getPhase()
{
	return getTime()/duration();
}

float HumanMotionManager::getTime()
{
	return pFloat(human_motion_manager_time);
}

bool HumanMotionManager::loops()
{
	return pBool(human_motion_manager_loops);
}
void HumanMotionManager::loops(bool p)
{
	setP(human_motion_manager_loops,p);
}
void HumanMotionManager::resets(bool p)
{
	setP(human_motion_manager_resets,p);
}


bool HumanMotionManager::resets()
{
	return pBool(human_motion_manager_resets);
}

void HumanMotionManager::addChannel(const GsString&  n, Serializable* sav, int param, channel_dof_types dof,int arrayIdx,chanel_modes mode,bool constant)
{
	if(sav)
	{
		if(currentMotion())
		{
			manager->message("Added new channel");
			Channel* ch = 0;
			if(mode == channel_trajectory)
			{
					trajectory_type t = TRAJ_BEZIER;
					if(dof == CH_BOOL)
					{
						t = TRAJ_STEP;
					}

					ch = new TrajectoryChannel(sav,param,dof,t,arrayIdx);
					
					
			}
			else
			{
				 ch = new Channel(sav,param,dof,mode,arrayIdx);
				ch->setControlVal(ch->currentChannelVal());
				
				
			}

			selectCurve(ch);
			ch->setP(channel_val_constant,constant);
			ch->setName(n);
			currentMotion()->pushChannel(ch);

			return;
		}
	}
	manager->message("Error adding channel");
}

void HumanMotionManager::deselectMotion()
{
	if(_dynoman_kinematic_scene)_dynoman_kinematic_scene->set_visibility(0,0,0,0);
	if(_cmu_kinematic_scene)_cmu_kinematic_scene->set_visibility(0,0,0,0);

	trajGrp->visible(false);
	_current_controller = -1;
	scvs.size(0);
	updateMotionTrajectories();
}

void HumanMotionManager::fixShoulders(KnSkeleton* sk)
{
	sk->joint("LeftArm")->rot()->value(sk->joint("LeftShoulder")->rot()->value()*sk->joint("LeftArm")->rot()->value());
	sk->joint("RightArm")->rot()->value(sk->joint("RightShoulder")->rot()->value()*sk->joint("RightArm")->rot()->value());
	sk->joint("RightShoulder")->rot()->value(GsQuat());
	sk->joint("LeftShoulder")->rot()->value(GsQuat());
}


GsString HumanMotionManager::currentMotionName()
{
	if(currentMotion())
		return currentMotion()->getMotionName();
	return "no motion selected";
}


void HumanMotionManager::makeTimeWarp()
{
	if(currentMotion())
	{
		TrajectoryChannel* ch = new TrajectoryChannel( manager->getFiles()->getMotionFile(manager->selectedCharacter()->characterName(),pString(human_motion_manager_timewarp_file)),"TimeWarp");
	
		ch->getCurve()->scale(currentMotion()->duration(),true);
		ch->copyFromTrajectory();
		currentMotion()->setTimeWarp(ch);

	}
	else
	{
		manager->message("must be editing motion");
	}
}
void HumanMotionManager::chooseKnMotion(KnMotion* _kn_motion )
{
	
	if(_cmu_kinematic_scene)
		_cmu_kinematic_scene->set_visibility(0,0,0,0);
	if(_dynoman_kinematic_scene)
		_dynoman_kinematic_scene->set_visibility(0,0,0,0);
	//_current_kn_motion = _kn_motion;
	if(_kn_motion == 0)
	{
		return;
	}
	GsString n = _kn_motion->name();
	phout<<"name = "<<n<<gsnl;

	for (int i=0;i<_kn_motions.size();i++)
	{
		if(_kn_motions[i]->name() == n)
		{
			_current_kn_motion = i;
		}
	}

	if(n.search("cmu")!=-1)
	{
		manager->message("cmu motion ",n);
		_current_skeleton = cmu_skeleton;
		if(_cmu_kinematic_skeleton==0)
		{
			_cmu_kinematic_scene  = new KnScene; _cmu_kinematic_scene->ref();
			_cmu_kinematic_skeleton = new KnSkeleton; _cmu_kinematic_skeleton->ref();
			if(_cmu_kinematic_skeleton->load(manager->getFiles()->getKinematicsFile(pString(human_motion_manager_cmu_skel))))
			{
				_cmu_kinematic_scene->connect(_cmu_kinematic_skeleton);

				_cmu_kinematic_skeleton->update_global_matrices();

				float ch_dis = dist(
					manager->selectedCharacter()->skref()->joint("LeftShoulder")->gcenter(),
					manager->selectedCharacter()->skref()->joint("LeftFoot")->gcenter());
				float cmu_dist = dist(
					_cmu_kinematic_skeleton->joint("LeftShoulder")->gcenter(),
					_cmu_kinematic_skeleton->joint("LeftFoot")->gcenter());
				_cmu_scale = ch_dis /cmu_dist;


				scale_skeleton(_cmu_kinematic_skeleton,_cmu_scale);
				_cmu_kinematic_scene->rebuild();
				
				_cmu_kinematic_scene->set_skeleton_joint_color(GsColor::yellow);
				manager->getRoot()->add(_cmu_kinematic_scene);
				
				_cmu_kinematic_skeleton->update_global_matrices();
				_cmu_kinematic_scene->set_skeleton_radius(0.1f);
			}
			else
			{
				manager->message("Could not load skeleton from cmu motion");
				_cmu_kinematic_scene->unref();
				_cmu_kinematic_skeleton->unref();
				_cmu_kinematic_scene = 0;
				_cmu_kinematic_skeleton = 0;
				return ;
			}
		}
		
	

		currentKnMotion()->connect(_cmu_kinematic_skeleton);
		_cmu_kinematic_scene->set_visibility(pBool(human_motion_manager_show_original_motion),0,0,0);
		currentKnMotion()->apply(0);
		copyCmuJointAngles(manager->selectedCharacter()->skref(),_cmu_kinematic_skeleton);
		_cmu_kinematic_scene->update();
	}
	else
	{
		//manager->message("dyno motion ",n);
		_current_skeleton = dynoman_skeleton;
		if(_dynoman_kinematic_skeleton==0)
		{
			_dynoman_kinematic_skeleton = new KnSkeleton(*manager->selectedCharacter()->skref());
			_dynoman_kinematic_scene = new KnScene;
			_dynoman_kinematic_scene->connect(_dynoman_kinematic_skeleton);
			_dynoman_kinematic_scene->set_skeleton_radius(0.5f);
			_dynoman_kinematic_scene->set_skeleton_joint_color(GsColor::green);

			manager->getRoot()->add(_dynoman_kinematic_scene);

		}
		_dynoman_kinematic_scene->set_visibility(pBool(human_motion_manager_show_original_motion),0,0,0);
		
		currentKnMotion()->connect(_dynoman_kinematic_skeleton);
		currentKnMotion()->apply(0);
		fixShoulders(_dynoman_kinematic_skeleton);
		copyJointAngles(manager->selectedCharacter()->skref(),_dynoman_kinematic_skeleton);
		_dynoman_kinematic_scene->update();
	}
	setP(human_motion_manager_time,0.0f);
	
}

int HumanMotionManager::getFrame()
{	
	return (int)(getTime()/manager->getAnimationTimeStep());
}

void HumanMotionManager::setFrame( int f )
{
	setPhase((f*manager->getAnimationTimeStep())/duration());
}

float HumanMotionManager::startFrame()
{
	return startTime()/manager->getAnimationTimeStep();
}

float HumanMotionManager::endFrame()
{
	return endTime()/manager->getAnimationTimeStep();
}

#include "ph_mod_contact.h"

void HumanMotionManager::setStartState( const GsString&  stateName )
{
	if(currentMotion() && currentMotion())
	{
		currentMotion()->setStartState(stateName);
	}
}

void HumanMotionManager::makeControlChannel()
{
	if(!currentMotion())
	{
		manager->message("must have motion selected");
		return;
	}
	if(scvs.size()==0)
	{
		manager->message("must have some channels selected");
		return;
	}

	for(int i=0;i<scvs.size();i++)
	{
			Channel* ch = new Channel((Serializable*)scvs[i]);
			GsString n = ch->name();
			n<<"_Control";
			ch->setName(n);
			
			
			scvs[i]->setControlCurve(ch);
			currentMotion()->pushChannel(ch);
	}
}

void HumanMotionManager::makeChannelNode( chanel_modes mode )
{
	if(currentMotion())
	{
		Channel* ch;
		if(mode == channel_trajectory)
			ch = new TrajectoryChannel(manager->getFiles()->getControlFile("default_node.motion"),"Trajectory");
		else 
			ch = new Channel(manager->getFiles()->getControlFile("default_node.motion"),"Node");

		ch->setChannelMode(mode);
		
		GsString s = "NewChannel_";
		s<<randomString();
		ch->setName(s);

		currentMotion()->pushChannel(ch);
		currentMotion()->updateChannelList();
	}

}

void HumanMotionManager::updateMotionData()
{
	
	currentMotion()->updateChannelList();
	
}

void HumanMotionManager::reloadController()
{
	if(currentMotion())
	{
		selectController(currentController()->getDirectoryName(),true);
	}
}

HumanMotion* HumanMotionManager::currentMotion()
{
	if(currentController())
		return currentController()->currentMotion();
	return 0;
}

void HumanMotionManager::makeLocalSample(float tol)
{
	if(currentMotion())
	{
		for (int i=0;i<currentMotion()->numChannels();i++)
		{
			if (currentMotion()->getChannel(i)->isTrajectory())
			{
				TrajectoryChannel* tch = (TrajectoryChannel*)currentMotion()->getChannel(i);
				Trajectory* t = tch->getCurve();
				if (tch->samples())
				{
					if(tch->pBool(trajectory_channel_lock_reconfigure))
					{

					}
					else
					{
						for(int s=0;s<t->numSamplePoints();s++)
						{
							sample_data* sd = t->sample(s);
							sd->rest = tch->getCurve()->getPoint(sd->index);
							sd->min = GsVec2(0.0f,-tol);
							sd->max = GsVec2(0.0f,tol);
						}
						tch->copyFromTrajectory();
					}
				}
			}
		}
	}
}

void HumanMotionManager::loadMotionEnvironment()
{
	if(currentMotion())
	{
		manager->loadScene(currentController()->getMotionScene()); 
	}
}
void HumanMotionManager::analyzeMotions()
{
	if(currentController())
	{
		if(_analyzingMotions)
		{
			_analyzingMotions = false;
			setRunning(false);
			currentController()->endAnalysis();
		}
		else
		{
			_analyzingMotions = true;
			resetAnimation();
			manager->resetState();
			currentController()->initializeAnalysis();
			currentController()->analyzeMotions();
			setRunning(true);
		}

	}
}

void HumanMotionManager::verifyMotions()
{
	if(currentController())
	{
		if(_verifyingMotions)
		{
			_verifyingMotions = false;
			setRunning(false);
			currentController()->verifyMotions();
		}
		else
		{
			_verifyingMotions = true;
			currentController()->verifyMotions(pInt(human_motion_manager_edit_motions),pInt(human_motion_manager_edit_motions,1));
			setRunning(true);
		}
		
	}
}



void HumanMotionManager::forceKeysFromState(int m)
{
	if(getMotion(m))
	{
		for (int i=0;i<getMotion(m)->numChannels();i++)
		{
			Channel* ch = getMotion(m)->getChannel(i);
			if(ch->isTrajectory() && ch->getObject())
			{

				TrajectoryChannel* tch = (TrajectoryChannel*)ch;
				phout<<"value[0] before for channel "<<tch->name()<<" was "<<tch->pFloat(trajectory_channel_p_y);
				tch->setP(trajectory_channel_p_y,tch->currentChannelVal(),0);
				tch->setP(trajectory_channel_p_t,0.0f,0);
				phout<<"  --> "<<tch->pFloat(trajectory_channel_p_y)<<gsnl;
				tch->applyParameters();
			}
		}
	}
}

void HumanMotionManager::setGoalPosition( GsVec pos )
{
	if(currentMotion())
	{
		
		currentMotion()->_tempEnv = true;
		currentMotion()->_tempEnvVec = pos;
		
			currentMotion()->selectEnv(-2);

		currentController()->startAnalysis();
		if(pBool(human_motion_manager_load_env))
			loadMotionEnvironment();
	}
}

void HumanMotionManager::motionLinesVisible(bool vis)
{
	_motion_lines->visible(vis);
	if(vis)
	{
		updateMotionLines();
	}
}
void HumanMotionManager::showAllMotionLines()
{
	_motion_lines->visible(true);
	_motion_lines->remove_all();
	for (int i=0;i<currentController()->numMotions();i++)
	{
		currentController()->getMotion(i)->env_lines->ref();
		_motion_lines->add(currentController()->getMotion(i)->env_lines);
	}
}

void HumanMotionManager::updateMotionLines(int first,int last)
{
	if(first==-1)
	{
		first = currentController()->currentMotionID();
		last = first;
	}
	_motion_lines->remove_all();
	for (int i=first;i<=last;i++)
	{
		HumanMotion* m = currentController()->getMotion(i);
		m->updateEnvLines();
		m->env_lines->ref();
		_motion_lines->add(m->env_lines);
		_motion_lines->visible(true);
	}
	


}
#include "ph_trajectory_planner.h"

void HumanMotionManager::expandEnv()
{
	if(currentMotion())
	{	
		goal_manip->visible(true);
		currentMotion()->expandEnv();
		goal_manip->translation(currentMotion()->getEnv());
		goal_manip->update();
		currentController()->startAnalysis();
		motionLinesVisible(true);
		manager->loadScene(currentController()->getMotionScene());
	}
}

void HumanMotionManager::addEnv()
{
	if(currentMotion())
	{	
		goal_manip->visible(true);
		currentMotion()->newEnv();
		goal_manip->translation(currentMotion()->getEnv());
		currentController()->startAnalysis();
		motionLinesVisible(true);
	}
}

void HumanMotionManager::startExpandingEnv()
{
	int first = pInt(human_motion_manager_edit_motions);
	int last = pInt(human_motion_manager_edit_motions,1);
	if(first<0)first=0;
	if(last>currentController()->numMotions())
		last = currentController()->numMotions()-1;

	currentController()->selectMotion(gs_random(first,last));
	_expandingEnv = true;
	addEnv();
	expandEnv();
	setRunning(true);
	_expand_tries = 0;
}

bool HumanMotionManager::selectClosestFromHull( GsVec _jumpP )
{
	GsArray<int> validMotions;
	if(currentController())
	{
		for (int i=0;i<currentController()->numMotions();i++)
		{
			HumanMotion* m = currentController()->getMotion(i);
			if(m->env_poly_hull.contains(GsPnt2(_jumpP.z,_jumpP.y)))
			{
				validMotions.push(i);
			}
		}
	}
	phout<<"found "<<validMotions.size()<<" motions\n";
	float dist = 1000;
	int idx = -1;
	for (int i=0;i<validMotions.size();i++)
	{
		int mid = validMotions[i];
		HumanMotion* m = currentController()->getMotion(mid);
		GsVec dis = m->envCentroid() - _jumpP;
		if(dis.len()<dist)
		{
			dist = dis.len();
			idx = mid;
		}
	}
	if(idx!=-1)
	{
		currentController()->selectMotion(idx);
		currentMotion()->setEnv(_jumpP,-2);
		currentMotion()->selectEnv(-2);
		if(pBool(human_motion_manager_load_env))
			loadMotionEnvironment();
		return true;
	}
	else
	{
		phout<<"could not find motion\n";
		return false;
	}
}

bool HumanMotionManager::setEnv( GsVec pos )
{
	if(pBool(human_motion_manager_edit_mode))
	{
		setGoalPosition(pos);
		currentMotion()->updateEnvLines();
		return true;
	}
	else
	{
		return selectClosestFromHull(pos);
	}
}

KnMotion* HumanMotionManager::getKnMotion( int i )
{
	if(_current_kn_motion>=0 && _current_kn_motion < _kn_motions.size())
		return _kn_motions[i];
	else return 0;
}

Controller* HumanMotionManager::newController( const GsString&  contName )
{
	_controllers.push() = new Controller(manager->selectedCharacter());
	_current_controller = _controllers.size()-1;
	currentController()->setName(contName);
	return currentController();
}

void HumanMotionManager::saveBaseMotion()
{
	if(currentController() && currentMotion())
	{
		GsString dir = currentController()->getDirectoryName();
		GsString shortName = dir;
		directory_short_name(shortName);
		dir<<shortName<<".base";
		manager->getFiles()->saveBaseMotion(dir,currentController()->currentMotion());
	}
}

void HumanMotionManager::hideSelectedNodes()
{
	for(int i=0;i<scvs.size();i++)
	{
		scvs[i]->hideNode();
	}
}

void HumanMotionManager::hideAllNodes()
{
	if(!currentMotion())
		return;

	for (int i=0;i<currentMotion()->numChannels();i++)
	{
		currentMotion()->getChannel(i)->hideNode();
	}
}

void HumanMotionManager::showAllNodes()
{
	for (int i=0;i<currentMotion()->numChannels();i++)
	{
		currentMotion()->getChannel(i)->showNode();
	}
}

void HumanMotionManager::setInputsVisible(Channel* ch)
{
	ch->showNode();
	for(int j=0;j<ch->numInputs();j++)
	{
		if(ch->getInput(j))
			setInputsVisible(ch->getInput(j));
	}
}
void HumanMotionManager::setOutputsVisible(Channel* ch)
{
	ch->showNode();
	for(int j=0;j<currentMotion()->numChannels();j++)
	{
		Channel* c2 = currentMotion()->getChannel(j);
		
		for(int i=0;i<c2->numInputs();i++)
		{
			if(c2 != ch)
			{
				if(c2->getInput(i) == ch)
				{
					setOutputsVisible(c2);
				}
			}
		}
	}
}
void HumanMotionManager::viewInputNodes()
{
	for(int i=0;i<scvs.size();i++)
	{
		setInputsVisible(scvs[i]);
	}
}
void HumanMotionManager::viewOutputNodes()
{
	for(int i=0;i<scvs.size();i++)
	{
		setOutputsVisible(scvs[i]);
	}
}


void HumanMotionManager::selectChannel( Channel* ch )
{
	scvs.size(0);
	scvs.push(ch);

}


SnapShotVis::SnapShotVis( PhysicalHuman* h )
{
	g = new SnGroup;
	for(int i=0;i<h->numJoints();i++)
	{
		Box* b = new Box(h->joint(i)->getCOMPosition(),h->joint(i)->getDimension()*0.95f);
		b->setRotation(h->joint(i)->getGlobalOrientation());
		GsColor c = h->joint(i)->getColor();
		c.r= (gsbyte) (c.r*0.5);
		c.g = (gsbyte)(c.g*0.5);
		c.b = (gsbyte)(c.b*0.5);
		b->setColor(c);
		g->add(b->getGrp());
		b->name = h->joint(i)->name();
		boxes.push(b);
	}
}

void SnapShotVis::set( HumanState* s )
{
	for(int i=0;i<boxes.size();i++)
	{
		if(s->jointDefs)
		{
			Serializable* bdef = s->jointDefs->getSerializable(boxes[i]->name);
			if(bdef)
			{
				GsQuat q = ((QuatParameter*)bdef->getParameter("ode_orientation"))->val;
				GsVec v = ((VecParameter*)bdef->getParameter("ode_position"))->val;
				boxes[i]->setRotation(q);
				boxes[i]->setPosition(v);
			}
		}
		else
		{
			phout<<"no joint defs\n";
		}
	}
}