
#include "ph_trajectory_planner.h"
#include "ph_human.h"
#include "util_channel.h"
#include "util_motion.h"
#include "ph_motion.h"
#include "ph_motion_manager.h"
#include "ph_manager.h"
#include "ph_file_manager.h"
#include "ph_mod_contact.h"
#include "ph_controller.h"
#include "util_manipulator.h"


static void manipC ( SnManipulator* mnp,const GsEvent& ev, void* udata )
{
	TrajectoryPlanner* planner = ((TrajectoryPlanner*)udata);
	planner->updateSampleLine();
}


TrajectoryPlanner::TrajectoryPlanner(GsString file):Serializable("TrajectoryPlanner")
{
	loadParametersFromFile(file);

	CHECK_FLOAT(trajectory_planner_plan_time);
	CHECK_FLOAT(trajectory_planner_min_distance);
	CHECK_FLOAT(trajectory_planner_min_com_height);
	CHECK_INT(trajectory_planner_tries_per_jump);
	CHECK_FLOAT(trajectory_planner_start_sample_tolerance);
	CHECK_FLOAT(trajectory_planner_max_sample_tolerance);
	CHECK_FLOAT(trajectory_planner_start_sample_increment);
	CHECK_INT(trajectory_planner_max_interpolation_motions);
	CHECK_FLOAT(trajectory_planner_energy_weight);
	CHECK_FLOAT(trajectory_planner_sample_bounds);
	CHECK_INT(trajectory_planner_seed);
	_running=false;
	manager = 0;
	_human = 0;
	_tries = 0;
	_plan_time = 0;
	
	_explore_mode = _explore_jumps;
	goal_bound_max = new Manipulator("TrajectoryGoalManip","Manip",file);
	goal_bound_min = new Manipulator("TrajectoryGoalManip","Manip",file);
	goal_bound_min->callback(manipC,this);
	goal_bound_max->callback(manipC,this);
	goal_bound_min->translation(GsVec(0.0f,pFloat(trajectory_planner_sample_bounds,2),pFloat(trajectory_planner_sample_bounds,0)));
	goal_bound_max->translation(GsVec(0.0f,pFloat(trajectory_planner_sample_bounds,3),pFloat(trajectory_planner_sample_bounds,1)));
	goal_bound_max->visible(false);
	goal_bound_min->visible(false);

}


void TrajectoryPlanner::makeNewTry()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

	motion_manager->randomize();
	_plan_time = 0.0f;
	motion_manager->currentController()->needsReset(true);
	motion_manager->resetAnimation();
	motion_manager->updateMotionTrajectories();
	motion_manager->setRunning(true);
	motion_manager->currentController()->initializeAnalysis();
	motion_manager->currentController()->startAnalysis();
	motion_manager->currentMotion()->falls(false);
	_tries++;
	 if(_explore_mode == _randomize_motion)
	 {
		 return;
	 }

	if(_tries>pInt(trajectory_planner_tries_per_jump))
	{
		_tries = 0;
		_sample_tolerance += pFloat(trajectory_planner_start_sample_increment);
		if(_explore_mode == _explore_jumps)
			makeJumpCurves();
		else if(_explore_mode == _explore_walks)
			interpolateWalkCurves();
	}
	if(_sample_tolerance>pFloat(trajectory_planner_max_sample_tolerance))
	{
		resetSampleValues();
		makeNewGoal();
		phout<<"exceeded max tolerance so creating new jump\n";
	}

	GsString m = "makeNewTry(): ";
	m<<_tries<<" with sample tolerance: "<<_sample_tolerance<<" ";
	phout<<m<<gsnl;
}

bool TrajectoryPlanner::update()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return false;

	bool ret = false;
	HumanMotion* motion = motion_manager->currentMotion();
	if(!motion)
	{
		_running = false;
	}
	if(_running && manager->animationStep())
	{
		_plan_time +=manager->getAnimationTimeStep();
		
		if(motion->falls()) // _human->getCOM().y<pFloat(trajectory_planner_min_com_height))
		{
			makeNewTry();
		}
		if(!motion_manager->currentController()->analyzing())
		{
			if(motion_manager->currentController()->pBool(controller_wants_static_balance))
			{
				if(_human->contact_module()->bothFeetPlanted())
				{
					bool save = true;
					ret = true;
					if(pBool(trajectory_planner_randomize_environment ))
					{
						if(motion_manager->currentController()->achievesGoal())
						{
							
						}
						else
						{
							phout<<"didn't move far enough :" << motion_manager->currentController()->getDescriptorValue("distance")<<" min dist:" <<pFloat(trajectory_planner_min_distance)-0.3f<<gsnl;
							save = false;
						}
						if(motion_manager->currentController()->getDescriptorValue("com_contact_dist")<0.15f)
						{
							
						}
						else
						{
							phout<<"the com was not good enough "<<motion_manager->currentController()->getDescriptorValue("com_contact_dist")<<gsnl;
						//	save = false;
						}

					
					}
					if(save)
					{
						_numFound++;
						phout<<"Found a Jump "<<gsnl;
						motion->setP(motion_success,1);

						GsString s;
						int sze = motion->sizeOfParameter(motion_descriptor_avg);
						for (int ds = 0;ds<sze;ds++)
						{
							s<<(int)(abs(motion->pFloat(motion_descriptor_avg,ds))*100)<<"_";
						}
						GsString dir = motion_manager->currentController()->getDirectoryName();
						dir<<s<<".motion";
						phout<<"saving file "<<dir<<gsnl;
						manager->getFiles()->saveMotionTrajectories(dir,motion);

						if(manager->pBool(human_manager_graphics_active))
							manager->message("found motion and saved ",s);

						
						makeNewGoal();
						
						resetSampleValues();
					}

				}
				else
				{
					phout<<"feet not planted\n";
				}

			}
			else if(_human->isStanding())
			{
				
				if(motion_manager->currentController()->getDescriptorValue("distance")>pFloat(trajectory_planner_min_distance))
				{
					
					GsString s;
					int sze = motion_manager->currentController()->sizeOfParameter(controller_parameters);
					for (int ds = 0;ds<sze;ds++)
					{
						s<<(int)(abs(motion->pFloat(motion_descriptor_avg,ds))*100)<<"_";
					}
					GsString dir = motion_manager->currentController()->getDirectoryName();
					dir<<s<<".motion";
					phout<<"saving file "<<dir<<gsnl;
					manager->getFiles()->saveMotionTrajectories(dir,motion);
					
					if(manager->pBool(human_manager_graphics_active))
						manager->message("found motion and saved ",s);


					makeNewGoal();

					resetSampleValues();

					ret = true;
				}
				else 
				{
					phout<<motion_manager->currentController()->getDescriptorValue("distance") << "too short discarding\n";
				}
			}
		
			
			makeNewTry();
			
		}
	}
	return ret;
}



void TrajectoryPlanner::start()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

	if(motion_manager->currentMotion())
	{
		gs_rseed(pInt(trajectory_planner_seed));
		_human = manager->selectedCharacter();
		_running = true;
		_plan_time = 0;
		_numFound = 0;
		
		motion_manager->loops(false);
		manager->setP(human_manager_auto_reset_from_hip_height,false);
		resetSampleValues();
		makeNewGoal();
		motion_manager->setRunning(true);
	}
	else 
		manager->message("Must have motion selected");

	
}
void TrajectoryPlanner::stop()
{
	_running = false;
}

void TrajectoryPlanner::init( HumanManager* mngr )
{
	manager = mngr;
	
	
	_sample_tolerance = 0.01f;
	manager->getRoot()->add(goal_bound_max);
	manager->getRoot()->add(goal_bound_min);
	_goal_bound_line = new SnLines ;
	_goal_bound_line->visible(false);
	manager->getRoot()->add(_goal_bound_line);
}
void TrajectoryPlanner::makeJumpCurves()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

	phout<<"makeJumpCurves()\n";
	GsArray<float> fs;
	GsArray<float> ws;

	int distId = motion_manager->currentController()->getDescriptorIndex("distance");
	int heightID = motion_manager->currentController()->getDescriptorIndex("height");
	int energyID = motion_manager->currentController()->getDescriptorIndex("energy");
	int com_offsetID = motion_manager->currentController()->getDescriptorIndex("com_contact_dist");
	for (int i=0;i<max(max(heightID,distId),energyID)+1;i++)
	{
		fs.push(0);
		ws.push(0);
	}

	fs.set(distId,motion_manager->currentMotion()->getEnv().z);
	ws.set(distId,1);
	fs.set(heightID,motion_manager->currentMotion()->getEnv().y);
	ws.set(heightID,1);
	fs.set(energyID,0);
	ws.set(energyID,pFloat(trajectory_planner_energy_weight));
	fs.set(com_offsetID,0);
	ws.set(com_offsetID,1);

	motion_manager->interpolatMotion(fs,ws,motion_manager->currentController()->currentMotionID(), pInt(trajectory_planner_max_interpolation_motions));
	motion_manager->makeLocalSample(_sample_tolerance);
}


void TrajectoryPlanner::resetSampleValues()
{
	_tries = 0;
	_sample_tolerance = pFloat(trajectory_planner_start_sample_tolerance);
}

void TrajectoryPlanner::makeNewGoal()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

	if(_explore_mode == _explore_jumps)
	{
		//create a new motion that is a copy of the current one
		motion_manager->currentController()->makeMotion(motion_manager->currentMotion());
		HumanMotion* m = motion_manager->currentMotion();

		motion_manager->connectMotion(m);

		//make a random _goal_position based on the sample bounds in pref.dat
		float width = pFloat(trajectory_planner_sample_bounds,1) - pFloat(trajectory_planner_sample_bounds,0) ;
		GsVec _goal_position;
		_goal_position.z =  pFloat(trajectory_planner_sample_bounds,0) +gs_random(0.0f,width);
		_goal_position.y = gs_random(pFloat(trajectory_planner_sample_bounds,2),pFloat(trajectory_planner_sample_bounds,3));
		m->getFloatParameter(motion_descriptor_env)->val.size(2);
		phout<<"new goal position: "<<_goal_position<<gsnl;
		m->selectEnv(-2);
		m->setEnv(_goal_position,-2);
	
		//create the curves by interpolating the database. the first try is a direct interpolation
		makeJumpCurves();

		motion_manager->currentController()->needsReset(true);
		motion_manager->currentController()->startAnalysis();
	}
	else if(_explore_mode == _explore_walks)
	{
		interpolateWalkCurves();
	}
	else if(_explore_mode == _randomize_motion)
	{
		
		motion_manager->currentMotion()->setMotionName(randomString());
		motion_manager->randomize();
		motion_manager->currentController()->initializeAnalysis();
		motion_manager->currentController()->startAnalysis();
	
	}
}

void TrajectoryPlanner::interpolateWalkCurves()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

	GsArray<float> fs;
	GsArray<float> ws;

	int speedId = motion_manager->currentController()->getDescriptorIndex("speed");
	int energyID = motion_manager->currentController()->getDescriptorIndex("energy");

	for (int i=0;i<max(speedId,energyID)+1;i++)
	{
		fs.push(0);
		ws.push(0);
	}

	fs.set(speedId,motion_manager->currentMotion()->pFloat(motion_descriptor_env));
	ws.set(speedId,1);
	fs.set(energyID,0);
	ws.set(energyID,pFloat(trajectory_planner_energy_weight));

	motion_manager->interpolatMotion(fs,ws,motion_manager->currentController()->currentMotionID(), pInt(trajectory_planner_max_interpolation_motions));
	
	motion_manager->currentMotion()->setP(motion_success,0);
	motion_manager->makeLocalSample(_sample_tolerance);
}

bool TrajectoryPlanner::running()
{
	return _running;
}

void TrajectoryPlanner::updateSampleLine()
{

	
	GsVec pmax =  goal_bound_max->globalPosition();
	GsVec pmin =  goal_bound_min->globalPosition();
	_goal_bound_line->color(GsColor::red);
	_goal_bound_line->init();

	_goal_bound_line->begin_polyline();
	_goal_bound_line->push(GsVec(0.0f,pmax.y,pmax.z));
	_goal_bound_line->push(GsVec(0.0f,pmax.y,pmin.z));
	_goal_bound_line->push(GsVec(0.0f,pmin.y,pmin.z));
	_goal_bound_line->push(GsVec(0.0f,pmin.y,pmax.z));
	_goal_bound_line->push(GsVec(0.0f,pmax.y,pmax.z));
	_goal_bound_line->end_polyline();

	setP(trajectory_planner_sample_bounds,pmin.z,0);
	setP(trajectory_planner_sample_bounds,pmax.z,1);
	setP(trajectory_planner_sample_bounds,pmin.y,2);
	setP(trajectory_planner_sample_bounds,pmax.y,3);

}

void TrajectoryPlanner::toggleBoundLines()
{
	bool vis = !_goal_bound_line->visible();

	_goal_bound_line->visible(vis);
	goal_bound_max->visible(vis);
	goal_bound_min->visible(vis);
	updateSampleLine();
}


