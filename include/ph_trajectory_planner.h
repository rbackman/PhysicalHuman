
#pragma once

#include "common.h"
#include "util_serializable.h"

class HumanMotionManager;
class MotionGroup;
class PhysicalHuman;
class HumanMotion;
class HumanManager;
class Manipulator;

enum traj_plan_parms
{
	trajectory_planner_plan_time,
	trajectory_planner_min_distance,
	trajectory_planner_randomize_environment,
	trajectory_planner_min_com_height,
	trajectory_planner_tries_per_jump,
	trajectory_planner_start_sample_tolerance,
	trajectory_planner_max_sample_tolerance,
	trajectory_planner_start_sample_increment,
	trajectory_planner_max_interpolation_motions,
	trajectory_planner_sample_bounds,
	trajectory_planner_energy_weight,
	trajectory_planner_seed,
};

enum exploration_mod
{
	_explore_jumps,
	_explore_walks,
	_randomize_motion
};
class TrajectoryPlanner :public Serializable
{	
	bool _running; 
public:
	exploration_mod _explore_mode;
	Manipulator* goal_bound_max;
	Manipulator* goal_bound_min;
	SnLines* _goal_bound_line;
	int _tries; //the number of attempts to solve the current _goal_position
	int _numFound;
	float _sample_tolerance;
	float _plan_time;
	TrajectoryPlanner(GsString file);
	void init( HumanManager* mngr );

	bool update();
	void setMotion( HumanMotion* m );
	void start();
	void stop();
	bool running();
	void updateSampleLine();
	void toggleBoundLines();
private:
	
	HumanManager* manager;
	PhysicalHuman* _human;

	void makeNewTry();

	void resetSampleValues();
	void makeNewGoal();
	void interpolateWalkCurves();
	void makeJumpCurves();
	
};

