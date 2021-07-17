#pragma once

#include "common.h"
#include "util_motion.h"

enum human_motion_parms
{
	human_motion_stance_start = motion_last,
	human_motion_mirror,
	human_motion_manual_com,
	human_motion_stance_swing_vec_start,
	human_motion_stance_swing_vec_end,
	human_motion_stance_com_vec_start,
	human_motion_stance_com_vec_end,
};
class TrajectoryChannel;


class HumanMotion : public Motion
{
	human_stance _startStance;
	human_stance _currentStance;
	/*in case this motion is from .s or .bvh*/
	KnMotion* _kn_motion;
	bool _falls;
	int _currentEnv;
	

public:
	bool _tempEnv;
	GsVec _tempEnvVec;

	GsPolygon env_poly_hull;
	SnLines* env_lines;

	//HumanMotion(GsString file);
	HumanMotion();
	HumanMotion(HumanMotion* m);
	human_stance startStance(){return _startStance;}
	human_stance endStance();
	bool setStance(human_stance st);
	bool isMirrored();
	bool load(GsString file);
	GsString startState();
	TrajectoryChannel* findComplement(TrajectoryChannel* cvin);
	void generalizeNames( human_stance leftStance );
	void setStartState( const GsString& stateName );
	void setTime(float t);
	void reset();
	human_stance getCurrentStance();
	float getWarpedTime( float tme );
	KnMotion* getKnMotion();
	void setKnMotion( KnMotion* km );
	void setMotionDescriptor( float dist,int idx = 0);
	void falls( bool fall ){_falls = fall;}
	bool falls(){return _falls;}
	void setPlayTime( float t );
	void setEnv( GsVec pos ,int env = -1);
	void newEnv();
	GsVec getEnv(int env = -1);
	void saveEnv();
	bool tempEnv();
	void updateEnvLines();
	void expandEnv();
	void removeEnv(int env = -1);
	GsVec envCentroid();
	void updateEnvHull();
	void selectEnv(int env = -1);
	void init();
	int numEnv();
};

