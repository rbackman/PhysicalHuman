#pragma once

#include "common.h"
#include "util_trajectory.h"
#include "util_serializable.h"
#include "util_channel.h"

	
class HumanMotion;
class PhysicalHuman;

enum controller_parms
{
	controller_name,
	controller_scene,
	controller_parameters,
	controller_parameter_weights,
	controller_load_env,
	controller_wants_static_balance,
	controller_analyze_time,
	controller_verify_time,
	controller_accumulate_descriptors,
	controller_num_test,
	controller_max_com_contact_dist
};

enum controller_descriptors
{
	contdes_distance,
	contdes_height,
	contdes_energy,
	contdes_com_contact_dist,
	contdes_lat_distance,
	contdes_headin,
	contdes_speed,
	contdes_foot_offset,
	contdes_jump_height,
	contdes_num
};
typedef struct _parm_desc
{
	float val;
	bool active;
}parm_desc;

class Controller : public Serializable
{
	int _current_motion;
	int _first_motion;
	int _last_motion;
	GsArray<HumanMotion*> _motions;
	
	GsString _directory_name;

	bool _analyzing;
	int analyze_count;
	GsArray<parm_desc> _parms;
	PhysicalHuman* _human;
	bool _need_save;
	bool _verifyingMotions;
	bool _analyzingMotions;
public:
	
	GsStrings _motions_to_delete;
	bool _needs_reset;
	Controller(PhysicalHuman* human,GsString filename);
	Controller(PhysicalHuman* human);
	~Controller();
	HumanMotion* getMotion( int idx );
	HumanMotion* currentMotion();
	bool achievesGoal();
	void selectMotion(int i);
	int selectMotion( const GsString&  s );
	GsString getDirectoryName(){return _directory_name;}
	void pushMotion( HumanMotion* motion );
	int numMotions();
	bool interpolateMotion( float parm );
	bool interpolateMotion(HumanMotion* new_motion,const GsArray<float>& parms ,const GsArray<float>& weights,float support,int max_neighbors);
	void setDirectoryName( const GsString&  dirname );

	void startAnalysis();
	void analyzeFrame();
	bool analyzing(){return _analyzing;}
	void endAnalysis(bool print = false);

	void initDescriptor( HumanMotion* motion, controller_descriptors v, GsString label);

	void update();
	int currentMotionID();
	bool needsSave()
	{
		if(_need_save)
		{
			_need_save = false; 
			return true;
		} 
		return false;
	}
	void configureBounds(float buf);
	float getDescriptorValue( const GsString&  name );
	void setDescriptorValue( const GsString&  name ,float val );
	int getDescriptorIndex( const GsString&  param1 );
	bool needsReset();
	void needsReset(bool re);
	void makeMotion( HumanMotion* m );

	bool analyzingMotion(){return _analyzing;}
	bool verifyingAllMotions(){return _verifyingMotions;}
	bool analyzingAllMotions(){return _analyzingMotions;}
	void verifyMotions(int first = -1, int last= -1);
	void analyzeMotions(int first=-1, int last=-1);
	void selectClosest( GsArray<float> parms, GsArray<float> weights );
	void selectClosest(GsVec goal);
	void initializeAnalysis();
	void setDescriptorFromEnv();
	void setEnvFromDescriptor();

	GsString getMotionScene();
	bool analyzingDescriptor( controller_descriptors des );
	float getCurrentDescriptorValue( controller_descriptors des );
	void setCurrentDescriptorValue(controller_descriptors des, float v );
	void setDescriptorActive( controller_descriptors des ,bool val);
	void updateDescriptor( HumanMotion* motion, controller_descriptors des, GsString label );
	
};