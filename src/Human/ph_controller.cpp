#include "ph_controller.h"
#include "ph_motion.h"
#include "util_channel_traj.h"
#include "ph_human.h"
#include "ph_joint.h"
#include "ph_mod_contact.h"
#include "ph_manager.h"

HumanMotion* Controller::getMotion( int idx )
{
	return _motions[idx];
}
Controller::Controller(PhysicalHuman* human, GsString filename ) :Serializable("Controller")
{
	_human = human;
	_current_motion = -1;
	_need_save = false;
	_needs_reset= false;
	_analyzing = false;
	_verifyingMotions = false;
	_analyzingMotions = false;
	loadParametersFromFile(filename);
	CHECK_STRING(controller_name);
	CHECK_STRING(controller_scene);
	CHECK_STRING(controller_parameters);
	CHECK_FLOAT(controller_parameter_weights);
	CHECK_BOOL(controller_load_env);
	CHECK_BOOL(controller_wants_static_balance);
	CHECK_FLOAT(controller_analyze_time);
	CHECK_FLOAT(controller_verify_time);
	CHECK_INT(controller_num_test);
	CHECK_BOOL(controller_accumulate_descriptors);
	CHECK_FLOAT(controller_max_com_contact_dist);
	verifyParameters();
}

Controller::Controller(PhysicalHuman* human):Serializable("Controller")
{
	_human = human;
	_current_motion = -1;
		_needs_reset= false;
	_need_save = false;
	_analyzing = false;
	_verifyingMotions = false;
	_analyzingMotions = false;
	MAKE_PARM(controller_name,"Controller");
	MAKE_PARM(controller_scene,"FlatGround");
	MAKE_TEMP_PARM(controller_parameters,"empty");
	MAKE_TEMP_PARM(controller_parameter_weights,0.0f);
	MAKE_TEMP_PARM(controller_load_env,false);
	MAKE_TEMP_PARM(controller_wants_static_balance,false);
	MAKE_TEMP_PARM(controller_analyze_time,4.0f);
		MAKE_TEMP_PARM(controller_verify_time,4.0f);
		MAKE_TEMP_PARM(controller_accumulate_descriptors,false);
		MAKE_TEMP_PARM(controller_max_com_contact_dist,0.1f);
}

void Controller::pushMotion( HumanMotion* motion )
{
	_motions.push(motion);
	_current_motion = _motions.size()-1;
}

Controller::~Controller()
{

	for(int i=0;i<_motions.size();i++)
	{
		delete _motions[i];
	}
}

int Controller::numMotions()
{
	return _motions.size();
}

bool Controller::interpolateMotion( float parm )
{
	int lowerIdx = -1;
	float lowerVal = 0;
	int upperIdx = -1;
	float upperVal = 100000;

	for (int i=0;i<numMotions();i++)
	{
		HumanMotion* m = getMotion(i);
		float v = m->pFloat(motion_descriptor_avg);
		if(v<parm && v>lowerVal)
		{
			lowerVal = v;
			lowerIdx = i;
		}
		if(v>parm && v<upperVal)
		{
			upperVal = v;
			upperIdx = i;
		}
	}

	if(lowerIdx == -1 || upperIdx == -1)
	{
		phout<<"could not find two motions to interpolate\n";
		return false;
	}
	HumanMotion* motion1 = _motions[lowerIdx];
	HumanMotion* motion2 = _motions[upperIdx];

	if(motion1->numChannels()!= motion2->numChannels())
	{
		phout<<"error motions channel number doesn'new_trajectory match ";
		return false;
	}

	
	float weight = (upperVal - parm)/(upperVal-lowerVal);

	HumanMotion* m = new HumanMotion(motion1);
	GsString name;
	name <<(int)parm;
	m->setMotionName(name);
	m->setP(motion_success,0);
	phout<<"new neighbor_motion "<<name<<" made by interpolating motions "<<motion1->getMotionName()<<" with neighbor_motion "<<motion2->getMotionName()<<" with weight "<<weight<<gsnl;

	for (int i=0;i<m->numChannels();i++)
	{
		Channel* ch = m->getChannel(i);
		Channel* ch1 = motion1->getChannel(i);
		Channel* ch2 = motion2->getChannel(i);

		ch->setControlVal(ch1->getControlVal()*weight + (1-weight)*ch2->getControlVal());

		if(ch->isTrajectory())
		{
			Trajectory* tch = ((TrajectoryChannel*)ch)->getCurve();
			Trajectory* t1 =  ((TrajectoryChannel*)ch1)->getCurve();
			Trajectory* t2 =  ((TrajectoryChannel*)ch2)->getCurve();

			for (int i=0;i<tch->numPoints();i++)
			{
				tch->setPoint(i,t1->getPoint(i)*weight+t2->getPoint(i)*(1-weight));
			}
			for (int i=0;i<tch->numTangents();i++)
			{
				tch->setTangent(i,t1->getTangent(i)*weight + t2->getTangent(i)*(1-weight));
			}
			tch->update();
			((TrajectoryChannel*)ch)->copyFromTrajectory();
		}
		
		
	}
	
	
	_motions.push(m);
	_current_motion = _motions.size()-1;
	return true;
}
struct mo_dist
{
	float rb_ds;
	int mot;
};

static int mocomp ( const mo_dist* pt1, const mo_dist* pt2 )
{
	return ((mo_dist*)pt1)->rb_ds < ((mo_dist*)pt2)->rb_ds;
}

inline float DS(const GsArray<float>& p1,const GsArray<float>& w1,const GsArray<float>& p2)
{
	if(p1.size() > p2.size())
	{
		phout<<"p2 is missing parameters\n";
		return 0;
	}
	float d = 0;
	for (int i=0;i<p1.size();i++)
	{
		d += (p1[i]-p2[i])*(p1[i]-p2[i])*w1[i];
	}

	return sqrtf(d);
}
void Controller::selectClosest(GsVec goal)
{
	int closestID = -1;
	float closestDist = 10000;
	for (int i=0;i<numMotions();i++)
	{
		HumanMotion* m = getMotion(i);
		GsVec mgoal = GsVec(0.0f,m->pFloat(motion_descriptor_env,1),m->pFloat(motion_descriptor_env,0));
		mgoal -= goal;
		if(mgoal.len()<closestDist)
		{
			closestDist = mgoal.len();
			closestID = i;
		}
		
	}
	if(closestID!=-1)
	{
		phout<<"closest motion to goal is "<<getMotion(closestID)->getMotionName()<<gsnl;
		selectMotion(closestID);
	}
}
void Controller::selectClosest( GsArray<float> parms, GsArray<float> weights )
{
	GsArray<mo_dist> neighbor_motions;
	int numDesc = parms.size();
	neighbor_motions.size(0);
	for (int i=0;i<numMotions();i++)
	{
		HumanMotion* m = getMotion(i);
		neighbor_motions.push().rb_ds = DS(parms,weights,*m->pFloatArray(motion_descriptor_avg));
		neighbor_motions.top().mot = i;
	}
	/*sort the motions in order of descending influence*/
	neighbor_motions.sort(&mocomp);
	selectMotion(_motions[neighbor_motions[0].mot]->getMotionName());
	phout<<"selecting closest motion "<<_motions[neighbor_motions[0].mot]->getMotionName()<<gsnl;
}

bool Controller::interpolateMotion(HumanMotion* new_motion,const GsArray<float>& parms,const GsArray<float>& weights,float support,int max_neighbors)
{
	GsArray<mo_dist> neighbor_motions;
	int numDesc = parms.size();
	neighbor_motions.size(0);
	for (int i=0;i<numMotions();i++)
	{
		HumanMotion* m = getMotion(i);
		if(m!=new_motion)//make sure you don't consider this motion in the interpolation
		{
			neighbor_motions.push().rb_ds = RB(DS(parms,weights,*m->pFloatArray(motion_descriptor_avg)),support,1.0f);
			neighbor_motions.top().mot = i;
		}
	}
	/*sort the motions in order of descending influence*/
	neighbor_motions.sort(&mocomp);
	
	if(neighbor_motions.size()>max_neighbors)
	{
		neighbor_motions.size(max_neighbors);
	}

	//normalize the weights
	float total = 0;
	for(int i=0;i<neighbor_motions.size();i++)
	{
		total+=neighbor_motions[i].rb_ds;
	}
	for(int i=0;i<neighbor_motions.size();i++)
	{
		neighbor_motions[i].rb_ds /= total;
	}
	
// // 	for (int i=0;i<parms.size();i++)
// // 	{
// // 		phout<<parms.get(i)<<" ";
// // 	}
// // 	phout<<gsnl;
// 
// 	int distID = getDescriptorIndex("distance");
// 	int heightID = getDescriptorIndex("height");
// // 	phout<<"to get motion with position "<<parms.get(distID) <<" , "<<parms.get(heightID)<< " " << max_neighbors<<" were used: "<<gsnl;
// // 	for (int i=0;i<neighbor_motions.size();i++)
// // 	{
// // 	phout<<"neighbor "<<i<<" env: "<< _motions[neighbor_motions[i].mot]->pFloat(motion_descriptor_env,0)  <<" , "<<_motions[neighbor_motions[i].mot]->pFloat(motion_descriptor_env,1)<<gsnl;
// // 	}
// //  	phout<<gsnl;

	new_motion->setP(motion_descriptor_avg,parms);
	for (int i=0;i<new_motion->numChannels();i++)
	{
		Channel* new_ch = new_motion->getChannel(i);
		float new_channel_val = 0;
		for (int j=0;j<neighbor_motions.size();j++)
		{
			HumanMotion* neighbor_motion = _motions[neighbor_motions[j].mot];
			Channel* neighbor_channel = neighbor_motion->getChannel(i);
			new_channel_val += neighbor_motions[j].rb_ds*neighbor_channel->getControlVal();
		}
		new_ch->setControlVal(new_channel_val);

		if(new_ch->isTrajectory())
		{
			TrajectoryChannel* new_tch = (TrajectoryChannel*)new_ch;
			Trajectory* new_trajectory = new_tch->getCurve();
		
			for (int cp=0;cp<new_trajectory->numPoints();cp++)
			{
				if(new_trajectory->sample(cp))
				{
					GsVec2 interpolated_point;
					float interpolated_tangent = 0;
					for (int j=0;j<neighbor_motions.size();j++)
					{
						HumanMotion* neighbor_motion = _motions[neighbor_motions[j].mot];
						TrajectoryChannel* neighbor_trajectory_channel =  (TrajectoryChannel*)neighbor_motion->getChannel(i);
						Trajectory* neighbor_trajectory = neighbor_trajectory_channel->getCurve();

						interpolated_point.x += neighbor_motions[j].rb_ds*neighbor_trajectory->getPoint(cp).x;
						interpolated_point.y += neighbor_motions[j].rb_ds*neighbor_trajectory->getPoint(cp).y;
						if(new_trajectory->numTangents()>cp)
						{
							interpolated_tangent+=neighbor_motions[j].rb_ds*neighbor_trajectory->getTangent(cp);
						}
					}
				//	phout<<"interpolated point "<<interpolated_point.x<<" "<<interpolated_point.y<<gsnl;
					new_trajectory->setPoint(cp,interpolated_point);
					if(new_trajectory->numTangents()>cp)
					{
						new_trajectory->setTangent(cp,interpolated_tangent);
					}
				}
			}
			new_trajectory->update();
			((TrajectoryChannel*)new_tch)->copyFromTrajectory();
		}
	}
	return true;
}
int Controller::currentMotionID()
{
	return _current_motion;
}
void Controller::setDirectoryName( const GsString&  dirname )
{
	_directory_name = dirname;
}

int Controller::selectMotion( const GsString&  s )
{
	for (int i=0;i<_motions.size();i++)
	{
		if(_motions[i]->getMotionName()==s)
		{
			selectMotion(i);
			return i;
		}
	}
	return -1;
}

void Controller::selectMotion( int i )
{
	if(currentMotion())
	{
		if(currentMotion()->tempEnv())
		{
			currentMotion()->removeEnv();
		}
	}
	if(i<_motions.size())
	{	
		_current_motion = i; 
		currentMotion()->selectEnv();
	}
	else {
		phout<<"this controller doesn't have that many motions\n";
		_current_motion = -1;
	}
}

void Controller::initializeAnalysis()
{

	HumanMotion* motion = currentMotion();
	if(!motion)
		return;

	int numParms = sizeOfParameter(controller_parameters);
	_parms.size(numParms);
	motion->setP(motion_success,0);
	motion->pFloatArray(motion_descriptor_avg)->size(numParms);
	motion->pFloatArray(motion_descriptor_max)->size(numParms);
	motion->pFloatArray(motion_descriptor_min)->size(numParms);
	
// 	for (int i=0;i<numParms;i++)
// 	{
// 		motion->pFloatArray(motion_descriptor_avg)->set(i,0.0f);
// 		motion->pFloatArray(motion_descriptor_max)->set(i,0.0f);
// 		motion->pFloatArray(motion_descriptor_min)->set(i,0.0f);
// 	}
}
void Controller::startAnalysis()
{
	HumanMotion* motion = currentMotion();
	if(!motion)
		return;

	motion->falls(false);
	
	motion->setPlayTime(0);
	_analyzing = true;
	analyze_count = 0;
	int numParms = sizeOfParameter(controller_parameters);
	_parms.size(contdes_num);
	for (int i=0;i<contdes_num;i++)
	{
		motion->setP(motion_descriptor_avg,0.0f,i);
		setDescriptorActive((controller_descriptors)i,false);
		setCurrentDescriptorValue((controller_descriptors)i,0.0f);
	}
	for (int i=0;i<numParms;i++)
	{
		if(pString(controller_parameters,i)=="com_contact_dist")
		{
			setDescriptorActive(contdes_com_contact_dist,true);
			setCurrentDescriptorValue(contdes_com_contact_dist,0);
		}
		else if (pString(controller_parameters,i)=="distance")
		{
			setDescriptorActive(contdes_distance,true);
			setCurrentDescriptorValue(contdes_distance,_human->stancePoint().z);
		}
		else if (pString(controller_parameters,i)=="lat_distance")
		{
			setDescriptorActive(contdes_lat_distance,true);
			setCurrentDescriptorValue(contdes_lat_distance,_human->stancePoint().x);
		}
		else if (pString(controller_parameters,i)=="heading")
		{
			setDescriptorActive(contdes_headin,true);
			setCurrentDescriptorValue(contdes_headin,_human->getHeadingAngle());
		}
		else if(pString(controller_parameters,i) == "speed")
		{
			setDescriptorActive(contdes_speed,true);
			setCurrentDescriptorValue(contdes_speed,_human->getCOM().z);
		}
		else if (pString(controller_parameters,i)=="foot_offset")
		{
			setDescriptorActive(contdes_foot_offset,true);
			setCurrentDescriptorValue(contdes_foot_offset,_human->leftFoot()->getCOMPosition().z -  _human->rightFoot()->getCOMPosition().z);
		}
		else if(pString(controller_parameters,i)=="jump_height")
		{
			setDescriptorActive(contdes_jump_height,true);
			setCurrentDescriptorValue(contdes_jump_height,_human->stancePoint().y);
		}
		else if(pString(controller_parameters,i)=="height")
		{
			setDescriptorActive(contdes_height,true);
			setCurrentDescriptorValue(contdes_height,_human->stancePoint().y);
		}
		else if(pString(controller_parameters,i)=="energy")
		{
			setDescriptorActive(contdes_energy,true);
			setCurrentDescriptorValue(contdes_energy, _human->totalTorque());
			//phout<<"start torque "<<_human->totalTorque()<<gsnl;
		}
	}
}

bool Controller::analyzingDescriptor( controller_descriptors des )
{
	if(des<_parms.size())
	{
		return _parms[des].active;
	}
	return false;
}
float Controller::getCurrentDescriptorValue( controller_descriptors des )
{
	if(des<_parms.size())
	{
		return _parms[des].val;
	}
	return 0.0f;
}
void Controller::setCurrentDescriptorValue(controller_descriptors des, float v )
{
	if(des<_parms.size())
	{
		 _parms[des].val = v;
	}

}

void Controller::setDescriptorActive( controller_descriptors des ,bool val)
{
 _parms[des].active = val;
}

void Controller::endAnalysis(bool print)
{
	_analyzing = false;
	_need_save = true;
	int numParms = sizeOfParameter(controller_parameters);
	
	HumanMotion* motion = currentMotion();
	float v=0;

	if(motion->tempEnv())
	{
		if(!_human->isStanding() || !_human->feetPlanted())
		{
			motion->removeEnv();
		}
		else
		{
			motion->saveEnv();
			
		}
		needsReset(true);
	}

	if(analyzingDescriptor(contdes_distance))
	{
		v = _human->stancePoint().z - getCurrentDescriptorValue(contdes_distance);
		setCurrentDescriptorValue(contdes_distance,v);
	}
	if(analyzingDescriptor(contdes_height))
	{
		v = _human->stancePoint().y - getCurrentDescriptorValue(contdes_height);
		setCurrentDescriptorValue(contdes_height,v);
	}
	
	if(analyzingDescriptor(contdes_com_contact_dist))
	{
			
	}
	if(analyzingDescriptor(contdes_lat_distance))
	{
		v = _human->stancePoint().x - getCurrentDescriptorValue(contdes_lat_distance);
		setCurrentDescriptorValue(contdes_lat_distance,v);
	}
	if(analyzingDescriptor(contdes_speed))
	{
		v = (_human->getCOM().z - getCurrentDescriptorValue(contdes_speed))/pFloat(controller_analyze_time);
		setCurrentDescriptorValue(contdes_lat_distance,v);
	}
	if(analyzingDescriptor(contdes_headin))
	{
		v = _human->getHeadingAngle() - getCurrentDescriptorValue(contdes_headin);
		setCurrentDescriptorValue(contdes_headin,v);
	}
	if(analyzingDescriptor(contdes_foot_offset))
	{
		v = _human->leftFoot()->getCOMPosition().z -  _human->rightFoot()->getCOMPosition().z;
		setCurrentDescriptorValue(contdes_foot_offset,v);
	}
		
	if(analyzingDescriptor(contdes_energy))
	{
		v = getCurrentDescriptorValue(contdes_energy) / (float)analyze_count;
		v/=500.0f;
		setCurrentDescriptorValue(contdes_energy,v);
	}
	if(motion->sizeOfParameter(motion_descriptor_avg)!=numParms || !pBool(controller_accumulate_descriptors))
	{
		motion->setP(motion_success,0);
		motion->pFloatArray(motion_descriptor_avg)->size(numParms);
		motion->pFloatArray(motion_descriptor_min)->size(numParms);
		motion->pFloatArray(motion_descriptor_min)->size(numParms);

		initDescriptor(motion, contdes_distance, "distance");		
		initDescriptor(motion, contdes_height, "height");			
		initDescriptor(motion, contdes_energy, "energy");			
		initDescriptor(motion, contdes_com_contact_dist, "com_contact_dist");			
		initDescriptor(motion, contdes_lat_distance, "lat_distance");			
		initDescriptor(motion, contdes_headin, "heading");			
		initDescriptor(motion, contdes_speed, "speed");
		initDescriptor(motion, contdes_foot_offset, "foot_offset");
		initDescriptor(motion, contdes_jump_height, "jump_height");	
	}
	else
	{
		updateDescriptor(motion, contdes_distance, "distance");		
		updateDescriptor(motion, contdes_height, "height");			
		updateDescriptor(motion, contdes_energy, "energy");			
		updateDescriptor(motion, contdes_com_contact_dist, "com_contact_dist");			
		updateDescriptor(motion, contdes_lat_distance, "lat_distance");			
		updateDescriptor(motion, contdes_headin, "heading");			
		updateDescriptor(motion, contdes_speed, "speed");
		updateDescriptor(motion, contdes_foot_offset, "foot_offset");
		updateDescriptor(motion, contdes_jump_height, "jump_height");
		motion->setP(motion_success,motion->pInt(motion_success)+1);
		if(motion->pInt(motion_success) == 1)
		{
			setEnvFromDescriptor();
		}
	}
		
	if(print)
	{
		phout<<"done analysis for motion "<<motion->getMotionName()<<gsnl;
		for (int i=0;i<numParms;i++)
		{
			phout<<pString(controller_parameters,i)<<" : "<<motion->pFloat(motion_descriptor_avg,i)<<gsnl; // " min :"<< motion->pFloat(motion_descriptor_min,i) <<"   max:"<<motion->pFloat(motion_descriptor_max,i)<<gsnl;
		}
		phout<<gsnl;
	}
}
void Controller::initDescriptor( HumanMotion* motion, controller_descriptors v, GsString label)
{
	if(analyzingDescriptor(v))
	{
		float val = getCurrentDescriptorValue(v);
		int idx = getDescriptorIndex(label);
		motion->setP(motion_descriptor_min,val,idx);
		motion->setP(motion_descriptor_max,val,idx);
		motion->setP(motion_descriptor_avg,val,idx);
	}
}

void Controller::setDescriptorFromEnv()
{
	for (int i=0;i<numMotions();i++)
	{
		HumanMotion* motion = getMotion(i);
		motion->setP(motion_descriptor_avg,motion->pFloat(motion_descriptor_env,0),getDescriptorIndex("distance"));
		motion->setP(motion_descriptor_avg,motion->pFloat(motion_descriptor_env,1),getDescriptorIndex("height"));
	}
}
void Controller::setEnvFromDescriptor()
{
	for (int i=0;i<numMotions();i++)
	{
		HumanMotion* motion = getMotion(i);
		motion->setP(motion_descriptor_env,motion->pFloat(motion_descriptor_avg,getDescriptorIndex("distance")),0);
		motion->setP(motion_descriptor_env,motion->pFloat(motion_descriptor_avg,getDescriptorIndex("height")),1);
	}
}

void Controller::analyzeFrame()
{
	if(!_human->isStanding() )
	{
		currentMotion()->falls(true);
	}
	if(_human->nonFootContact())
	{
		phout<<"non foot contact \n";
		currentMotion()->falls(true);
	}
	
	HumanMotion* motion = currentMotion();
	analyze_count++;
	
	if(analyzingDescriptor(contdes_jump_height))
	{
		float fHeight = _human->stancePoint().y;
		if(fHeight > getCurrentDescriptorValue(contdes_jump_height))
		{
			setCurrentDescriptorValue(contdes_jump_height,fHeight);
		}
	}
	if(analyzingDescriptor(contdes_energy))
	{
		setCurrentDescriptorValue(contdes_energy,getCurrentDescriptorValue(contdes_energy) + _human->totalTorque());
		//phout<<"current torque "<<getCurrentDescriptorValue(contdes_energy)<<gsnl;
	}
	if(analyzingDescriptor(contdes_com_contact_dist))
	{
		if(!_human->flying())
		{
			GsVec vec = _human->getContactProjectionCenter()-_human->getCOMProjection();
			vec.y = 0;
			float len = vec.len();
			if(len>getCurrentDescriptorValue(contdes_com_contact_dist)) 
				setCurrentDescriptorValue(contdes_com_contact_dist,len);
			//phout<<"com offset len "<<len<<gsnl;
		}
	}
	
}

void Controller::update()
{
	if(!currentMotion())
		return;

	currentMotion()->update();
	
	
	if(verifyingAllMotions())
	{
		if(currentMotion()->playTime()>pFloat(controller_verify_time) || currentMotion()->falls())
		{
			if(currentMotion()->falls()||!achievesGoal() || getCurrentDescriptorValue(contdes_com_contact_dist)>pFloat(controller_max_com_contact_dist))
			{
				phout<<"want to delete motion "<<currentMotion()->getMotionName()<<gsnl;
				if(currentMotion()->falls())
					phout<<"it falls\n";
				if( getCurrentDescriptorValue(contdes_com_contact_dist)>pFloat(controller_max_com_contact_dist))
					phout<<"it's com is to far off "<< getCurrentDescriptorValue(contdes_com_contact_dist)<<gsnl;

				_motions_to_delete.push(currentMotion()->getMotionName());
				delete currentMotion();
				_motions.remove(_current_motion);
			}
			else
			{
				_current_motion++;
				if(currentMotion())
					currentMotion()->selectEnv();
			}

			if (_current_motion > _last_motion)
			{
				_current_motion = -1;
				
			}
			else
			{
				_needs_reset = true;
				
				//phout<<"testing next current_motion "<<currentMotion()->getMotionName()<<gsnl;
			}
		}
	}
	if(currentMotion())
	{
		if(analyzingMotion())
		{
			analyzeFrame();
			if(currentMotion()->playTime()>pFloat(controller_analyze_time))
			{
				endAnalysis();
			}
		}
	}
	else
	{
		_verifyingMotions = false;
		_analyzingMotions = false;
	}
	
}

void Controller::configureBounds(float buf)
{
	if(numMotions()==0)
	{
		phout<<"there is only one current_motion in this controller so sample bounds would be a point\n";
		return;
	}
	HumanMotion* m = currentMotion(); 

	for (int chid=0;chid<m->numChannels();chid++)
	{
		if(m->getChannel(chid)->isTrajectory())
		{
			TrajectoryChannel* tch = (TrajectoryChannel*)m->getChannel(chid);
			if(tch->pBool(trajectory_channel_lock_reconfigure))
			{

			}
			else
			{

				for (int j=0;j<tch->sizeOfParameter(trajectory_channel_sample_indexes);j++)
				{
					int sid = tch->pInt(trajectory_channel_sample_indexes,j);
					GsVec2 avg;
					GsVec2 minP = tch->getCurve()->getPoint(sid);
					GsVec2 maxP = tch->getCurve()->getPoint(sid);
					for (int i=0;i<numMotions();i++)
					{
						TrajectoryChannel* tch = (TrajectoryChannel*)getMotion(i)->getChannel(chid);
						GsVec2 p = tch->getCurve()->getPoint(sid);
						if(p.x<minP.x)
							minP.x = p.x;
						if(p.y<minP.y)
							minP.y = p.y;
						if(p.x>maxP.x)
							maxP.x = p.x;
						if(p.y>maxP.y)
							maxP.y = p.y;
						avg+=p;
					}
					avg/=(float)numMotions();
					maxP -= avg;
					minP -= avg;
					maxP += GsVec2(buf,buf);
					minP -= GsVec2(buf,buf);
					//phout<<"avg point is "<<avg <<" minP is "<<minP<<" maxP is "<<maxP<<gsnl;

					for (int i=0;i<numMotions();i++)
					{
						TrajectoryChannel* tch = (TrajectoryChannel*)getMotion(i)->getChannel(chid);
						Trajectory* t = tch->getCurve();
						for(int k=0;k<t->numSamplePoints();k++)
						{
							if(t->sample(k)->index == sid)
							{
								sample_data* s = t->sample(j);
								s->rest = avg;
								s->min = minP;
								s->max = maxP;
							}
						}
					}
				}
			}
		}
	}
	for (int i=0;i<numMotions();i++)
	{
		for (int j=0;j<getMotion(i)->numChannels();j++)
		{
			if(getMotion(i)->getChannel(j)->isTrajectory())
			{
				TrajectoryChannel* tch = (TrajectoryChannel*)getMotion(i)->getChannel(j);
				tch->copyFromTrajectory();
			}
		}
	}
}

float Controller::getDescriptorValue( const GsString&  name )
{
	if(currentMotion())
	{
		int id = getDescriptorIndex(name);
		if(id!=-1)
			return currentMotion()->pFloat(motion_descriptor_avg,id);
		
	}

	return 0.0f;
}
void Controller::setDescriptorValue( const GsString&  name,float val )
{
	if(currentMotion())
	{
		int id = getDescriptorIndex(name);
		if(id!=-1)
			return currentMotion()->setP(motion_descriptor_avg,val,id);

	}

}

int Controller::getDescriptorIndex( const GsString&  param1 )
{
	for (int i=0;i<sizeOfParameter(controller_parameters);i++)
	{
		if(pString(controller_parameters,i)==param1)
			return i;
	}
	phout<<"couldn't find that parm\n";
	return -1;
}
void Controller::needsReset(bool re)
{
	_needs_reset = re;
}
bool Controller::needsReset()
{
	if(_needs_reset)
	{
		_needs_reset = false;
		return true;
	}
	return false;
}

void Controller::makeMotion( HumanMotion* m )
{
	_motions.push() = new HumanMotion(m);
	_current_motion = _motions.size()-1;
	currentMotion()->setMotionName(randomString());
}

void Controller::verifyMotions(int first, int last)
{
	if(first == -1)
	{
		first = 0;
		last = numMotions()-1;
	}
	if(last>=numMotions())
		last = numMotions()-1;

	selectMotion(first);
	currentMotion()->selectEnv();
	_first_motion = first;
	_last_motion = last;
	_verifyingMotions =true;
	_needs_reset = true;
	_motions_to_delete.size(0);
	startAnalysis();
}

void Controller::analyzeMotions(int first, int last)
{
	if(first == -1)
	{
		first = 0;
		last = numMotions()-1;
	}
	if(last>=numMotions())
		last = numMotions()-1;

	_first_motion = first;
	_last_motion = last;

	_analyzingMotions =true;
	selectMotion(0);
	currentMotion()->selectEnv();
	_needs_reset = true;
	_motions_to_delete.size(0);
	initializeAnalysis();
	startAnalysis();
}

GsString Controller::getMotionScene()
{
	return pString(controller_scene);
}

void Controller::updateDescriptor( HumanMotion* motion, controller_descriptors des, GsString label )
{
	phout<<"updateDescriptor()\n";
	if(analyzingDescriptor(des))
	{
		float v = getCurrentDescriptorValue(des);
		int idx = getDescriptorIndex(label);
		float vmin = motion->pFloat(motion_descriptor_min,idx);
		float vmax = motion->pFloat(motion_descriptor_max,idx);
		if(v<vmin)
			motion->setP(motion_descriptor_min,v,idx);
		if(v>vmax)
			motion->setP(motion_descriptor_max,v,idx);
		motion->setP(motion_descriptor_avg,v,idx);
	}
}

bool Controller::achievesGoal()
{
	bool achievesGoal = false;
	GsVec off = currentMotion()->getEnv();
	phout<<"goal "<<off<<gsnl;
	phout<<"stance "<<_human->stancePoint()<<gsnl;
	off -= _human->stancePoint();
	if (off.len()<0.4f)
	{
		achievesGoal = true;
		phout<<"achieves goal\n";
	}
	return achievesGoal;
}

HumanMotion* Controller::currentMotion()
{
	if(_current_motion<0)
		return 0; 
	if(_current_motion>=_motions.size())
		return 0;

	return _motions[_current_motion];
}





