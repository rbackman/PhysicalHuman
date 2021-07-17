#include "ph_motion.h"
#include "ph_human.h"

HumanMotion::HumanMotion():Motion()
{
	_startStance = STANCE_LEFT;
	MAKE_PARM(human_motion_mirror,false);
	MAKE_PARM(human_motion_stance_start,"Left");
	MAKE_PARM(human_motion_manual_com,true);
	MAKE_PARM(human_motion_stance_swing_vec_start,GsVec());
	MAKE_PARM(human_motion_stance_swing_vec_end,GsVec());
	MAKE_PARM(human_motion_stance_com_vec_start,GsVec());
	MAKE_PARM(human_motion_stance_com_vec_end,GsVec());
		human_motion=true;
		_falls = false;
		_playTime=0;
		_currentEnv = 0;
		env_lines = new SnLines;
		_tempEnv = false;
		_kn_motion = 0;
}
HumanMotion::HumanMotion(HumanMotion* m):Motion(m)
{
	human_motion=true;
	if(pString(human_motion_stance_start)=="Left")
		_startStance = STANCE_LEFT;
	else
		_startStance = STANCE_RIGHT;
	_kn_motion = 0;
	setStance(_startStance);
	applyParameters();
	_falls = false;
	_playTime=0;
	_currentEnv = 0;
	env_lines = new SnLines;
	_tempEnv = false;
}
bool HumanMotion::load(GsString dir)
{
	if(!Motion::load(dir))
		return false;

	CHECK_STRING(human_motion_stance_start);
	CHECK_BOOL(human_motion_mirror);
	CHECK_BOOL(human_motion_manual_com);
	CHECK_VEC(human_motion_stance_swing_vec_start);
	CHECK_VEC(human_motion_stance_swing_vec_end);
	CHECK_VEC(human_motion_stance_com_vec_start);
	CHECK_VEC(human_motion_stance_com_vec_end);
	


		_currentEnv = 0;

	

	if(pString(human_motion_stance_start)=="Left")
		_startStance = STANCE_LEFT;
	else
		_startStance = STANCE_RIGHT;
	_kn_motion = 0;
	_falls = false;
	setStance(_startStance);
	verifyParameters();

	return true;
}
void HumanMotion::init()
{
	
	updateEnvHull();
	updateEnvLines();
}

void HumanMotion::generalizeNames( human_stance leftStance )
{
	setP(human_motion_mirror,true);
	for (int i=0;i<numChannels();i++)
	{
		Channel* ch = getChannel(i);
		bool changed = false;
		GsString objName;
		if(leftStance==STANCE_LEFT)
		{
			objName = ch->pString(channel_object_name);

			if(objName.replace("Left","Stance")!=-1)
			{
				GsString nm = pStringArray(motion_trajectory_list)->get(i);
				nm.replace("Left","Stance");
				pStringArray(motion_trajectory_list)->set(i,nm);
				changed = true;
			}
			if(objName.replace("Right","Swing")!=-1)
			{
				
				GsString nm = pStringArray(motion_trajectory_list)->get(i);
				nm.replace("Right","Swing");
				pStringArray(motion_trajectory_list)->set(i,nm);
				changed = true;
			}
		}
		else
		{
			objName = ch->pString(channel_object_name);

			if(objName.replace("Right","Stance")!=-1)
			{
				GsString nm = pStringArray(motion_trajectory_list)->get(i);
				nm.replace("Right","Stance");
				pStringArray(motion_trajectory_list)->set(i,nm);
				changed = true;
			}
			if(objName.replace("Left","Swing")!=-1)
			{
				GsString nm = pStringArray(motion_trajectory_list)->get(i);
				nm.replace("Left","Swing");
				pStringArray(motion_trajectory_list)->set(i,nm);
				changed = true;
			}
			
		}

		if(changed)
		{
			ch->setP(channel_general_name,objName);
			ch->setName(pString(motion_trajectory_list,i));
			ch->getParameter(channel_object_name)->save = false;
		}
		else
		{

		//	phout<<"didn't change "<<ch->toString();
		}
	}
	
}

bool HumanMotion::setStance( human_stance st )
{
	if(_currentStance != st)
	{
		_currentStance = st;
		for(int i=0;i<numChannels();i++)
		{

			Channel* c = getChannel(i);
			c->setP(channel_stance,(int)st);
			c->flip(flip);
			
		//	phout<<"general name "<<c->pString(channel_general_name)<<gsnl;
			if(c->pString(channel_general_name)!="empty")
			{
				GsString general_name = c->pString(channel_general_name);
				GsString specific_name = general_name;
				bool changed = false;
				if(specific_name!="empty")
				{
					int swing_idx = specific_name.search("Swing");
					int stance_idx = specific_name.search("Stance");
					if(swing_idx!=-1)
					{
						if (st==STANCE_LEFT)
						{
							specific_name.replace("Swing","Right");
						}
						else
						{
							specific_name.replace("Swing","Left");
						}

						changed = true;
					}
					else if(stance_idx!=-1)
					{

						if (st==STANCE_LEFT)
						{
							specific_name.replace("Stance","Left");
						}
						else
						{
							specific_name.replace("Stance","Right");
						}
						changed = true;
					}
					if(changed)
					{
						c->setP(channel_object_name,specific_name);
					}
				}
			}
		}
		return true;
	}
	return false;
}

human_stance HumanMotion::endStance()
{
	return (_startStance==STANCE_LEFT)?STANCE_RIGHT:STANCE_LEFT;
}

bool HumanMotion::isMirrored()
{
	return pBool(human_motion_mirror);
}

GsString HumanMotion::startState()
{
	return pString(motion_start_state);
}
#include "util_channel_traj.h"

TrajectoryChannel* HumanMotion::findComplement( TrajectoryChannel* cvin )
{
	GsString complement = cvin->name();

	if(complement.search("Stance")!=-1)
	{
		complement.replace("Stance","Swing");
	}
	else if(complement.search("Swing")!=-1)
	{
		complement.replace("Swing","Stance");
	}
	else if(complement.search("Left")!=-1)
	{
		complement.replace("Left","Right");
	}
	else if(complement.search("Right")!=-1)
	{
		complement.replace("Right","Left");
	}
	else
	{
		return 0;
	}
	Channel* ch = getChannel(complement);
	if(ch)
	{
		if(ch->getChannelMode()== channel_trajectory)
			return (TrajectoryChannel*)ch;
	}

	return 0;
}

void HumanMotion::setStartState( const GsString& stateName )
{
	setP(motion_start_state,stateName);
	if(stateName =="NULL")
		getParameter(motion_start_state)->save = false;
	else
		getParameter(motion_start_state)->save = true;
}

void HumanMotion::setTime( float t )
{
	if(_kn_motion)
		_kn_motion->apply(t);

	Motion::setTime(t);
	if(isMirrored())
	{
		if(flip) //this is the second half of a mirrored motion so it should be reveresed
		{
			if(setStance(endStance()))
				refreshChannels();
		}
		else
		{
			if(setStance(_startStance))
				refreshChannels();
		}
	}
}

void HumanMotion::reset()
{
	flip = false;
	if(setStance(_startStance))
		refreshChannels();
}

human_stance HumanMotion::getCurrentStance()
{
	return _currentStance;
}

float HumanMotion::getWarpedTime( float tme )
{
	if(_timewarp)
	{
		float p = tme/duration();
		return duration()*_timewarp->getVal(p);
	}
	return tme;
}

KnMotion* HumanMotion::getKnMotion()
{
	return _kn_motion;
}

void HumanMotion::setKnMotion( KnMotion* km )
{
	_kn_motion = km;
}

void HumanMotion::setMotionDescriptor( float dist,int idx )
{
	setP(motion_descriptor_avg,dist,idx);
}

void HumanMotion::setPlayTime( float t )
{
	_playTime = t;
}

void HumanMotion::setEnv( GsVec pos ,int env)
{
	if(env == -1)
		env = _currentEnv;
	if(env == -2)
		_tempEnvVec = pos;
	else
	{
		setP(motion_descriptor_env,pos.z,2*env);
		setP(motion_descriptor_env,pos.y,2*env+1);
	}

	//phout<<"set"<<env <<" "<<pos.z<<" "<<pos.y<<gsnl;

}
GsVec HumanMotion::getEnv(int env)
{
	if(env == -1)
	{
		env = _currentEnv;
		if(_currentEnv == -1)
			return envCentroid();
		if(_currentEnv == -2)
			return _tempEnvVec;
	}

	
	return GsVec(0.0f,pFloat(motion_descriptor_env,2*env+1),pFloat(motion_descriptor_env,2*env));
}
void HumanMotion::newEnv()
{
	_tempEnv = true;
	_currentEnv = -2;
	_tempEnvVec =  envCentroid();
}

void HumanMotion::selectEnv(int env)
{
	_currentEnv = env;
}
void HumanMotion::saveEnv()
{
	phout<<"saveEnv() :"<<_tempEnvVec<<gsnl;
	_tempEnv = false;
	_currentEnv = -1;
	getFloatParameter(motion_descriptor_env)->val.push(_tempEnvVec.z);
	getFloatParameter(motion_descriptor_env)->val.push(_tempEnvVec.y);
	
	updateEnvHull();
}
GsVec HumanMotion::envCentroid()
{
	GsVec centroid;
	for (int i=0;i<numEnv();i++)
	{
		centroid+=getEnv(i);
	}
	centroid/=(float)numEnv();
	return centroid;
}
void HumanMotion::expandEnv()
{
	newEnv();
	GsVec centroid = envCentroid();
	GsVec newP;
	if(numEnv()>1)
	{
		int p = (int)(gs_random()*numEnv());
		newP = getEnv(p);
		newP = newP - centroid;
		newP.len(gs_random(0.01f,pFloat(motion_env_expand_len)));
		GsQuat q(GsVec(1,0,0),gs_random(-1.0f,1.0f));
		newP = q.apply(newP);
		newP = getEnv(p)+newP;
	}
	else
	{
		GsVec p = GsVec(0.0f,gs_random(-10.0f,10.0f),gs_random(-10.0f,10.0f));
		p.len(gs_random(0.01f,pFloat(motion_env_expand_len)));
		newP = getEnv(0)+p;
	}
	_tempEnvVec = newP;
	phout<<"expandEnv() :"<<_tempEnvVec<<gsnl;
}

bool HumanMotion::tempEnv()
{
	return _tempEnv;
}

void HumanMotion::updateEnvLines()
{
	
	if(numEnv()==1)
		return;

	if(env_lines==0)
	{
		env_lines = new SnLines;
	}
	env_lines->init();
	env_lines->color(GsColor::green);
	env_lines->begin_polyline();
	//getFloatParameter(motion_descriptor_env)->val.size(env_poly_hull.size()*2);
	for (int i=0;i<numEnv();i++)
	{
		env_lines->push( getEnv(i));
		//getFloatParameter(motion_descriptor_env)->val.set(i*2,p.x);
		//getFloatParameter(motion_descriptor_env)->val.set(i*2+1,p.y);
	}
	env_lines->push(getEnv(0));
	env_lines->end_polyline();
}
void HumanMotion::updateEnvHull()
{
	GsPolygon pts;

	for(int i=0;i<numEnv();i++)
	{
		GsVec p = getEnv(i);
		pts.push(GsVec2(p.z,p.y));
	}
	pts.convex_hull(env_poly_hull);

	int nump = env_poly_hull.size();
	getFloatParameter(motion_descriptor_env)->val.size(nump*2);
	for (int i=0;i<nump;i++)
	{
		GsVec2 p = env_poly_hull.get(i);
		getFloatParameter(motion_descriptor_env)->val.set(i*2,p.x);
		getFloatParameter(motion_descriptor_env)->val.set(i*2+1,p.y);
	}
	_currentEnv = -1;
}
void HumanMotion::removeEnv(int env)
{
	phout<<"removeEnv()\n";
	_tempEnv = false;
	_currentEnv = -1;
}

int HumanMotion::numEnv()
{
	return getFloatParameter(motion_descriptor_env)->val.size()/2;
}











