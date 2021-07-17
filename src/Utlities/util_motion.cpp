
#include "util_motion.h"
#include "util.h"
#include "util_channel_traj.h"

Motion::Motion():Serializable("Motion")
{
	
	MAKE_PARM(motion_name,"NULL");

	MAKE_PARM(motion_start_state,"NULL");
	MAKE_PARM(motion_trajectory_list,"NULL");
	MAKE_TEMP_PARM(motion_node_list,"NULL");
	MAKE_PARM(motion_duration,1.0f);
	MAKE_PARM(motion_loops,false);
	MAKE_PARM(motion_descriptor_avg,0.0f);
	MAKE_PARM(motion_descriptor_min,0.0f);
	MAKE_PARM(motion_descriptor_max,0.0f);
	MAKE_PARM(motion_descriptor_env,0.0f);
	MAKE_PARM(motion_success,0);
	MAKE_TEMP_PARM(motion_time,0.0f);
	MAKE_PARM(motion_env_expand_len,0.0f);
	human_motion=false;
	flip = false;

	_needsRefresh = true;
	_active = true;
	_timewarp=0;
	_control_motion=0;
}

Motion::Motion( Motion* m ):Serializable("Motion")
{
	human_motion=false;
	flip = false;

	_needsRefresh = true;
	_active = true;
	_timewarp=0;
	_control_motion=0;

	loadParametersFromSerializable(m);
	for (int i=0;i<m->numChannels();i++)
	{
		Channel* c = m->getChannel(i);
		if(c->isTrajectory())
		{
			_channels.push(new TrajectoryChannel(c));
		}
		else
		{
			_channels.push(new Channel(c));
		}
	}
}

	void Motion::activate()
	{
		_active = true;
	}

	void Motion::initChannels()
	{
		getStringParameter(motion_trajectory_list)->init();
		getStringParameter(motion_node_list)->init();

		for (int i=0;i<_channels.size();i++)
		{
			delete _channels[i];
		}
		_channels.size(0);
	}


void Motion::pushChannel( Channel* ch )
{
	if(ch->isTrajectory())
	{
		getStringParameter(motion_trajectory_list)->add(ch->name()); 
	}
	else
	{
		getStringParameter(motion_node_list)->add(ch->name()); 
	}

	_channels.push(ch);
}

void Motion::insertChannel( Channel* ch, int idx )
{
	bool found = false;

	if(ch->isTrajectory())
	{
		getStringParameter(motion_trajectory_list)->insert(idx,ch->name());
	}
	else
	{
		getStringParameter(motion_node_list)->insert(idx,ch->name());
	}
	_channels.insert(idx) = ch;
}

GsString Motion::toString(bool printAll)
{
	GsString s = Serializable::toString(printAll);
	s<<gsnl<<gsnl;
	for (int i=0;i<_channels.size();i++)
	{
		s<<_channels[i]->toString(printAll);
	}
	return s;
}
GsString Motion::boundsString()
{
	GsString s = "Motion\n{\n";
	s<<"}\n\n";

	for (int i=0;i<_channels.size();i++)
	{
		if(_channels[i]->isTrajectory())
		{
			if(((TrajectoryChannel*)_channels[i])->samples())
			{
				s<<_channels[i]->name()<<" \n{\n";
				s<<"\t"<<_channels[i]->getParameter(trajectory_channel_sample)->getString();
				s<<"\t"<<_channels[i]->getParameter(trajectory_channel_sample_indexes)->getString();
				s<<"\t"<<_channels[i]->getParameter(trajectory_channel_sample_values)->getString();
				s<<"}\n";
			}
		}
	}
	return s;
}
bool Motion::haseTrajectories()
{
	if(sizeOfParameter(motion_trajectory_list)==0)
		return false;

	for (int i=0;i<sizeOfParameter(motion_trajectory_list);i++)
	{
		if(pString(motion_trajectory_list,i)!="NULL")
			return true;
	}
	return false;
}

bool Motion::hasNodes()
{
	if(sizeOfParameter(motion_node_list)==0)
		return false;

	for (int i=0;i<sizeOfParameter(motion_node_list);i++)
	{
		if(pString(motion_node_list,i)!="NULL")
			return true;
	}
	return false;
}



GsString Motion::baseString()
{
	GsString s = "Motion\n{\n";
	if(haseTrajectories())
		s<<"\t"<<getParameter(motion_trajectory_list)->getString();
	if(hasNodes())
		s<<"\t"<<getParameter(motion_node_list)->getString();

	s<<"\t"<<getParameter(motion_duration)->getString();

	if(pBool(motion_loops))
		s<<"\t"<<getParameter(motion_loops)->getString();
	if(pFloat(motion_env_expand_len)!=0)
		s<<"\t"<<getParameter(motion_env_expand_len)->getString();
	//s<<"\t"<<getParameter(human_motion_stance_start)->getString();
//	s<<"\t"<<getParameter(human_motion_mirror)->getString();
//	s<<"\t"<<getParameter(human_motion_manual_com)->getString();
	s<<"}\n\n";

	for (int i=0;i<numChannels();i++)
	{
		Channel* ch = getChannel(i);

		s<<ch->name()<<" \n{\n";

		if(ch->getObject())
		{
			s<<"\t"<<ch->getParameter(channel_parameter_name)->getString();
			s<<"\t"<<ch->getParameter(channel_object_name)->getString();
			s<<"\t"<<ch->getParameter(channel_object_type)->getString();
			s<<"\t"<<ch->getParameter(channel_dof)->getString();
		}
		
		

		if(ch->isTrajectory())
		{
			s<<"\t"<<ch->getParameter(trajectory_channel_type)->getString();
			s<<"\t"<<ch->getParameter(channel_phase_repetitions)->getString();
			s<<"\t"<<ch->getParameter(channel_phase_flip)->getString();
			s<<"\t"<<ch->getParameter(channel_range)->getString();
			if(ch->constantValue())
			{
				s<<"\t"<<ch->getParameter(trajectory_channel_p_t)->getString();
				s<<"\t"<<ch->getParameter(trajectory_channel_p_y)->getString();
				s<<"\t"<<ch->getParameter(trajectory_channel_tng)->getString();
			}
		}
		else
		{
		
			s<<"\t"<<ch->getParameter(channel_control_mode)->getString();
			if(ch->getChannelMode() != channel_feedback && ch->numInputs()>0) //feedback nodes dont have control input
				s<<"\t"<<ch->getParameter(channel_control_list)->getString();
			
		}
		if(ch->constantValue())
			s<<"\t"<<ch->getParameter(channel_node_val)->getString();
		if(ch->constantValue())
			s<<"\t"<<ch->getParameter(channel_val_constant)->getString();

			s<<"\t"<<ch->getParameter(channel_node_position)->getString();
		s<<"}\n";

	}
	return s;

}



GsString Motion::trajectoryString()
{
	GsString s = "Motion\n{\n";
	if(pInt(motion_success )>0)
	{
		s<<"\t"<<getParameter(motion_descriptor_avg)->getString();
		s<<"\t"<<getParameter(motion_descriptor_min)->getString();
		s<<"\t"<<getParameter(motion_descriptor_max)->getString();
		s<<"\t"<<getParameter(motion_success)->getString();
	}
	if(pFloat(motion_descriptor_env)!=0)
		s<<"\t"<<getParameter(motion_descriptor_env)->getString();
		
	s<<"}\n\n";

	for (int i=0;i<_channels.size();i++)
	{
		if(_channels[i]->isTrajectory() && !_channels[i]->constantValue())
		{
			s<<_channels[i]->name()<<" \n{\n";
			s<<"\t"<<_channels[i]->getParameter(trajectory_channel_p_t)->getString();
			s<<"\t"<<_channels[i]->getParameter(trajectory_channel_p_y)->getString();
			s<<"\t"<<_channels[i]->getParameter(trajectory_channel_tng)->getString();
			s<<"}\n";
		}
		else if(_channels[i]->getControlVal() && !_channels[i]->constantValue() && _channels[i]->getChannelMode() != channel_feedback)
		{
			s<<_channels[i]->name()<<" \n{\n";
			s<<"\t"<<_channels[i]->getParameter(channel_node_val)->getString();
			s<<"}\n";
		}
	}
	return s;
}
void Motion::pushChannels( GsArray<Channel*> chs )
{
	for(int i=0;i<chs.size();i++)
	{
		pushChannel(chs[i]);
	}
}

Motion::~Motion()
{
	for (int i=0;i<_channels.size();i++)
	{
		delete _channels[i];
	}
}
void Motion::applyParameters()
{

	//phout<<"motion "<<getMotionName()<<" time step: "<<tmstp<<"  duration: "<<dur<<gsnl;
	for(int i=0;i<_channels.size();i++)
	{
		_channels[i]->loops(loops());
		_channels[i]->duration(pFloat(motion_duration));
		_channels[i]->applyParameters();
	}
}



void Motion::reduceMotion( Motion* m_original ,int numSamples,float slope_tolerance,float merge_distance,float concavity_tolerance,bool flat)
{
	initChannels();

	duration(m_original->duration());
	for (int i=0;i<m_original->numChannels();i++)
	{
		if(m_original->getChannel(i)->isTrajectory())
		{
			phout<<"fix reduceMotion()\n";
// 			TrajectoryChannel* ch = new TrajectoryChannel(m_original->getChannel(i));
// 			ch->fitCurve(m_original->getChannel(i),numSamples,slope_tolerance,merge_distance,concavity_tolerance,flat);
// 			pushChannel(ch);
		}
	}
}

bool Motion::load( GsString file )
{
	SerializableGroup* sav_grp = new SerializableGroup();
	if(!sav_grp->loadFromFile(file))
		return false;

	Serializable* moSav = sav_grp->getSerializable("Motion");
	if(!moSav)
	{
		phout<<"couldn't find motion header\n";
		return false;
		
	}

	loadParametersFromSerializable(moSav);
	CHECK_STRING(motion_name);
	CHECK_STRING(motion_start_state);
	CHECK_STRING(motion_trajectory_list);
	CHECK_STRING(motion_node_list);
	CHECK_FLOAT(motion_duration);
	CHECK_BOOL(motion_loops);
	CHECK_FLOAT(motion_time);
	CHECK_FLOAT(motion_descriptor_avg);
	CHECK_FLOAT(motion_descriptor_min);
	CHECK_FLOAT(motion_descriptor_max);
	CHECK_FLOAT(motion_descriptor_env);
	CHECK_INT(motion_success);
	CHECK_FLOAT(motion_env_expand_len);
	if(getParameter(motion_trajectory_list)->save)
	{
		for(int i=0;i<pStringArray(motion_trajectory_list)->size();i++)
		{
			GsString cname = pString(motion_trajectory_list,i);
			Serializable* s = sav_grp->getSerializable(cname);
			if(s)
			{
				_channels.push(new TrajectoryChannel(s));

			}
			else
				phout<<"didn't find channel "<<cname<<gsnl;
		}
	}

	
	if(getParameter(motion_node_list)->save)
	{
		for(int i=0;i<pStringArray(motion_node_list)->size();i++)
		{
			GsString cname = pString(motion_node_list,i);
			Serializable* s = sav_grp->getSerializable(cname);
			if(s)
			{
				_channels.push(new Channel(s));
			}
			else
				phout<<"didn't find channel "<<cname<<gsnl;
		}
	}
	else
	{
	 
	}

	applyParameters();
	return true;
}

bool Motion::loops()
{
	return pBool(motion_loops);
}

void Motion::loops( bool lp )
{
	setP(motion_loops,lp);
	applyParameters();
}

void Motion::setTime( float t )
{
	_active = true;
	_playTime =t;
	int cycles = (int)(t/duration());

	float time = t;
	
	if(loops())
	{
		time = t - cycles*duration();
		if(cycles%2==0)
		{
			flip = false;
		}
		else
		{
			flip = true;
		}
	}
	else
	{
		if(cycles>0)
		{
			_active = false;
		}
	}
	if(_timewarp)
	{
		time = _timewarp->getVal(time);
	}
	setP(motion_time,time);
}

void Motion::update( )
{
	if(_active)
	{
		for(int j=0;j<numChannels();j++)
		{ 
			if(_channels[j]->getObject())
			{
				float offset = 0.0f;
				if(_channels[j]->pInt(channel_phase_repetitions)==2 && flip)
				{
					offset+=duration();
				}
				_channels[j]->setTime(pFloat(motion_time)+offset);
				
			}
		}	
		for(int j=0;j<numChannels();j++)
		{ 
			if(_channels[j]->getChannelMode() != channel_feedback)
			{
				Serializable* obj = _channels[j]->getObject();
				if(obj)
					obj->applyParameters();
			}
		}
	}
}




void Motion::removeChannel( int it )
{
	if(it<0 || it >= numChannels())
		return;

	for(int i=0;i<_channels.size();i++)
	{
		_channels[i]->removeInput(_channels[it]);
	}
	delete _channels[it];
	_channels.remove(it);

}
void Motion::swapChannel( int prevI ,int newI)
{
	if(prevI<numChannels() && newI<numChannels())
	{
		Channel* ch = _channels[prevI];
		_channels[prevI] = _channels[newI];
		_channels[newI] = ch;
	}
}
Channel* Motion::getChannel( GsString name )
{
	for(int i=0;i<numChannels();i++)
	{
		if(name == getChannel(i)->name())
			return getChannel(i);
	}
	//phout<<"no channel named "<<name<<gsnl;
	return 0;
}

Channel* Motion::getChannel( Channel* otherCh )
{
	for(int i=0;i<_channels.size();i++)
	{
		Channel* ch = _channels[i];
		if(ch->pString(channel_object_name) == otherCh->pString(channel_object_name))
		{
			if(ch->pString(channel_object_type) == otherCh->pString(channel_object_type))
			{
				if(ch->pString(channel_parameter_name) == otherCh->pString(channel_parameter_name))
				{
					if(ch->pString(channel_dof) == otherCh->pString(channel_dof))
					{
						return ch;
					}
				}
			}
		}
		
	}
return 0;
}

void Motion::setMotionName( GsString n )
{
	setP(motion_name,n);
}

GsString Motion::getMotionName()
{
	return pString(motion_name);
}



float Motion::duration()
{
	return pFloat(motion_duration);
}

void Motion::setControlMotion( Motion* m, bool overrideConnections)
{
	_control_motion = m;
	if(overrideConnections)
	{
		for (int i=0;i<m->numChannels();i++)
		{
			Channel* control_curve = m->getChannel(i);
			Channel* myCurve = getChannel(control_curve);
			if(myCurve)
			{
				control_curve->setObject(0);
				myCurve->pushInput(control_curve);
				myCurve->updateInputList();
				myCurve->setChannelMode(channel_additive);
			}
		}
	}
// 	}
// 	Channel* stanceChannel = m->getChannel("LeftStance");
// 	if(stanceChannel)
// 	{
// 		for (int i=0;i<numChannels();i++)
// 		{
// 			
// 
// 			Channel* myCurve = getChannel(i);
// 			GsString leftName;
// 			GsString rightName;
// 	
// 			Channel* leftChannel = 0;
// 			Channel* rightChannel = 0;
// 			
// 			bool threeChannels = true;
// 			bool StanceChannel = false;
// 
// 			if(myCurve->pString(channel_general_name).search("Swing")>=0)
// 			{
// 				
// 				rightName = myCurve->pString(channel_general_name);
// 				StanceChannel = false;
// 				leftName = 	rightName;
// 				leftName.replace("Swing","Stance");
// 			}
// 			else if(myCurve->pString(channel_general_name).search("Stance")>=0)
// 			{
// 				leftName = myCurve->pString(channel_general_name);
// 				StanceChannel = true;
// 				rightName = leftName;
// 				rightName.replace("Stance","Swing");
// 			}
// 			else
// 			{
// 				threeChannels = false;
// 			}
// 			if(threeChannels)
// 			{
// 				for (int i=0;i<m->numChannels();i++)
// 				{
// 					//first look for the 
// 					Channel* mcurve = m->getChannel(i);
// 					if(mcurve->pString(channel_general_name) == rightName)
// 					{
// 						if(mcurve->pString(channel_dof) == myCurve->pString(channel_dof))
// 						{
// 							rightChannel = mcurve;
// 						}
// 					}
// 					else if(mcurve->pString(channel_general_name) == leftName)
// 					{
// 						if(mcurve->pString(channel_dof) == myCurve->pString(channel_dof))
// 						{
// 							leftChannel = mcurve;
// 						}
// 					}
// 				}
// 				if(StanceChannel)
// 					myCurve->setControlCurve(stanceChannel,leftChannel,rightChannel);
// 				else
// 					myCurve->setControlCurve(stanceChannel,rightChannel,leftChannel);
// 			}
// 			else
// 			{
// 				Channel* cntc = m->getChannel(myCurve);
// 				if(cntc)
// 					myCurve->setControlCurve(cntc);
// 			}
// 				
// 			
// 			
// 			
// 		}
// 
// 	}
// 	else
// 	{
// 		for (int i=0;i<m->numChannels();i++)
// 		{
// 			Channel* control_curve = m->getChannel(i);
// 			Channel* myCurve = getChannel(control_curve);
// 
// 			if(myCurve)
// 			{
// 				myCurve->setControlCurve(control_curve);
// 				//this channels will effect the other so it wont be updated directly
// 			}
// 			else
// 			{
// 				//the trajectory motion doesnt have this channel so the control motion will directly effect object
// 				control_curve->activate();
// 			}
// 		}
// 	}
}
void Motion::updateChannelList()
{
	getStringParameter(motion_trajectory_list)->init();
	getStringParameter(motion_node_list)->init();
	for (int i=0;i<numChannels();i++)
	{
		if(getChannel(i)->isTrajectory())
		{
			getStringParameter(motion_trajectory_list)->add(getChannel(i)->name());
		}
		else
			getStringParameter(motion_node_list)->add(getChannel(i)->name());
	}
	if(pStringArray(motion_trajectory_list)->size()==0)
	{
		getParameter(motion_trajectory_list)->save = false;
	}
	else
	{
		getParameter(motion_trajectory_list)->save = true;
	}
	if(pStringArray(motion_node_list)->size()==0)
	{
		getParameter(motion_node_list)->save = false;
	}
	else
	{
		getParameter(motion_node_list)->save = true;
	}
	for (int i=0;i<_channels.size();i++)
	{
		_channels[i]->updateInputList();
	}
}


void Motion::setTimeWarp( Channel* ch )
{
	
	
	if(_control_motion)
	{
		_control_motion->setTimeWarp(ch);
	}
	else
	{
		ch->setName("TimeWarp");
		ch->deactivate();
		if(_timewarp)
			delete _timewarp;
		insertChannel(ch,0);
	}
	
	_timewarp = ch;
}

bool Motion::loadDataFromFile( GsString motion_files )
{
	SerializableGroup* sav_grp = new SerializableGroup();
	if(!sav_grp->loadFromFile(motion_files))
		return false;

	Serializable* moSav = sav_grp->getSerializable("Motion");
	if(!moSav)
	{
		phout<<"couldn't find motion header for "<<motion_files<<gsnl;
		return false;

	}

	setParametersFromSerializable(moSav);
	CHECK_FLOAT(motion_descriptor_avg);
	CHECK_FLOAT(motion_descriptor_min);
	CHECK_FLOAT(motion_descriptor_max);
	CHECK_FLOAT(motion_descriptor_env);
	CHECK_INT(motion_success);
	for (int i=0;i<numChannels();i++)
	{
		Channel* ch = getChannel(i);
		Serializable* s = sav_grp->getSerializable(ch->name());
		if(s)
		{
			ch->setParametersFromSerializable(s);
			ch->applyParameters();
		}
	}
		
	applyParameters();
	return true;
}



