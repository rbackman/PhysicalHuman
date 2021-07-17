#include "util_channel.h"
#include "util_curve.h"
#include "util.h"
#include "util_channel_traj.h"

void Channel::setObject(Serializable* obj)
{
	_object = obj;
	
	if(_object)
	{
		setP(channel_parameter_index,_object->getParameterIndex(pString(channel_parameter_name)));
		setP(channel_object_name,obj->name());
		getParameter(channel_object_name)->save = true;
		
	}
	else
	{
		setP(channel_object_name,"none");
		getParameter(channel_object_name)->save = false;
		getParameter(channel_parameter_name)->save = false;
		getParameter(channel_object_type)->save = false;
		getParameter(channel_general_name)->save = false;
		getParameter(channel_dof)->save = false;
		getParameter(channel_float_index)->save = false;
		getParameter(channel_feedback_mode)->save = false;
	}
}


void Channel::setParameter( Serializable* sav, int param ,channel_dof_types type,int arrId)
{
	//phout<<"setting parameter "<<sav->getParameter(param)->name  << " for object type "<<sav->type()<<" with name "<<sav->name()<<gsnl;
	_object = sav;
	setP(channel_parameter_index,param);
	if(sav)
	{
		setP(channel_object_name,sav->name());
		setP(channel_object_type,sav->type());
		setP(channel_parameter_name,sav->getParameter(param)->name);
	}
	setP(channel_float_index,arrId);
	
	_channel_type =type;
	setP(channel_dof,channelTypeToString(type));

// 	GsString cname = sav->name();
// 	cname<<"_"<<pString(channel_dof);
// 	setName(cname);
	if(_channel_type == CH_FLOAT)
		getParameter(channel_float_index)->save = true;
	else
		getParameter(channel_float_index)->save = false;

	getParameter(channel_object_name)->save = true;
	getParameter(channel_parameter_name)->save = true;
	getParameter(channel_object_type)->save = true;
	getParameter(channel_dof)->save = true;
	
}




Channel::Channel(Serializable* objct,int parameter_id, channel_dof_types channelType,chanel_modes mode,int array_indx):Serializable(objct->name(),"Channel")
{
	_object = objct;
	_isTrajectory = false;

	MAKE_TEMP_PARM(channel_active,true);
	MAKE_PARM(channel_node_val,0.0f);
	MAKE_PARM(channel_val_constant,false);
	MAKE_PARM(channel_object_name,_object->name());
	MAKE_PARM(channel_object_type,_object->type());
	MAKE_PARM(channel_parameter_name,_object->getParameter(parameter_id)->name);
	//this should go to swing or stance
	GsString genName = pString(channel_object_name);
	int lC = genName.replace("Left","Stance");
	int rC =genName.replace("Right","Swing");
	if(lC!=-1 || rC !=-1)
		MAKE_PARM(channel_general_name, genName );
	else
		MAKE_TEMP_PARM(channel_general_name,genName);

	MAKE_PARM(channel_dof,channelTypeToString(channelType)); //
	_channel_type = channelType;
	if(array_indx==-1)
		MAKE_TEMP_PARM(channel_float_index,0); //
	else
		MAKE_PARM(channel_float_index,array_indx);

	MAKE_TEMP_PARM(channel_parameter_index,parameter_id);
	MAKE_TEMP_PARM(channel_stance,STANCE_LEFT);

	GsArray<float> curve_limits;
	float val = 0; //currentChannelVal();
	if(channelType == CH_BOOL)
	{
		curve_limits.push(0);curve_limits.push(val);curve_limits.push(val+1);
	}
	else if(channelType==CH_ROT_X || channelType==CH_ROT_Y || channelType==CH_ROT_Z)
	{	curve_limits.push(val-gs2pi);curve_limits.push(val);curve_limits.push(val+gs2pi);}
	else
	{	curve_limits.push(val-1);curve_limits.push(val);curve_limits.push(val+1);}

	MAKE_PARM(channel_range,curve_limits);

	MAKE_PARM(channel_phase_repetitions,1);
	MAKE_PARM(channel_phase_flip,false);

	
	GsString label = name();
	label<<"_"<<pString(channel_dof);
	setName(label);
	MAKE_TEMP_PARM(channel_control_mode,"ADD");
	MAKE_TEMP_PARM(channel_control_list,"NULL");
	MAKE_TEMP_PARM(channel_feedback_mode,false);
	MAKE_PARM(channel_node_position,GsVec());
	getParameter(channel_node_position)->save = true;
	setChannelMode(mode);
//	init();
}



void Channel::init()
{
	
	MAKE_TEMP_PARM(channel_active,true);
	CHECK_FLOAT(channel_node_val);
	CHECK_BOOL(channel_val_constant);
	CHECK_STRING(channel_parameter_name);
	CHECK_STRING(channel_object_name);
	CHECK_STRING(channel_object_type);
	CHECK_STRING(channel_general_name);
	CHECK_STRING(channel_dof);
	CHECK_FLOAT(channel_range);

	setP(channel_range,0.0f,1);

	CHECK_INT(channel_phase_repetitions);
	CHECK_BOOL(channel_phase_flip);
	_channel_type = stringToChannelType(pString(channel_dof));
	
	CHECK_INT(channel_float_index);
	CHECK_INT(channel_parameter_index);

	MAKE_TEMP_PARM(channel_stance,STANCE_LEFT);
	
	CHECK_STRING(channel_control_mode);
	CHECK_STRING(channel_control_list);
	CHECK_BOOL(channel_feedback_mode);
	CHECK_VEC(channel_node_position);
	getParameter(channel_node_position)->save = true;
	_isTrajectory = false;

	if(_channel_type == CH_FREE)
	{
		getParameter(channel_dof)->save = false;
		getParameter(channel_parameter_name)->save = false;
		getParameter(channel_object_name)->save = false;
		getParameter(channel_object_type)->save = false;
		getParameter(channel_general_name)->save = false;
	}

	if(pString(channel_control_mode) == "ADDITIVE")
		setChannelMode(channel_additive);
	else if(pString(channel_control_mode) == "SCALE")
		setChannelMode(channel_multiply);
	else if(pString(channel_control_mode) == "MODULATE")
		setChannelMode(channel_modulate);
	else if(pString(channel_control_mode) == "SWITCH")
		setChannelMode(channel_switch);
	else if(pString(channel_control_mode) == "INVERSE")
		setChannelMode(channel_inverse);
	else if(pString(channel_control_mode) == "FEEDBACK")
	{
		setChannelMode(channel_feedback);
	}
	else
		channel_mode = channel_trajectory;

	
	if(_channel_type == CH_FLOAT)
		getParameter(channel_float_index)->save = true;
	else
		getParameter(channel_float_index)->save = false;

	_flip = false;
	
	_curve_vis = false;
	_node_vis = false;
}
Channel::Channel(const GsString& file,const GsString& name):Serializable(name)
{
	_object = 0;
	loadParametersFromFile(file);
	init();
}

Channel::Channel( Serializable* sav):Serializable(sav)
{
	_object = 0;
	init();
}

Channel::Channel( Channel* ch ):Serializable(ch->name())
{
	loadParametersFromSerializable(ch);
	init();
	setParameter(ch->getObject(),ch->getParameterID(),ch->getDofType());
}

Channel::Channel( const GsString& nme, channel_dof_types channelType):Serializable(nme,"Channel")
{
	MAKE_PARM(channel_control_mode,channelTypeToString(channelType));
	init();
}



void Channel::setTime(float t)
{
	if(getChannelMode() == channel_feedback)
	{
		_currentValPt.x = 0;
		_currentValPt.y = currentChannelVal();
		setControlVal(_currentValPt.y);
	}
	else
	{
		if(getParameter(channel_phase_repetitions)->save)
			t = t/(float)pInt(channel_phase_repetitions);
		_lastTime = t;
		_currentValPt.x = t;
		_currentValPt.y = getVal(t);
		setCurrentChannelVal(_currentValPt.y);
	}
}


float Channel::currentChannelVal()
{
	if(!_object)
		return pFloat(channel_node_val);
	
	if(_channel_type==CH_FLOAT)
	{
		return ((FloatParameter*)_object->getParameter(pInt(channel_parameter_index)))->val.get(pInt(channel_float_index));
	}else if(_channel_type == CH_VEC_X || _channel_type == CH_VEC_Y||_channel_type == CH_VEC_Z)
	{
		
		GsVec v = ((VecParameter*)_object->getParameter(pInt(channel_parameter_index)))->val;
		if(pInt(channel_stance)!=STANCE_LEFT)
		{
			v.x = -v.x;
		}
		switch(_channel_type)
		{
			case CH_VEC_X: return v.x; break;
			case CH_VEC_Y: return v.y; break;
			case CH_VEC_Z: return v.z; break;
		}
	}
	else if(_channel_type == CH_ROT_X || _channel_type == CH_ROT_Y||_channel_type == CH_ROT_Z)
	{
		//phout<<name()<<gsnl;
		GsQuat q = ((QuatParameter*)_object->getParameter(pInt(channel_parameter_index)))->val;
		gsEulerOrder rotationOrder = rotation_type_to_gs_euler_order(((QuatParameter*)_object->getParameter(pInt(channel_parameter_index)))->rotationOrder);
		float rx,ry,rz;
		gs_angles(rotationOrder,q,rx,ry,rz);
		if(pInt(channel_stance)!=STANCE_LEFT)
		{
				ry= -ry;
				rz= -rz;
		}
		switch(_channel_type)
		{
		case CH_ROT_X: return rx; break;
		case CH_ROT_Y: return ry; break;
		case CH_ROT_Z: return rz; break;
		}
	}
	else if(_channel_type== CH_BOOL)
	{
		if(((BoolParameter*)_object->getParameter(pInt(channel_parameter_index)))->val)
			return 1.0f;
		else
			return 0.0f;
	}
	phout<<"error returning current channel val\n";
	return 0;


	
}

float Channel::getControlVal()
{
	return pFloat(channel_node_val);
}
float Channel::setCurrentChannelVal(float a) //input should be between mine and max
{
	
	if(!active())return 0;
	if(!_object)return 0;
	if(pBool(channel_feedback_mode))
	{
		phout<<"cant set feedback channel";
		return 0;
	}
	

	int parmID = pInt(channel_parameter_index);
	
	int arrayIndex;
	if(!getParameter(channel_parameter_index)->save)
		arrayIndex = 0;
	else
		arrayIndex = pInt(channel_float_index);

	Serializable* object = _object;
	
	if(object)
	{
		ControlParameter* parm = object->getParameter(parmID);
		if(!parm)
			return 0;

		switch(_channel_type)
		{
		case CH_BOOL:
			{
				if(a>0)
					object->setP(parmID,true);
				else
					object->setP(parmID,false);
				

			}break;
			case CH_FLOAT: 
				{
					object->setP(parmID,a,arrayIndex);

				}break;
			case CH_VEC_X: 
				{
					
					if(pInt(channel_stance)!=STANCE_LEFT)
							   a=-a;
					((VecParameter*)parm)->val.x = a; break;
				}
			case CH_VEC_Y: 
				{
					((VecParameter*)parm)->val.y = a; 
				}break;
			case CH_VEC_Z: ((VecParameter*)parm)->val.z = a; break;
			case CH_ROT_X: 
				{
					QuatParameter* quatParm = (QuatParameter*)parm;
					GsQuat q = quatParm->val;
					float rx,ry,rz;
					gsEulerOrder rotationOrder = rotation_type_to_gs_euler_order(quatParm->rotationOrder);

					gs_angles(rotationOrder,q,rx,ry,rz);
					rx = a;
					gs_rot(rotationOrder,q,rx,ry,rz);
					
					quatParm->val = q;
				}
				break;
			case CH_ROT_Y:
				{
					QuatParameter* quatParm = (QuatParameter*)parm;
					GsQuat q = quatParm->val;
					float rx,ry,rz;
					gsEulerOrder rotationOrder = rotation_type_to_gs_euler_order(quatParm->rotationOrder);

					gs_angles(rotationOrder,q,rx,ry,rz);
					
					if(pInt(channel_stance)!=STANCE_LEFT)
						a=-a;

					ry = a;
					gs_rot(rotationOrder,q,rx,ry,rz);
					
					quatParm->val = q;
				}
				break;
			case CH_ROT_Z: 
				{
					QuatParameter* quatParm = (QuatParameter*)parm;
					GsQuat q = quatParm->val;
					float rx,ry,rz;
					gsEulerOrder rotationOrder = rotation_type_to_gs_euler_order(quatParm->rotationOrder);

					gs_angles(rotationOrder,q,rx,ry,rz);
					if(pInt(channel_stance)!=STANCE_LEFT)
						a=-a;
					rz = a;
					gs_rot(rotationOrder,q,rx,ry,rz);
					
					quatParm->val = q;
				}break;
			default: phout<<"error setting current channel val" <<a<<"\n"; break;
		}

	}
	//phout<<object->toString();
	return a;

}



Channel::~Channel()
{
	
	_object = 0;
	
}

Serializable* Channel::getObject()
{
	return _object;
}


float Channel::getVal( float time )
{
	float val = pFloat(channel_node_val);
	if(pBool(channel_feedback_mode))
	{
		if(_object)
		{
			return currentChannelVal();
		}
		phout<<"feedback node should be connected to object";
		return 0;
	}
	if(_input.size()==0)
	{
		return val;
	}

	switch(channel_mode)
	{	
		case channel_trajectory:
		//this is a TrajectoryChannel so it should override getVal()
			//
			break;

		case channel_additive:
			//this will add the value of the input channels and control

			for(int i=0;i<_input.size();i++)
			{
				val += _input[i]->getVal(time);
			}
			return val;

			break;
		case channel_switch:
			//based on the value of one channel it will choose one or the other
			if(_input.size()==3)
			{
				if(_input[0]->getVal(time)>0)
				{
					return _input[1]->getVal(time);
				}
				else
				{
					return _input[2]->getVal(time);
				}
			}
			else if(_input.size()==2)
			{
				if(_input[0]->getVal(time)>0)
				{
					val = _input[1]->getVal(time);
				}
				return val;
			}
			else
			{
				phout<<"switch node should have atleast 2 inputs preferebly 3\n";
			}
			break;

		case channel_inverse:
			if(_input[0]->getVal(time)>0)
				return 0.0f;
			else
				return 0.5f;
		break;
		case channel_modulate:
			{
				if(!hasInputs())
				{
					return pFloat(channel_node_val);
				}
				if(!_input[0])
				{
					if(_input[1])
						return _input[1]->getVal(time);
					else if(_input[2])
						return _input[2]->getVal(time);
					else
						return 0;
				}
				if(_input[0]->isTrajectory())
				{
					//needs a step function to figure the phase based on the current positive step
					if(_input[0]->getVal(time)>0 && _input[1])
					{
						TrajectoryChannel* trajC = (TrajectoryChannel*)_input[0];
						float phase = trajC->getStepPhase(time);
						if(_input.size()<=1)
							return phase;

						return _input[1]->getVal(phase*_input[1]->duration());
					}
					else if(_input.size()>2 && _input[2])
					{
						TrajectoryChannel* trajC = (TrajectoryChannel*)_input[0];
						float phase = trajC->getStepPhase(time,true);
						return _input[2]->getVal(phase*_input[2]->duration());
					}
					else
					{
						return 0;
					}
				}

				if(_input[0]->getChannelMode() == channel_inverse)
				{
					if(_input[0]->numInputs()>0)
					{
						if(_input[0]->getInput(0)->isTrajectory())
						{
							TrajectoryChannel* trajC = (TrajectoryChannel*)_input[0]->getInput(0);
							float phase = trajC->getStepPhase(time,true);
							if(_input.size()<=1)
								return phase;
							return _input[1]->getVal(phase*_input[1]->duration());
						}
					}
				}
			}
			break;
		
		case channel_multiply:
			//multiplies the inputs together
			
				for(int i=0;i<_input.size();i++)
				{
					val *= _input[i]->getVal(time);
				}

				return val;

			break;
	}
	phout<<"bad call to getVal\n";
	return val;

}


int Channel::getParameterID()
{
	return pInt(channel_parameter_index);
}

void Channel::range( float mn,float rst,float mx )
{
	setP(channel_range,mn,0);
	setP(channel_range,rst,1);
	setP(channel_range,mx,2);
	applyParameters();
}


void Channel::setControlCurve( Channel* cntCurve )
{
	phout<<"fix makeControlChannel()\n";
// 	
// 	for(int i=0;i<_controlCurves.size();i++)
// 	{
// 		if(_controlCurves[i])
// 		{
// 			_controlCurves[i]->activate();
// 			_controlCurves[i]->control_mode = channel_trajectory;
// 		}
// 	}
// 	_controlCurves.size(0);
// 	if(cntCurve->getControlMode() == channel_trajectory)
// 		cntCurve->setChannelMode(channel_additive);
// 	_controlCurves.push(cntCurve);
// 	_controlCurves[0]->_object = _object;
// 	_controlCurves[0]->deactivate();
	
}
void Channel::setControlCurve( Channel* stanceCurve,Channel* leftCurve,Channel* rightCurve )
{
	phout<<"fix setControlCurve( Channel* stanceCurve,Channel* leftCurve,Channel* rightCurve )\n";
// 	_node->setMode(channel_switch);
// 	_node
// 	for(int i=0;i<_controlCurves.size();i++)
// 	{
// 		if(_controlCurves[i])
// 		{
// 			_controlCurves[i]->activate();
// 			_controlCurves[i]->setControlMode(channel_trajectory);
// 		}
// 	}
// 	_controlCurves.size(0);
// 
// 	_controlCurves.push(stanceCurve);
// 	_controlCurves.push(leftCurve);
// 	_controlCurves.push(rightCurve);
// 	if(leftCurve)
// 	{
// 		leftCurve->_object = _object;
// 		leftCurve->deactivate();
// 		if(leftCurve->getControlMode() == channel_trajectory)
// 			leftCurve->setChannelMode(channel_additive);
// 	}
// 	if(rightCurve)
// 	{
// 		rightCurve->_object = _object;
// 		rightCurve->deactivate();
// 		if(rightCurve->getControlMode() == channel_trajectory)
// 			rightCurve->setChannelMode(channel_additive);
// 	}
	
}


GsString Channel::getParameterName()
{
	GsString s;
	if(_object)
	{
		s = _object->getParameter(getParameterID())->name;
	}
	else
	{
		s= "none";
	}
	return s;
}

void Channel::loops( bool lps )
{
	_loops = lps;
}

chanel_modes Channel::getChannelMode()
{
	return channel_mode;
}

int Channel::numInputs()
{
	return _input.size();
}

Channel* Channel::getInput( int it )
{
	return _input.get(it);
}
void Channel::clearInputs()
{
	_input.size(0);
}
void Channel::pushInput(Channel* c )
{
	for(int i=0;i<_input.size();i++)
	{
		if(c==_input[i])
		{
			phout<<"found duplicate input "<<c->name();
			return;
		}
	}

	_input.push(c);

}

bool Channel::controlsObject()
{
	if(getParameter(channel_parameter_name)->save)
	{
		return true;
	}
	return false;
}

void Channel::setControlVal( float v )
{
	setP(channel_node_val,v);
	if(v!=0)
		getParameter(channel_node_val)->save = true;
}
void Channel::removeInput(int i)
{
	if(i<numInputs())
	{
		_input[i] = 0;
		setP(channel_control_list,"NULL",i);	
	}
}
void Channel::removeInput( Channel* ch )
{
	for(int i=0;i<_input.size();i++)
	{
		if(_input[i]==ch)
		{	
			_input[i] = 0;
			setP(channel_control_list,"NULL",i);	
			return;
		}
	}
}


channel_dof_types Channel::getDofType()
{
	return stringToChannelType(pString(channel_dof));
}

void Channel::updateInputList()
{
	getStringParameter(channel_control_list)->init();
	if(_input.size()>0)
		getStringParameter(channel_control_list)->save = true;
	for (int i=0;i<_input.size();i++)
	{
		if(_input[i])
			getStringParameter(channel_control_list)->add(_input[i]->name());
		else
			getStringParameter(channel_control_list)->add("NULL");
	}
}

void Channel::setChannelMode( chanel_modes mode )
{
	channel_mode = mode;
	getParameter(channel_feedback_mode)->save = false;
	switch(mode)
	{
	case channel_additive:
		setP(channel_control_mode,"ADDITIVE");
		getParameter(channel_control_mode)->save = true;
		break;
	case channel_trajectory:
		setP(channel_control_mode,"DIRECT");
		getParameter(channel_control_mode)->save = false;
		break;
	case channel_multiply:
		setP(channel_control_mode,"SCALE");
		getParameter(channel_control_mode)->save = true;
		break;
	case channel_modulate:
		setP(channel_control_mode,"MODULATE");
		getParameter(channel_control_mode)->save = true;
		_input.size(3);
		_input[0] = 0;
		_input[1] = 0;
		_input[2] = 0;
		break;
	case channel_inverse:
		setP(channel_control_mode,"INVERSE");
		getParameter(channel_control_mode)->save = true;
		break;
	case channel_feedback:
		setP(channel_control_mode,"FEEDBACK");
		getParameter(channel_control_mode)->save = true;
		getParameter(channel_feedback_mode)->save = true;
		setP(channel_feedback_mode,true);
		break;
	}
}

GsVec2 Channel::getNodePosition()
{
	return GsVec2(pVec(channel_node_position).x,pVec(channel_node_position).y);
}

void Channel::setInput( Channel* c, int i )
{
	if(_input.size()<=i)
		_input.size(i+1);

	//phout<<"inserting "<< c->name() <<" as input at index "<<i<<gsnl;
	_input[i] = c;
}

bool Channel::constantValue()
{
	return pBool(channel_val_constant);
}

bool Channel::hasInputs()
{
	for (int i=0;i<numInputs();i++)
	{
		if(getInput(i))
			return true;
	}
	return false;
}
bool Channel::hasInput(Channel* ch)
{
	for (int i=0;i<numInputs();i++)
	{
		if(getInput(i) == ch)
			return true;
	}
	return false;
}




VecTrajectory::VecTrajectory( const GsString& name,int id,GsVec orgin ,GsQuat frm)
{
	_parmId = id;
	objectName = name;
	origin = orgin;
	frame = frm;
	_curve = new Curve;
	_curve->cCol.set(255,0,0);
	_curve->curveMode = Curve::LINEAR;
	chs.size(3); chs[0]=0;chs[1]=0;chs[2]=0;
	valid = false;
}
void VecTrajectory::setObject(const GsString& name, GsVec orgn ,GsQuat frm)
{
	objectName = name;
	origin = orgn;
	frame = frm;
}
VecTrajectory::VecTrajectory( Channel* x,Channel* y,Channel* z )
{
	_curve = new Curve;
	_curve->cCol.set(255,0,0);
	_curve->curveMode = Curve::LINEAR;
	chs.push(x);chs.push(y);chs.push(z);
	valid = true;
	objectName = x->getObject()->name();
}
#include "util_motion.h"
void VecTrajectory::init()
{
	chs.size(3);
	chs[TX]=0;
	chs[TY]=0;
	chs[TZ]=0;
}
void VecTrajectory::setFromMotion( Motion* m)
{
	if(!m)
		return;

	int cnt = 0;


	for (int i=0;i<m->numChannels();i++)
	{
		if(m->getChannel(i)->getObject())
		{
			Channel* ch = m->getChannel(i);
			if(ch->pString(channel_object_name)==objectName && _parmId == ch->getParameterID())
			{
				if(ch->getChannelType()==CH_VEC_X)
				{
					//if(chs[TX]==0)
					{
						chs[TX] = ch; 
						cnt++;
					}
					//else phout<<"error x\n";
				} 
				else if(ch->getChannelType()==CH_VEC_Y)
				{
					//if(chs[TY]==0)
					{
						chs[TY] = ch; 
						cnt++;
					}
					//else phout<<"error y\n";
				} 
				else if(ch->getChannelType()==CH_VEC_Z)
				{
					//if(chs[TZ]==0)
					{
						chs[TZ] = ch; 
						cnt++;
					}
					//else phout<<"error z\n";
				} 
				else
				{
					phout<<"you sure you dont want "<<ch->name()<<gsnl;
				}
			}
		}
	}
	
}

void VecTrajectory::update()
{
	_curve->clear();
	
	int cnt = 0;
	
	float duration = 0.0f;
	for (int i=0;i<3;i++)
	{
		if(chs[i]!=0)
		{
			cnt++;
			float dur = chs[i]->duration();
			//phout<<"curve "<<i<<" has "<<numPoints<<" poiints\n";
			if (duration<dur)
			{
				duration = dur;
			}
		}
	}

	if(duration==0.0f)
		return;

	if (cnt==0)
	{
		_curve->_line->visible(false);
		valid = false;
		return;
	}
	else
	{
		valid = true;
		_curve->_line->visible(true);
	}

	GsVec vc;
	float dt = 1.0f/30.0f;
	for (float t=0;t<duration;t+=dt)
	{
		
		if(chs[TX]!=0)
		{
			vc.x = chs[TX]->getVal(t);

		}
		if(chs[TY]!=0)
		{
			vc.y = chs[TY]->getVal(t);

		}
		if(chs[TZ]!=0)
		{
			vc.z = chs[TZ]->getVal(t);
		}
		_curve->addPoint(origin+frame.apply(vc));
	}
	_curve->update();
}

VecTrajectory::~VecTrajectory()
{
	if(_curve)delete _curve;
	chs.size(0);
}

void VecTrajectory::setOrigin( GsVec og ,GsQuat frm)
{
	origin = og;
	frame = frm;
}
