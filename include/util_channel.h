
#pragma once


#include "common.h"
#include "util_serializable.h"
#include "util_trajectory.h"

class Channel;

typedef enum channel_dof_types_
{
	CH_VEC_X,
	CH_VEC_Y,
	CH_VEC_Z,
	CH_ROT_X,
	CH_ROT_Y,
	CH_ROT_Z,
	CH_FLOAT,
	CH_BOOL,
	CH_FREE
}channel_dof_types;

enum chanel_modes
{
	channel_trajectory,  //the value is based on a spline trajectory must be a TrajectoryChannel*
	channel_additive,    //adds the channels
	channel_multiply,	 //multiplies
	channel_modulate,	 //the pulses of the first input decide timing of second input
	channel_switch,      //the first input decides what of the next to inputs to use
	channel_inverse,		 //needs a bool in and out
	channel_feedback
};

enum channel_parms
{
	channel_active,
	channel_node_val,		//some nodes may have a single value instead of a curve
	channel_val_constant,   //the node value may be constant throughout controller;
	channel_parameter_name, //ik_manip_position,joint_rot_offset..etc
	channel_object_name,    //IKModule LeftHand .. etc  this can be set directly or indirectly through channel_genneral_name (ex SwingFoot --> LeftFoot)
	channel_object_type,    //Controller,Joint,IKManip..etc
	channel_general_name,   //SwingHand StanceUpLeg.. etc  if it is not a mirror joint leave blank
	channel_dof,		    //string : FLOAT VEC_X VEC_Y VEC_Z
	channel_range,
	channel_phase_repetitions,
	channel_phase_flip,
	channel_float_index,
	channel_parameter_index,
	channel_stance,
	channel_control_mode,
	channel_control_list, //this list of channels that are inputs to the node
	channel_feedback_mode,
	channel_node_position,//this is just for drawing the nodes
	channel_last
};
enum trajectory_channel_parms
{
	trajectory_channel_type = channel_last,
	trajectory_channel_p_t, 
	trajectory_channel_p_y,
	trajectory_channel_tng, //weight of each point going out.. the weight of the next tangent in is 1-curve_tng[i-1]
	trajectory_channel_sample,
	trajectory_channel_loop_continuity,
	trajectory_channel_loop_flip,
	trajectory_channel_sample_indexes,
	trajectory_channel_sample_values,
	trajectory_channel_curve_color,
	trajectory_channel_lock_reconfigure,
	trajectory_channel_curve_width,
	trajectory_channel_point_color,
	trajectory_channel_point_size,
	trajectory_channel_show_tangents,
	trajectory_channel_show_control_points,
};
class Channel:public Serializable
{
	Serializable* _object;
	rotation_type _rotation_order;
	GsArray<Channel*> _input;
	bool _node_vis;
	bool _curve_vis;
protected:
	chanel_modes channel_mode;
	bool _flip;
	bool _loops;
	float _lastTime;
	GsVec2 _currentValPt;
	float _duration;
	bool _isTrajectory;

	channel_dof_types _channel_type;
public:
	GsVec2 getLastPoint(){return _currentValPt;}
	//for drawing in the node viewer

	bool nodeVisible(){return _node_vis;}
	bool curveVisible(){return _curve_vis;}
	void hideCurve(){_curve_vis = false;}
	void showCurve(){_curve_vis = true;}
	void hideNode(){_node_vis = false;}
	void showNode(){_node_vis = true;}

	bool isTrajectory(){return _isTrajectory;}
	void flip( bool f )
	{
		_flip = f;
	}
	//any new node must inherit this given a time it outputs a value
	virtual	float getVal( float time );
	virtual float duration(){return _duration;}
	virtual void duration(float d){_duration = d;}
	virtual void randomize(){
		//shouldnt radomize all contorl vals
		//setControlVal(gs_random(cmin(),cmax()));
	}


	chanel_modes getChannelMode();
	void setChannelMode(chanel_modes mode);

	void init();
	
	Channel(const GsString& name,channel_dof_types channelType);
	Channel( Serializable* sav);
	Channel( const GsString& file,const GsString& name);
	Channel( Channel* ch);
	Channel(Serializable* object,int parameter_id,channel_dof_types channelType,chanel_modes mode,int param_idx =-1);
	~Channel();

	

	
	//this returns the value of the target channel for example the IKLeftFoot.x or a parameter value StanceSwingRatio
	float currentChannelVal();
	//this will set the value of the target channel
	float setCurrentChannelVal(float a);

	

	void setTime(float t);

	
	void setObject(Serializable* obj);

	
	void range(float mn,float rst,float mx);
	int creps(){return pInt(channel_phase_repetitions);}
	bool flips(){return pBool(channel_phase_flip);}
	channel_dof_types getChannelType(){return (channel_dof_types)(_channel_type);}


	Serializable* getObject();



	float cmin(){return pFloat(channel_range,0);}
	float crest(){return pFloat(channel_range,1);}
	float cmax(){return pFloat(channel_range,2);}

	bool active(){return pBool(channel_active);}
	void activate(){setP(channel_active,true);}
	void deactivate(){setP(channel_active,false);}
	GsString getParameterName();
	
	void setParameter( Serializable* sav, int param ,channel_dof_types type,int arrId = -1);
	int getParameterID();

	void setControlCurve( Channel* cntCurve );
	void setControlCurve( Channel* stanceCurve,Channel* leftCurve,Channel* rightCurve );
	void loops( bool lps );
	int numInputs();
	Channel* getInput( int it );

	bool controlsObject();
	void clearInputs();
	void pushInput(Channel* c );
	void setControlVal( float v );
	float getControlVal();
	void removeInput( Channel* ch );
	void removeInput(int i);
	channel_dof_types getDofType();
	void updateInputList();
	GsVec2 getNodePosition();
	void setInput( Channel* c, int i );
	bool constantValue();
	bool hasInputs();
	bool hasInput(Channel* ch);
};



static channel_dof_types stringToChannelType(const GsString& n)
{
	channel_dof_types type = CH_FREE;
	if(n=="BOOL")
	{
		type = CH_BOOL;
	}
	else if(n=="FLOAT")
	{
		type = CH_FLOAT;
	}
	else if(n=="VEC_X")
	{
		type = CH_VEC_X;
	}
	else if(n=="VEC_Y")
	{
		type = CH_VEC_Y;
	}
	else if(n=="VEC_Z")
	{
		type = CH_VEC_Z;
	}
	else if(n=="ROT_X")
	{
		type = CH_ROT_X;
	}
	else if(n=="ROT_Y")
	{
		type = CH_ROT_Y;
	}
	else if(n=="ROT_Z")
	{
		type = CH_ROT_Z;
	}

	return type;
}

static GsString channelTypeToString(channel_dof_types d)
{

	switch(d)
	{
	case CH_FLOAT: return "FLOAT"; break;
	case CH_VEC_X: return "VEC_X"; break;
	case CH_VEC_Y: return "VEC_Y"; break;
	case CH_VEC_Z: return "VEC_Z"; break;
	case CH_ROT_X: return "ROT_X"; break;
	case CH_ROT_Y: return "ROT_Y"; break;
	case CH_ROT_Z: return "ROT_Z"; break;
	case CH_BOOL: return "BOOL"; break;
	case CH_FREE: return "FREE";break;
	}
	phout<<"channelTypeToString() got bad channel_dof_types";
	return "FLOAT";

}