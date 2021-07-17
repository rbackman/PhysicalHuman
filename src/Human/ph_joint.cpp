#include "ph_joint.h"
 
#include "ph_manager.h"


void PhysicalJoint::applyParameters()
{
	GsVec sze = pVec(joint_box_dim);
	((Box*)_body->getModel())->setSize(sze);
	_body->setColor(pColor(joint_color));

	//bad ideas to set position this way unless sim is stopped
	//box->setPosition(pVec(joint_anchor_point)+pVec(joint_box_offset));
	
	_anchor_ball->setPosition(pVec(joint_anchor_point));
	
	_goal_vis->visible(pBool(joint_draw_goal_box)) ;
	_torque_line->visible(pBool(joint_draw_torque));
}
rotation_type stringToRotationOrder(GsString name)
{
	if (name == "XYZ")return ROT_XYZ;
	else if (name == "XZY")return ROT_XZY;
	else if (name == "ZYX")return ROT_ZYX;
	else if (name == "ZXY")return ROT_ZXY;
	else if (name == "YXZ")return ROT_YXZ;
	else if (name == "YZX")return ROT_YZX;
	
	phout<<" no rotation order called "<<name<<gsnl;
	return ROT_XYZ;
}
int stringToJointType(GsString name)
{

	if (name == "HINGE_X")return HINGE_X;
	else if (name == "HINGE_Y")return HINGE_Y;
	else if (name == "HINGE_Z")return HINGE_Z;
	else if (name == "UNI_XY")return UNI_XY;
	else if (name == "UNI_XZ")return UNI_XZ;
	else if (name == "UNI_YZ")return UNI_YZ;
	else if (name == "BALL")return BALL;
	else if (name == "ROOT_JOINT")return ROOT_JOINT;
	phout<<" no joint called "<<name<<gsnl;
	return 0;
}

int axisToJointType(GsVec axis)
{
	int dof = (int)(axis.x+axis.y+axis.z);
	if(dof==1)
	{//its a universal joint
		if(axis.x==1)
			return HINGE_X;
		if(axis.y==1)
			return HINGE_Y;
		if(axis.z==1)
			return HINGE_Z;
	}

	if(dof==2)
	{//its a universal joint
		if(axis.x==1&&axis.y==1)
			return UNI_XY;
		if(axis.x==1&&axis.z==1)
			return UNI_XZ;
		if(axis.y==1&&axis.z==1)
			return UNI_YZ;
	}
	if(dof==3)
	{
		return BALL;
	}
	if(dof==0)
		return ROOT_JOINT;

	phout<<"could not find a joint type for the axis\n";
	return 0;
}
void PhysicalJoint::createHinge()
{
	dWorldID worldID = _world->GetWorldID();
	dBodyID body1 = _parent->getBody()->getBodyID();
	dBodyID body2 = getBody()->getBodyID();
	GsVec jointPoint = pVec(joint_anchor_point);
	

	_jid =  dJointCreateHinge(worldID, 0);
	dJointAttach(_jid, body1, body2);
	dJointSetHingeAnchor(_jid, jointPoint.x, jointPoint.y, jointPoint.z);
	
	switch(_joint_type)
	{
		case HINGE_X: 	dJointSetHingeAxis(_jid, 1, 0, 0);break;
		case HINGE_Y:	dJointSetHingeAxis(_jid, 0, 1, 0);break;
		case HINGE_Z:   dJointSetHingeAxis(_jid, 0, 0, 1);break;
		default:		dJointSetHingeAxis(_jid, 1, 0, 0); phout<<"hinge not xyz\n";break;
	}
	
	dJointSetHingeParam(_jid, dParamLoStop, GS_TORAD(pFloat(joint_limits,0)));
	dJointSetHingeParam(_jid, dParamHiStop, GS_TORAD(pFloat(joint_limits,1)));

}
void PhysicalJoint::createBall()
{
	dWorldID worldID = _world->GetWorldID();
	dBodyID body1 = _parent->getBody()->getBodyID();
	dBodyID body2 = getBody()->getBodyID();
	GsVec jointPoint = pVec(joint_anchor_point);
	
	
	_jid = dJointCreateBall(worldID, 0);
	dJointAttach(_jid, body1, body2);
	dJointSetBallAnchor(_jid, jointPoint.x, jointPoint.y, jointPoint.z);

	m_jid = dJointCreateAMotor(worldID,0);
	dJointAttach(m_jid, body1, body2);

	//dAMotorUser	 The AMotor axes and joint angle settings are entirely controlled by the user. This is the default mode.
	//dAMotorEuler	 Euler angles are automatically computed. The axis a1 is also automatically computed. 
	//The AMotor axes must be set correctly when in this mode, as described below. When this mode is initially 
	//set the current relative orientations of the bodies will correspond to all euler angles at zero.
	//dJointSetAMotorMode (m_jid,dAMotorUser);
	dJointSetAMotorMode (m_jid,dAMotorEuler);
	dJointSetAMotorNumAxes(m_jid, 3);

	//dJointSetAMotorAxis (dJointID, int anum, int rel,  dReal x, dReal y, dReal z);
	//anum = 0: The axis is anchored to the global frame.
	//anum = 0: The axis is anchored to the first body.
	//anum = 0: The axis is anchored to the second body.
	switch(rotationOrder())
	{
		case ROT_XYZ:
		{
			dJointSetAMotorAxis(m_jid, 0, 0, 1, 0, 0);
			dJointSetAMotorAxis(m_jid, 1, 0, 0, 1, 0);
			dJointSetAMotorAxis(m_jid, 2, 0, 0, 0, 1);
		}break;
			case ROT_XZY:
		{
			dJointSetAMotorAxis(m_jid, 0, 0, 1, 0, 0);
			dJointSetAMotorAxis(m_jid, 1, 0, 0, 0, 1);
			dJointSetAMotorAxis(m_jid, 2, 0, 0, 1, 0);
		}break;
			case ROT_ZYX:
		{
			dJointSetAMotorAxis(m_jid, 0, 0, 0, 0, 1);
			dJointSetAMotorAxis(m_jid, 1, 0, 0, 1, 0);
			dJointSetAMotorAxis(m_jid, 2, 0, 1, 0, 0);
		}break;
			case ROT_ZXY:
		{
			dJointSetAMotorAxis(m_jid, 0, 0, 0, 0, 1);
			dJointSetAMotorAxis(m_jid, 1, 0, 1, 0, 0);
			dJointSetAMotorAxis(m_jid, 2, 0, 0, 1, 0);
		}break;
			case ROT_YXZ:
		{
			dJointSetAMotorAxis(m_jid, 0, 0, 0, 1, 0);
			dJointSetAMotorAxis(m_jid, 1, 0, 1, 0, 0);
			dJointSetAMotorAxis(m_jid, 2, 0, 0, 0, 1);
		}break;
			case ROT_YZX:
		{
			dJointSetAMotorAxis(m_jid, 0, 0, 0, 1, 0);
			dJointSetAMotorAxis(m_jid, 1, 0, 0, 0, 1);
			dJointSetAMotorAxis(m_jid, 2, 0, 1, 0, 0);
		}break;
	}
	dJointSetAMotorParam(m_jid, dParamStopCFM, 0.1f);
	dJointSetAMotorParam(m_jid, dParamStopCFM2, 0.1f);
	dJointSetAMotorParam(m_jid, dParamStopCFM3, 0.1f);

	float FMax = 0.01f;
	float jStop = (float)GS_PI;

	dJointSetAMotorParam(m_jid, dParamLoStop, pFloat(joint_limits,0));
	dJointSetAMotorParam(m_jid, dParamHiStop,  pFloat(joint_limits,1));
	
	dJointSetAMotorParam(m_jid, dParamLoStop2,  pFloat(joint_limits,2));
	dJointSetAMotorParam(m_jid, dParamHiStop2, pFloat(joint_limits,3));

	dJointSetAMotorParam(m_jid, dParamLoStop3,  pFloat(joint_limits,4));
	dJointSetAMotorParam(m_jid, dParamHiStop3,  pFloat(joint_limits,5));

	dJointSetAMotorParam(m_jid, dParamFMax,     FMax);
	dJointSetAMotorParam(m_jid, dParamBounce,   0.0);
	dJointSetAMotorParam(m_jid, dParamVel,      0.0);

	dJointSetAMotorParam(m_jid, dParamFMax2,    FMax);
	dJointSetAMotorParam(m_jid, dParamBounce2,  0.0);
	dJointSetAMotorParam(m_jid, dParamVel2,     0.0);

}
void PhysicalJoint::createUniversal()
{

	dWorldID worldID = _world->GetWorldID();
	dBodyID body1 = _parent->getBody()->getBodyID();
	dBodyID body2 = getBody()->getBodyID();
	GsVec jointPoint = pVec(joint_anchor_point);
	
	

	_jid = dJointCreateUniversal(worldID, 0);
	dJointAttach(_jid, body1, body2);
	dJointSetUniversalAnchor(_jid, jointPoint.x, jointPoint.y, jointPoint.z);
	
	switch(_joint_type)
	{
	case UNI_XY:
		{
			dJointSetUniversalAxis1(_jid, 1, 0, 0);
			dJointSetUniversalAxis2(_jid, 0, 1, 0);
		}break;
	case UNI_XZ:
		{
			
			dJointSetUniversalAxis1(_jid, 1, 0, 0);
			dJointSetUniversalAxis2(_jid, 0, 0, 1);
		}break;
	case UNI_YZ:
		{
			
			dJointSetUniversalAxis1(_jid, 0, 1, 0);
			dJointSetUniversalAxis2(_jid, 0, 0, 1);
		}break;
	default: 
		{
			
			dJointSetUniversalAxis1(_jid, 1, 0, 0);
			dJointSetUniversalAxis2(_jid, 0, 1, 0);
		}break;
	}


	dJointSetUniversalParam(_jid, dParamLoStop, pFloat(joint_limits,0));
	dJointSetUniversalParam(_jid, dParamHiStop,  pFloat(joint_limits,1));
	dJointSetUniversalParam(_jid, dParamLoStop2, pFloat(joint_limits,2));
	dJointSetUniversalParam(_jid, dParamHiStop2,  pFloat(joint_limits,3));
}
GsString PhysicalJoint::jointTypeName()
{
	return pString(joint_type);
}
PhysicalJoint::PhysicalJoint(PhysicalHuman* hm,GsString nme, GsString file) : Serializable(nme,"PhysicalJoint")
{
#ifdef PRINT_CONSTRUCTORS
	gsout<<"PhysicalJoint(PhysicalHuman* hm,GsString nme, GsString file)"<<gsnl;
#endif
	_human = hm;
	_world = _human->getWorld();
	_manager = _human->getManager();
	_jid=0;
	_body = 0;
	m_jid = 0;


	//first set all the values to default
	loadParametersFromFile(file,"default_joint");
	
	CHECK_INT(joint_type);
	CHECK_STRING(joint_parent);
	CHECK_VEC(joint_anchor_point);
	CHECK_QUAT(joint_box_orientation);
	CHECK_VEC(joint_box_offset);
	CHECK_VEC(joint_box_dim);
	CHECK_COLOR(joint_color);
	CHECK_FLOAT(joint_gain_p);
	CHECK_FLOAT(joint_gain_d);
	CHECK_FLOAT(joint_max_t);
	CHECK_FLOAT(joint_limits);
	CHECK_BOOL(joint_use_pd);
	CHECK_BOOL(joint_grav_comp);
	CHECK_BOOL(joint_char_frame);
	CHECK_VEC(joint_vf_scale);
	CHECK_STRING(joint_rot_order);

	CHECK_VEC(joint_pd_scale);
	CHECK_BOOL(joint_draw_torque);
	CHECK_BOOL(joint_draw_goal_box);
	MAKE_TEMP_PARM(joint_desired_rot,GsQuat());
	MAKE_TEMP_PARM(joint_setpoint_rot,GsQuat());
	CHECK_STRING(joint_to_copy);
	MAKE_TEMP_PARM(joint_rot_offset,GsVec());
	MAKE_TEMP_PARM(joint_desired_rotational_velocity,GsVec());
	MAKE_TEMP_PARM(joint_gain_mult_p,1.0f);
	MAKE_TEMP_PARM(joint_gain_mult_d,1.0f);


	setP(joint_gain_d,2*sqrtf(pFloat(joint_gain_p)));
	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif  

	//I just do this to get the joint_to_copy var
	setParametersFromFile(file);
	
	//check if this joint wants to copy another
	if(pString(joint_to_copy)!="none")
	{
		//phout<<"copying from "<<pString(joint_to_copy)<<" to "<<name()<<gsnl;
		setParametersFromFile(file,pString(joint_to_copy));
		
		//phout<<toString()<<gsnl;
		//finally set any non default or copied values for this joint
		setParametersFromFile(file);
		//phout<<"anchor point "<<pVec(joint_anchor_point)<<gsnl;
		
	}

	if(pString( joint_parent)=="none")
		_parent = 0;
	else
	{
		_parent = _human->joint(pString( joint_parent));
		_parent->children.push(this);
	}
	_grp = new SnGroup();
	_grp->separator(true);
	
	//to visualize the torque applied
	_torque_line = new Line(pVec(joint_anchor_point),pVec(joint_anchor_point)+GsVec(0.1f,0.0f,0.0f));
	_torque_line->setColor(GsColor(255,0,0));
	_torque_line->visible(pBool(joint_draw_torque));
	_grp->add(_torque_line->getGrp());

	

	//this is an interesting way to visualize the reference angle
	_goal_vis = new Box( pVec(joint_anchor_point)+pVec(joint_box_offset), pVec(joint_box_dim)*0.8f);
	_goal_vis->setColor(GsColor(255,0,0));
	_goal_vis->visible(pBool(joint_draw_goal_box));
	_grp->add(_goal_vis->getGrp());
	
	//to visualize the joint position
	_anchor_ball = new Ball(pVec(joint_anchor_point),0.01f,GsColor::blue);
	_grp->add(_anchor_ball->getGrp());
	_anchor_ball->setPosition(pVec(joint_anchor_point));
	_anchor_ball->visible(true);

	
	rotation_type rotationOrder = stringToRotationOrder(pString(joint_rot_order));
	getQuatParameter(joint_desired_rot)->rotationOrder = rotationOrder;

	makeJoint();

	/*//right now all joints are just boxes.. should add meshes or capsules
	_body = new ODEBox(_world, true, pVec(joint_anchor_point)+pVec(joint_box_offset) , pVec(joint_box_dim));
	_body->setName(nme);
	_body->setColor(pColor(joint_color));
	_grp->add(_body->getModel()->getGrp());

	_joint_type = stringToJointType(pString(joint_type));
	switch(_joint_type)
	{
		case BALL:
			createBall();
		break;
		case UNI_XY:
		case UNI_XZ:
		case UNI_YZ:
		{
			createUniversal();
		}
		break;
		case HINGE_X:
		case HINGE_Y:
		case HINGE_Z:
			{
				createHinge();
			}break;
		case ROOT_JOINT:
			break;
		default: _jid = 0;
	}
	*/

}


PhysicalJoint::PhysicalJoint(PhysicalHuman* hm,GsString jName,GsVec jointPos, GsVec boxPos, GsVec boxDim,  PhysicalJoint* prnt) : Serializable(jName)
{
#ifdef PRINT_CONSTRUCTORS
	gsout<<"PhysicalJoint(PhysicalHuman* hm,GsString jName,GsVec jointPos, GsVec boxPos, GsVec boxDim,  PhysicalJoint* prnt)"<<gsnl;
#endif

	_human = hm;
	_world = _human->getWorld();
	_manager = _human->getManager();
	_parent = prnt;
	_jid=0;
	m_jid = 0;
	_body = 0;
	MAKE_PARM(joint_gain_p,0.2f);
	MAKE_PARM(joint_gain_d,0.03f);
	MAKE_PARM(joint_max_t,2.0f);
	MAKE_PARM(joint_box_offset,boxPos-jointPos);
	MAKE_PARM(joint_color,GsVec(0,0,1));
	MAKE_PARM(joint_anchor_point,jointPos);
	MAKE_PARM(joint_box_orientation,GsQuat());
	MAKE_PARM(joint_box_dim,boxDim);
	MAKE_PARM(joint_use_pd,false);
	MAKE_PARM(joint_grav_comp,false);
	MAKE_PARM(joint_char_frame,false);
	MAKE_PARM(joint_rot_offset,GsVec());
	MAKE_PARM(joint_vf_scale,GsVec());
	
	MAKE_PARM(joint_rot_order,"XYZ");
	MAKE_PARM(joint_type,"BALL");
   

	if(prnt)
		MAKE_PARM(joint_parent,prnt->name());
	else
		MAKE_PARM(joint_parent,GsString("none"));


	if(prnt==0)
		setP(joint_type, "ROOT_JOINT");

	_body = new ODEBox(_world, true, boxPos , boxDim);

	_body->setColor(pColor(joint_color));

	_anchor_ball = new Ball(jointPos,0.3f);
	_anchor_ball->setPosition(jointPos);

	
	if(prnt)
		createBall();
	else
		_jid = 0;
	
}

GsQuat PhysicalJoint::getRelativeOrientation()
{
	GsQuat parentRot;
	 if(_parent==0)
	{
	//	phout<<"no parent so quat from getRelativeOrientation() is global\n";
		return getGlobalOrientation();
	}
	else
	{
		parentRot = _parent->getGlobalOrientation();
	}
	//find the relative orientation between this joints parent and this joint
	// parentGlobalQuat * chileRelativeQuat = childGlobalQuat ==>  chileRelativeQuat = parentGlobalQuat.inverse() * childGlobalQuat
	// heading * childRelativeRot = childGlobalQuat   childRelRot = heading.inverse()*childGlobalQuat();

	return parentRot.inverse()*getGlobalOrientation();
}
void PhysicalJoint::clearTorque()
{
	_torque = GsVec(); 
}


GsVec PhysicalJoint::getCOMPosition()
{
	return _body->getPosition();
}
GsVec PhysicalJoint::getJointPosition()
{
	return _body->getPosition() + getGlobalOrientation().apply(-pVec(joint_box_offset));
}

void PhysicalJoint::loadStateFromFile(GsString file)
{
	Serializable* sav = new Serializable(name(),type());
	sav->loadParametersFromFile(file);
	_body->setParametersFromSerializable(sav);
	setParametersFromSerializable(sav);
	_body->applyParameters();
	_anchor_ball->setPosition(getJointPosition());
	delete sav;
}
void PhysicalJoint::update()
{
	
	if(_manager->animationStep())
	{
		GsQuat change = _last_desired_orientation.inverse()*pQuat(joint_desired_rot);
		change.normalize();
		if(change.angle()>0.01f)
		{
			GsVec desired = change.axis()*change.angle()*_human->pFloat(human_desired_v_scale);
			// 		if(name() == "LeftUpLeg")
			// 			phout<<"desired "<<desired<<gsnl;
			setP(joint_desired_rotational_velocity,desired);
		}
		else
			setP(joint_desired_rotational_velocity,GsVec());

		_last_desired_orientation = pQuat(joint_desired_rot);

	}



	
	
	//change torque to local coordinates to apply limits
	//parent_global_orientation * torque_local = torque_global
	//torque_local = parent_global_orientation.inverse()*torque_global
	if(_parent && !pBool(joint_char_frame))
		_torque = _parent->getGlobalOrientation().inverse().apply(_torque); 

	float maxT = pFloat(joint_max_t);

	_torque.x = bound(_torque.x,maxT);
	_torque.y = bound(_torque.y,maxT);
	_torque.z = bound(_torque.z,maxT);

	if(_manager->animationStep())
	{
		_anchor_ball->setPosition(getJointPosition());
		if(pBool(joint_draw_torque))
		{
			_torque_line->setPoints( getJointPosition(), getJointPosition() + _torque*_world->getSimStep() * 5);
		}
		if(_world->pBool(world_ode_torque_color_feedback))
		{
			if(abs(_torque.x) > maxT) 
			{
				_body->setColor(GsColor::red);
			}
			else if(abs(_torque.y) > maxT) 
			{
				_body->setColor(GsColor::green);
			}
			else if(abs(_torque.z) > maxT) 
			{
				_body->setColor(GsColor::yellow);
			}
			else
			{
				_body->setColor(pColor(joint_color));
			}
		}
		if(pBool(joint_draw_goal_box))
		{
			GsQuat pq = pQuat(joint_desired_rot);
			_goal_vis->setRotation(pq); //GsQuat(pq.axis(),pq.angle()*pf);
			_goal_vis->setPosition(getJointPosition()+pq.apply(pVec(joint_box_offset)));
		}
	}

	
	if(_torque.len()>0)
	{
		switch(_joint_type)
		{
		case BALL:
			if(pBool(joint_char_frame))
			{
				if(_parent)
				{
					if(_parent->pBool(joint_use_pd))
					{
						//GsVec pt = getRelativeOrientation().apply(torque); //change it back to global coordinates
						//parent->box->addGlobalTorque(-pt);

					}
					_parent->_body->addGlobalTorque(-_torque);
				}

				//torque = h->getHeading().apply(torque);
				_body->addGlobalTorque(_torque);
			}
			else
			{
				if(_parent)
					_torque = _parent->getGlobalOrientation().apply(_torque); //change it back to global coordinates

				_body->addGlobalTorque(_torque);

				if(_parent)
					_parent->_body->addGlobalTorque(-_torque);
			}
			break;

		case UNI_XY:
			dJointAddUniversalTorques(_jid, -_torque.x, -_torque.y);
		case UNI_XZ:
			dJointAddUniversalTorques(_jid, -_torque.x, -_torque.z);
			break;
		case UNI_YZ:
			dJointAddUniversalTorques(_jid, -_torque.y, -_torque.z);
			break;
		case HINGE_X:
			dJointAddHingeTorque(_jid,-_torque.x);
			break;
		case HINGE_Y:
			dJointAddHingeTorque(_jid,-_torque.y);
			break;
		case HINGE_Z:
			dJointAddHingeTorque(_jid,-_torque.y);
			break;
		case ROOT_JOINT:
#ifdef GLOBAL_ROOT_CONTROL
			_body->addGlobalTorque(_torque);
#endif

			break;

		}
	}

	_last_torque = _torque;
	_torque = GsVec(0,0,0);


	_body->update(_manager->animationStep());

}

PhysicalJoint::~PhysicalJoint(void)
{
	delete _body;
	if(_jid)dJointDestroy(_jid);
}

GsString PhysicalJoint::stateString()
{
	GsString f = name();
	f<<"\n{\n";
	f<<"\t"<<_body->parameterAsString(ode_position);
	f<<"\t"<<_body->parameterAsString(ode_velocity);
	f<<"\t"<<_body->parameterAsString(ode_orientation);
	f<<"\t"<<_body->parameterAsString(ode_rotational_velocity);
	f<<"\t"<<parameterAsString(joint_setpoint_rot);
	f<<"\t"<<parameterAsString(joint_desired_rotational_velocity);
	f<<"\t"<<parameterAsString(joint_char_frame);

	
	//joint_color,
	//joint_max_t,


	/*if(pBool(joint_use_pd))
	{
		f<<"\t"<<parameterAsString(joint_gain_p);
		f<<"\t"<<parameterAsString(joint_gain_d);
	}*/

	f<<"}\n";
	return f;
}

rotation_type PhysicalJoint::rotationOrder()
{
	return getQuatParameter(joint_desired_rot)->rotationOrder;
}

GsVec PhysicalJoint::getDesiredRotationalVelocity()
{
	return pVec(joint_desired_rotational_velocity);
}

GsQuat PhysicalJoint::getDesiredOrientation(KnJoint* knJoint)
{
	setP(joint_setpoint_rot,knJoint->rot()->value());

	GsQuat qDesired;
	if(pBool(joint_char_frame))
	{
		mat2quat(knJoint->gmat(),qDesired);
		qDesired = qDesired * vecToQuat(pVec(joint_rot_offset),rotation_type_to_gs_euler_order(rotationOrder()));
		qDesired = _human->getDesiredHeading()*qDesired;
		qDesired.normalize();
	}
	else
	{
		qDesired = knJoint->quat()->value();
		qDesired = qDesired * vecToQuat(pVec(joint_rot_offset),rotation_type_to_gs_euler_order(rotationOrder()));
		if(_parent)
			qDesired = _parent->getGlobalOrientation() * qDesired;

	}


	setP(joint_desired_rot,qDesired);

	return qDesired;
}

void PhysicalJoint::setGaingMult( float swingGainPMult, float swingGainDMult )
{
	setP(joint_gain_mult_p,swingGainPMult);
	setP(joint_gain_mult_d,swingGainDMult);
}

GsVec PhysicalJoint::getRelativeRotationalVelocity()
{
	if(_parent)
		return getRotationalVelocity() - getGlobalOrientation().apply(_parent->getRotationalVelocity());
	else
		return getRotationalVelocity();
}

GsVec PhysicalJoint::getDimension()
{
	return pVec(joint_box_dim);
}

GsColor PhysicalJoint::getColor()
{
	return pColor(joint_color);
}

void PhysicalJoint::makeJoint()
{
	if(_body)
	{
		_grp->remove(_body->getModel()->getGrp());
		delete _body;
	}
	
	
	_body = new ODEBox(_world, true, pVec(joint_anchor_point)+pVec(joint_box_offset) , pVec(joint_box_dim));
	
	if(pQuat(joint_box_orientation) != GsQuat::null)
	{
		gsout<<"joint "<<name()<<" will save orientation\n";
		getParameter(joint_box_orientation)->save = true;
		_body->setP(ode_orientation,pQuat(joint_box_orientation));
	}
	else
		getParameter(joint_box_orientation)->save = false;

	
	_body->setName(name());
	_body->setColor(pColor(joint_color));
	_grp->add(_body->getModel()->getGrp());
	
	if(_jid)
		dJointDestroy(_jid);
	
	_joint_type = stringToJointType(pString(joint_type));

	switch(_joint_type)
	{
	case BALL:
		createBall();
		break;
	case UNI_XY:
	case UNI_XZ:
	case UNI_YZ:
		{
			createUniversal();
		}
		break;
	case HINGE_X:
	case HINGE_Y:
	case HINGE_Z:
		{
			createHinge();
		}break;
	case ROOT_JOINT:
		break;
	default: _jid = 0;
	}

}

