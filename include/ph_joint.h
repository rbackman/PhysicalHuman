#pragma once

#include "ode_box.h"
#include "common.h"
#include "ph_human.h"
#include "util_serializable.h"

class PID;


enum _joint_types //hinge or universal joint
{
	HINGE_X,
	HINGE_Y,
	HINGE_Z,
	UNI_XY,
	UNI_XZ,
	UNI_YZ,
	BALL,
	ROOT_JOINT
};

class PhysicalJoint : public Serializable
{
private:
	/*load this joint with a 1 dof hinge joint*/
	void createHinge();
	/*load this joint with a 2 dof universal joint*/
	void createUniversal();
	/*load this joint with a 3dof ball joint*/
	void createBall();
	/*the type and dof of joint specified in _joint_types*/
	int _joint_type;
	/*keep track of last orientation to estimate desired velocity*/
	GsQuat _last_desired_orientation;
	/*access to singelton HumanManager*/
	HumanManager* _manager;

	/*the desired torque that is accumulated from the modules and applied to the joint on each frame*/
	GsVec _torque;
	/*the last desired torque that is accumulated from the modules and applied to the joint on each frame*/
	GsVec _last_torque;
	/*a way to visualize the setpoint of this joint. set pBool(joint_draw_goal_box,true) to make visible*/
	Box* _goal_vis;
	/*a way to visualize the current torque of this joint.set pBool(joint_draw_torque,true) to make visible*/
	Line* _torque_line;

	PhysicalHuman* _human;
	ODEWorld* _world;
	ODEBox* _body;
	Ball* _anchor_ball; //to see if there is a difference between anchor points
	PhysicalJoint* _parent;

	dJointID _jid;
	dJointID m_jid; //for ball and socket a seperate motor;
	SnGroup* _grp;

public:
	GsArray<PhysicalJoint*> children;

	/*for creating a joint from a file*/
	PhysicalJoint(PhysicalHuman* hm,GsString name,GsString file);
	/*for manually creating a joint.. may not be up to date*/
	PhysicalJoint(PhysicalHuman* hm,GsString jName,GsVec jointPos, GsVec boxPos, GsVec boxDim,  PhysicalJoint* prnt);
	~PhysicalJoint(void);

	/*state information for joint position,velocity,orientation and angular velocity of the rigid body associated with this joint*/
	GsString stateString();
	/*set the joint position,velocity,orientation and angular velocity from a text file*/
	void loadStateFromFile(GsString file);
	/*convenience method to print the joint type*/
	GsString jointTypeName();
	/*Serializable virtual function*/
	void applyParameters();
	/*called once every simulation step. limits and applies the torque*/
	void update();
	/*remove the desired torques for this joint*/
	void clearTorque();
	/*apply a torque to this joint in global coordinates*/
	void addGlobalTorque(GsVec t){_torque+=t;  }
	/*apply a torque to this joint in local coordinates*/
	void addLocalTorque(GsVec t){_torque += getGlobalOrientation().apply(t);  }
	/*get the current torque of this joint in global coordinates*/
	GsVec getTorque(){return _torque;}
	GsVec getLastTorque(){return _last_torque;}
	/*the global position of the joint anchor point*/
	GsVec getJointPosition();
	/*the global position of the center of mass of the joint rigid body*/
	GsVec getCOMPosition();
	/*access the rigid body associated with this joint. right now they are all ODEBoxes*/
	ODEBox* getBody(){return _body;}
	/*get the current orientation of the rigid body*/
	GsQuat getGlobalOrientation(){return _body->getOrientation();}
	/*get the orientation of the body with respect to the parent joint*/
	GsQuat getRelativeOrientation();
	/*get the current rotational velocity of the rigid body*/
	GsVec getRotationalVelocity(){return _body->getRotationalVelocity();}
	/*get the current rotational velocity of this joint discounting the parents rotational velocity*/
	GsVec getRelativeRotationalVelocity();
	/*get the linear velocity of the COM of the rigid body*/
	GsVec getVelocity(){return _body->getVelocity();}
	/*get the global position of the anchor point when this joint was created.. use getJointPosition()
	to get the current anchor point location*/
	GsVec getAnchor(){return pVec(joint_anchor_point);}
	GsQuat getOriginalOrientation(){return pQuat(joint_box_orientation);}
	/*get the global position of the rigid body when this joint was created.. use getCOMPosition()
	to get the current box location*/
	GsVec getOriginalBoxPosition(){return getAnchor() + pVec(joint_box_offset);}
	/*get the offset of the box relative to the anchor point at creation*/
	GsVec getBoxOffset(){return pVec(joint_box_offset);}
	/*the rotation order of the desired orientation of this joint for converting from and to euler angles*/
	rotation_type rotationOrder();
	/*get the desired rotational velocity based on the change from the last desired orientation and 
	the current. updated on each animationStep*/
	GsVec getDesiredRotationalVelocity();
	/*gets the desired orientation from the KnJoint being tracked.. will be a quat in character frame
	coordinates or in local coordinates depending on the value of pBool(joint_char_frame)*/
	GsQuat getDesiredOrientation(KnJoint* knJoint);
	/*set the gain multiplier for this joint. primarily used to set the overall stiffness of the character*/
	void setGaingMult( float gainPMult, float gainDMult );
	/*get the bounding box dimensions of the rigid body*/
	GsVec getDimension();
	/*get the color of the joint collision geometry*/
	GsColor getColor();
	/*access the simulation world for the rigid body*/
	ODEWorld* getWorld(){return _world;}
	/*get the Scene node that contains the collision geometry visualization*/
	SnGroup* getGroup(){return _grp;}
	/*get the parent of this joint. returns 0 if there is no parent*/
	PhysicalJoint* getParent(){return _parent;}
	/*access the PhysicalHuman this joint is a member of*/
	PhysicalHuman* getHuman(){return _human;}
	void makeJoint();
};
