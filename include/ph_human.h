
#pragma once
/*
The PhysicalHuman class contains an array of PhysicalJoints and an array of Modules. Each PhysicalJoint except for the root is constrained to its parent by a hinge,universal or ball joint and it also contains
a rigid body. The modules operate on the PhysicalHuman in many different ways
*/
#include "common.h"
#include "util.h"
#include <ode/ode.h>
#include "ode_object.h"
#include "util_serializable.h"
#include "ph_parameters.h"

#define USE_IK
#include <direct.h>

enum _module_types
{
	COM_MODULE,
	ROOT_MODULE,
	GRAVITY_MODULE,
	IK_MODULE,
	PUPPET_MODULE,
	VIRTUAL_MODULE,
	REFERENCE_MODULE,
	BALANCE_MODULE,
	CONTACT_MODULE,
};

class ODESpring;
class KnSkeleton;
class ODEWorld;
class PhysicalJoint;
class ODESphere;
class ODEBox;
class Curve;
class Ball;
class Model;
class RootModule;
class COMModule;
class ReferenceModule;
class WalkController;
class ContactModule;
class GravityModule;
class PuppetModule;
class Module;
class VirtualModule;
class BalanceModule;
class IKModule;
class HumanManager;
class HumanFileManager;
class HumanMotionManager;
class HumanStateManager;

class PhysicalHuman :  public Serializable
{
public:
	PhysicalHuman(HumanManager* mngr, const char* file);
	~PhysicalHuman(); 
	/* creates the human based on the configuration file. deletes all joints and modules and creates new ones */
	void loadConfiguration(const GsString& file);
	/*get access to a module using the _module_types enum defined above*/
	Module* getModule(int i);
	/*access a module by name. less efficient since it needs to search for it*/
	Module* getModule(const GsString& cont);
	/*Serializable virtual function. takes the parameters from the Serializable class and applies 
	them to the character if they are more than just referenced*/
	void applyParameters();
	/*called for each simulation step*/
	void update();
	/*should be called only on the animationStep*/
	void redraw();
	void setControllerVis(bool b){_module_group->visible(b); setP(human_show_controllers,b);applyParameters();}
	/*returns the state of the character not including the joint states*/
	GsString stateString();
	/*returns a string with all the joint states in it*/
	GsString jointStateString();
	/*returns a string with all the modules definitions*/
	GsString moduleString();
	/*returns a string with all the joint definitions*/
	GsString jointString();
	/*the name of this character*/
	GsString characterName();
	
	HumanMotionManager* getMotionManager(){return _motion_manager;}
	/*get the StateManager*/
	HumanStateManager* getStateManager(){return _state_manager;}

private:
	/*The group of joints that define this character*/
	GsArray<PhysicalJoint*> _joints;
	/*The group of modules that operate on this character*/
	GsArray<Module*> _modules;
	/*a set of forces that are applied to this character such as a push*/
	GsArray<body_force> _external_forces;

	/*reference to the HumanManager*/
	HumanManager* _manager;
	HumanStateManager* _state_manager;
	HumanMotionManager* _motion_manager;

	/*convenience references to some important joints. set in loadConfiguration*/
	PhysicalJoint* _left_foot_joint;
	PhysicalJoint* _right_foot_joint;
	PhysicalJoint* _left_toe_joint;
	PhysicalJoint* _right_toe_joint;
	PhysicalJoint* _root_joint;
	PhysicalJoint* _shoulder_joint;
	PhysicalJoint* _left_hand_joint;
	PhysicalJoint* _right_hand_joint;


	/*holds the geometry for the humanoid and all the modules*/
	SnGroup* _grp;
	/*a line between the IK root  and the IK stance foot defined by getStanceState()*/
	Line* _motionSupportLine;
	/*the vector between the IK root and the IK stancefoot */
	GsVec _motionSupportVec;
	/*update _motionSupportVec/Line*/
	GsVec getSupportVecFromIK();

	/*should only be called once during loadConfiguration()*/
	float computeMass();
	/*should only be called once for each simulation step. use getVelocity to access the velocity*/
	void computeVelocity();
	/*transfers joint angles from _joints to the KnSkeleton. joints must have the same name*/
	void applyToSkel(KnSkeleton* sk);

	/*scene node that contains collision visualization geometry for the joints*/
	SnGroup* _joint_group;
	/*scene node that contains collision visualization geometry for the modules*/
	SnGroup* _module_group;
	/*scene node that contains random other visualization stuff*/
	SnGroup* _visualization_group;
	/*geometry to visualize the current and desired heading of the character*/
	Arrow* _heading_arrow;
	Arrow* _desired_heading_arrow;
	/*a more visually appealing version of the character. should have same dimmensions and joint names
	as the reference_module skeleton*/
	KnSkeleton* _vis_skel;
	KnScene* _vis_scene;
	/*the current joint that is being edited*/
	PhysicalJoint* _edit_joint;

	/*a convenience object that contains default values for all the joints in the skeleton so the 
	configuration files can be smaller. if a joint has the same parameter defined in the config file 
	it will overwrite the
	one defined in defaultJoint*/
	Serializable* _default_joint;
	/*reference to the ODE simulation environment*/
	ODEWorld* _world;
public:
	/*access the HumanManager*/
	HumanManager* getManager();
	/*access the ODE simulation environment*/
	ODEWorld* getWorld(){return _world;}
	/*access the root node of the visualization for this character.. contains all the joints and modules*/
	SnGroup* getGroup(){return _grp;}
	/*used to focus camera on collision group if it is visible*/
	SnGroup* getCollisionGroup(){return _joint_group;}
	/*used to focus camera on visualization group if it is visible*/
	KnScene* getVisSkelScene(){return _vis_scene;}
	
	/*set the joint to be edited. for adding forces to or editing with the UI*/
	void setEditJoint( PhysicalJoint* j ){_edit_joint = j;}
	PhysicalJoint* getEditJoint(){return _edit_joint;}
	/*The COM of the stance foot defined by getStanceState()*/
	GsVec getStancePosition();

    KnSkeleton* getVisSkel(){return _vis_skel;}
	/*get an all the Serializable objects associated with this character.. mainly the joints,their 
	rigid bodies and the modules*/
	GsArray<Serializable*> getSerializables();
	/*access the reference skeleton defined in reference_module()*/
	KnSkeleton* skref();
	/*the number of modules for this character*/
	int numModules(){return _modules.size();}
	/*The number of physical joints*/
	int numJoints(){return _joints.size();}
	/*true if the left foot is the stance foot false if the right foot is the stance foot*/
	bool leftStance(){return pBool(human_stance_state);}
	/*convert leftStance to enumerator human_stance defined above*/
	human_stance getStanceState(){ return leftStance() ?   STANCE_LEFT : STANCE_RIGHT; }
	void setStanceState(human_stance sstate){(sstate==STANCE_LEFT) ? setP(human_stance_state, true): setP(human_stance_state, false);}
	/*toggle the stance foot*/
	void flipStanceState(){pBool(human_stance_state)? setStanceState(STANCE_RIGHT):setStanceState(STANCE_LEFT);}

	/*if this is true than both feet will be considered for maintaing balance*/
	bool doubleSupported(){return pBool(human_double_support);}
	/*if double supported returns true if the COM projection to the floor is within the contact polygon
	of the feet. otherwise returns true if the COM is within the stance foot contact poly*/
	bool staticBalanced();
	/*to tell if the character is standing based on the height of the hips.. probably needs to be more generic*/
	bool isStanding();
	/*if doubleSupported returns the stanceSwingRatio defined in the contact_module*/
	float getStanceSwingRatio();
	/*access a joint by its name*/
	PhysicalJoint* joint(GsString name);
	/*access a joint by its index*/
	PhysicalJoint* joint(int idx);
	/*access some important joints*/
	PhysicalJoint* leftToe();
	PhysicalJoint* rightToe();
	PhysicalJoint* leftFoot();
	PhysicalJoint* rightFoot();
	PhysicalJoint* stanceFoot();
	PhysicalJoint* swingFoot();
	PhysicalJoint* swingKnee();
	PhysicalJoint* stanceKnee();
	PhysicalJoint* swingHip();
	PhysicalJoint* stanceHip();
	PhysicalJoint* root();
	PhysicalJoint* hips();
	PhysicalJoint* shoulder();
	PhysicalJoint* leftHand();
	PhysicalJoint* rightHand();
	
	/*get the center of pressure calculated in the contact_module*/
	GsVec getCOP();
	/*get the center of mass calculated in the com_module*/
	GsVec getCOM();
	/*the center of mass with the vertical component set to zero*/
	GsVec getCOMProjection();
	/*get the center of mass velocity calculated in the com_module*/
	GsVec getCOMVelocity();
	/*This is where the character wants it's center of mass to be*/
	GsVec getDesiredCOM();
	/*get the heading orientation of the character. this is calculated by taking the orientation of the 
	root PhysicalJoint and aligning it with the world up vector*/
	GsQuat getHeading();
	/*right now this is true at the beginning of the simulation then set to false as soon as the 
	characters feet touch the ground*/
	bool flying();
	/*the summed mass of all the rigid bodies*/
	float getMass(){return pFloat(human_mass);}
	/*a global gain multiplier for proportional control*/
	float getGainPMult();
	/*a global gain multiplier for derivative control*/
	float getGainDMult();
	/*a gain multiplier that makes the swing side stiffer or weaker*/
	float getGainMultSwing();
	/*a gain multiplier that makes the stance side stiffer or weaker*/
	float getGainMultStance();

	/*manages a set of PD servos that drive the PhysicalJoints of the human to setpoint orientation 
	defined by a KnSkeleton _skref*/
	ReferenceModule* reference_module(){return (ReferenceModule*)getModule(REFERENCE_MODULE);}
	/*calculates the characters COM and COM velocity at each point and determines what the desired 
	COM and COM velocity are*/
	COMModule*		 com_module(){return (COMModule*)getModule(COM_MODULE);}
	/*determines the contact state of the character*/
	ContactModule*	 contact_module(){return (ContactModule*)getModule(CONTACT_MODULE);}
	/*attempts to control the unactuated root joint of the character with a virtual PD servo. the 
	desired virtual torques desired are then distributed to the contact legs*/
	RootModule*		 root_module(){return (RootModule*)getModule(ROOT_MODULE);}
	/*tries to counteract the effect of gravity to allow more accurate tracking independent of the 
	character orientation*/
	GravityModule*	 gravity_module(){return (GravityModule*)getModule(GRAVITY_MODULE);}
	/*Uses the graphsim IK class to control the reference skeleton in the ReferenceModule*/
	IKModule*		 ik_module(){return (IKModule*)getModule(IK_MODULE);}
	/*controls the character with external forces by attaching springs between the manipulators and 
	the PhysicalJoint associated with it*/
	PuppetModule*	 puppet_module(){return (PuppetModule*)getModule(PUPPET_MODULE);}
	/*similar to the PuppetModule but the force that is calculated from the springs is not applied 
	directly to the rigid body but instead
	is converted to achievable joint torques using the Jacobian of the specified joint chain*/
	VirtualModule*	 virtual_module(){return (VirtualModule*)getModule(VIRTUAL_MODULE);}
	/*responsible for maintaining balance of the character through a variety of ways: Virtual force 
	on the COM, Linear Joint offsets of the contact legs and simbicon control laws for walking*/
	BalanceModule*	 balance_module(){return (BalanceModule*)getModule(BALANCE_MODULE);}

	/*used to allow the ui to access the gain multipliers*/
	void setGainMult( float p, float d ,float swp,float swd );

	/*reset default values of the character*/
	void reset();
	/*adds an external force to the current _edit_joint*/
	void addForce( GsVec dir, float dur ,ODEObject* body);
	/*get the desired global heading*/
	GsQuat getDesiredHeading();
	/*returns true if the current stancefoot is in contact with the ground*/
	bool stanceFootContact();
	float getHeadingAngle();
	float totalTorque();
	GsVec stancePoint();
	float floorHeight();
	GsVec getContactProjectionCenter();
	bool nonFootContact();
	bool feetPlanted();
	void makeJoints();
	void init( );
};


human_stance toggleStance( human_stance s );


//from simbicon
	/**
		This method is used to compute the torques that mimic the effect of applying a force on
		a rigid body, at some point. It works best if the end joint is connected to something that
		is grounded, otherwise (I think) this is just an approximation.

		This function works by making use of the formula:

		t = J' * f, where J' is dp/dq, where p is the position where the force is applied, q is
		'sorta' the relative orientation between links. It makes the connection between the velocity
		of the point p and the relative angular velocities at each joint. Here's an example of how to compute it.

		Assume: p = pBase + R1 * v1 + R2 * v2, where R1 is the matrix from link 1 to whatever pBase is specified in,
			and R2 is the rotation matrix from link 2 to whatever pBase is specified in, v1 is the point from link 1's
			origin to link 2's origin (in link 1 coordinates), and v2 is the vector from origin of link 2 to p 
			(in link 2 coordinates).

			dp/dt = d(R1 * v1)/dt + d(R2 * v2)/dt = d R1/dt * v1 + d R2/dt * v2, and dR/dt = wx * R, where wx is
			the cross product matrix associated with the angular velocity w
			so dp/dt = w1x * R1 * v1 + w2x * R2 * v2, and w2 = w1 + wRel
			
			= [-(R1*v1 + R2*v2)x   -(R2*v2)x ] [w1   wRel]', so the first matrix is the Jacobian.

			for a larger chain, we get:
				dp/dt = [-(R1*v1 + R2*v2 + ... + Rn*vn)x; -(R2*v2 + R3*v3 + ... + Rn*vn)x; ...; -(Rn*vn)x ] [w1; w2; ...; wn]'
				or
				dp/dt = [-p1x; -p2x; ...; -pnx ] [w1; w2; ...; wn]'
				where pi is the vector from the location of the ith joint to p;

				and all omegas are relative to the parent. The ith cross product vector in the jacobian is a vector from the
				location of the ith joint to the world location of the point where the force is applied.

			The first entry is the cross product matrix of the vector (in pBase coordinates) from the
			origin of link 1 to p, and the second entry is the vector (in pBase coordinates) from
			the origin of link 2 to p (and therein lies the general way of writing this).
	*/
void computeJointTorquesEquivalentToForce(PhysicalJoint* start, GsVec pGlobal, GsVec fGlobal, PhysicalJoint* end);


