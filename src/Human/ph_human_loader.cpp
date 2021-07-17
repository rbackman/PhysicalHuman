#include "common.h"

#include "ph_joint.h"
#include "ph_human.h"

#include "ph_mod_root.h"
#include "ph_mod_com.h"
#include "ph_mod_ref.h"
#include "ph_mod_ik.h"
#include "ph_mod_contact.h"
#include "ph_mod_gravity.h"
#include "ph_mod_puppet.h"
#include "ph_mod_virtual.h"
#include "ph_mod_balance.h"
#include "ph_manager.h"
#include "ph_file_manager.h"
#include "ph_motion_manager.h"
#include "ph_state_manager.h"

#define ISJOINT(num,nam) GsString::compare(GsString(_joints.get(num)->name()) ,nam)==0

void PhysicalHuman::init( )
{
	_motion_manager->init(this,_manager);
	_state_manager->init(this,_manager);
}

PhysicalHuman::PhysicalHuman(HumanManager* mngr,const char* file): Serializable("Human")
{
#ifdef PRINT_CONSTRUCTORS
	gsout<<"PhysicalHuman(HumanManager* mngr,const char* file)"<<gsnl;
#endif
	_manager = mngr;
	_world = mngr->getWorld();
	_grp = 0;
	_visualization_group=0;
	_joint_group=0;
	_module_group = 0;
	_motion_manager = new HumanMotionManager(_manager->getFiles()->getPrefFile());
	_state_manager = new HumanStateManager(_manager->getFiles()->getPrefFile());
	
	
	loadConfiguration(file);
}
void PhysicalHuman::loadConfiguration(const GsString& cnfg)
{
	if(_grp)
	{
		_grp->remove_all();
		_grp->unref();
		_joint_group->unref();
		_module_group->unref();
		_visualization_group->unref();
	}
	for(int i=0;i<_joints.size();i++)
		delete _joints.get(i);
	
	for(int i=0;i<_modules.size();i++)
		delete _modules.get(i);

	_joints.size(0);
	_modules.size(0);

	GsInput in;
	GsVars config;
	GsVars human;	

	_grp = new SnGroup(); _grp->ref(); _grp->separator(true);
	_joint_group = new SnGroup; _joint_group->ref(); _joint_group->separator(true);
	_grp->add(_joint_group);
	_module_group = new SnGroup; _module_group->ref(); _module_group->separator(true);
	_grp->add(_module_group);
	_visualization_group = new SnGroup(); _visualization_group->ref(); _visualization_group->separator(true);
	_grp->add(_visualization_group);
	_motionSupportLine = new Line(GsVec(),GsVec(0.0f,0.0f,0.1f));
	_grp->add(_motionSupportLine->getGrp());


	loadParametersFromFile( _manager->getFiles()->getConfigFile(cnfg,"human"));


	CHECK_STRING(human_joint_list);
	CHECK_STRING(human_module_list);
	CHECK_STRING(human_reference_skeleton_name);
	CHECK_STRING(human_default_state);
	CHECK_STRING(human_character_name);
	CHECK_BOOL(human_models_shaded);
	CHECK_BOOL(human_collide_feet_only);
	CHECK_BOOL(human_double_support);
	CHECK_INT(human_stance_state);
	CHECK_FLOAT(human_mass);
	CHECK_STRING(human_visualization_skeleton);
	CHECK_BOOL(human_show_skeleton);
	CHECK_BOOL(human_show_collision_geo);
	CHECK_BOOL(human_show_visual_geo);
	CHECK_BOOL(human_show_axis);
	CHECK_BOOL(human_show_heading);
	CHECK_BOOL(human_show_controllers);
	CHECK_BOOL(human_manual_com);
	CHECK_FLOAT(human_gain_mult);
	CHECK_FLOAT(human_desired_heading_delta);
	CHECK_FLOAT(human_desired_v_scale);
	CHECK_BOOL(human_show_support_vec);


	setType("Human");

	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif  
	
	GsString jfilename = _manager->getFiles()->getConfigFile(cnfg,"joints");
	
	_default_joint = new Serializable("default_joint");
	_default_joint->loadParametersFromFile(jfilename);

		
	for(int i=0;i<sizeOfParameter(human_joint_list);i++)
	{
		GsString jname  = pString(human_joint_list,i);
		_joints.push(new PhysicalJoint(this,jname,jfilename));
		_joint_group->add(_joints.top()->getGroup());
	}
	
	for(int i=0;i<_joints.size();i++)
	{
		if(ISJOINT(i,"Hips")) _root_joint = _joints.get(i);
		else if(ISJOINT(i,"LeftFoot")) _left_foot_joint = _joints.get(i);
		else if(ISJOINT(i,"RightFoot")) _right_foot_joint  = _joints.get(i);
		else if(ISJOINT(i,"Spine1")) _shoulder_joint = _joints.get(i);
		else if(ISJOINT(i,"LeftHand")) _left_hand_joint= _joints.get(i);
		else if(ISJOINT(i,"RightHand")) _right_hand_joint= _joints.get(i);
		else if(ISJOINT(i,"LeftToeBase")) _left_toe_joint= _joints.get(i);
		else if(ISJOINT(i,"RightToeBase")) _right_toe_joint= _joints.get(i);

	}
	setP(human_mass,computeMass());

	GsString contDir = _manager->getFiles()->getConfigFile(cnfg,"controllers");
	
	phout<<"cont dir "<<contDir<<gsnl;

	_modules.push(new COMModule(this,contDir));
	_modules.push(new RootModule(this,contDir));
	_modules.push(new GravityModule(this,contDir));
	_modules.push(new IKModule(this,contDir));
	_modules.push(new PuppetModule(this,contDir));
	_modules.push(new VirtualModule(this,contDir));	
	_modules.push(new ReferenceModule(this,contDir));
	_modules.push(new BalanceModule(this,contDir));
	_modules.push(new ContactModule(this,contDir));

	for(int i=0;i<_modules.size();i++)
	{
		_modules.get(i)->init();
		_module_group->add(_modules.get(i)->getGroup());
	}
	_desired_heading_arrow = new Arrow(GsVec(0,1,0),GsVec(0,0,1));
	_desired_heading_arrow->setColor(GsColor::green);
	_heading_arrow = new Arrow(GsVec(0,1,0),GsVec(0,0,1));
	_grp->add(_heading_arrow->getGrp());
	_grp->add(_desired_heading_arrow->getGrp());

	_vis_skel = new KnSkeleton;
	GsString visSkelName = _manager->getFiles()->getKinematicsFile(pString(human_visualization_skeleton));
	bool loaded = _vis_skel->load(visSkelName);
	if(loaded)
	{
		_vis_skel->root()->pos()->value(reference_module()->pVec(reference_skeleton_position));
		_vis_scene = new KnScene;
		_vis_scene->connect(_vis_skel);
		_vis_scene->set_skeleton_radius(0.5f);
		_vis_scene->set_axis_length(0.1f);
		_vis_scene->set_skeleton_joint_color(GsColor::red);
		_grp->add(_vis_scene);
		//	FlSkeletonWin* win = new FlSkeletonWin;
		//	win->add(dynoman);
		//	win->show();
	}
	else
	{
		phout<<"failed to load visSkel "<<visSkelName<<gsnl;
		delete _vis_skel; _vis_skel = 0;
		_vis_scene = 0;
	}

	setEditJoint(root());
	applyParameters();
}

void PhysicalHuman::makeJoints()
{
	for(int i=0;i<_joints.size();i++)
	{
		_joints[i]->makeJoint();
	}

	for(int i=0;i<_joints.size();i++)
	{
		if(ISJOINT(i,"Hips")) _root_joint = _joints.get(i);
		else if(ISJOINT(i,"LeftFoot")) _left_foot_joint = _joints.get(i);
		else if(ISJOINT(i,"RightFoot")) _right_foot_joint  = _joints.get(i);
		else if(ISJOINT(i,"Spine1")) _shoulder_joint = _joints.get(i);
		else if(ISJOINT(i,"LeftHand")) _left_hand_joint= _joints.get(i);
		else if(ISJOINT(i,"RightHand")) _right_hand_joint= _joints.get(i);
		else if(ISJOINT(i,"LeftToeBase")) _left_toe_joint= _joints.get(i);
		else if(ISJOINT(i,"RightToeBase")) _right_toe_joint= _joints.get(i);

	}
	for(int i=0;i<_joints.size();i++)
	{
		_joints[i]->applyParameters();
	}
	setP(human_mass,computeMass());
}
GsVec guessJointDim(KnJoint* j,float min)
{
	
	GsVec newD = GsVec(min,min,min);

	for(int i=0;i<j->children();i++)
	{
		KnJoint* childJ = j->child(i);
		
		if(fabs(childJ->offset().x)>newD.x) newD.x = fabs(childJ->offset().x);
		if(fabs(childJ->offset().y)>newD.y) newD.y = fabs(childJ->offset().y);
		if(fabs(childJ->offset().z)>newD.z) newD.z = fabs(childJ->offset().z);

	}
	return newD;
}
GsVec guessJointCenter(KnJoint* j)
{
	GsVec pos = j->gcenter();

	for(int i=0;i<j->children();i++)
	{
		pos += j->child(i)->gcenter();
		

	}
	pos/=((float)j->children()+1.0f);
	return pos;
}

/* old code that loaded a human froma  skeleton
PhysicalHuman::PhysicalHuman(ODEWorld* wrld, KnSkeleton* skel) : Savable("Human")
	{
	
	MAKE_PARM(human_reference_skeleton_name,"../data/ManRef.s");
	MAKE_PARM(human_configuration_name,"default");
	MAKE_PARM(human_character_name,GsString(skel->name()));
	MAKE_PARM(human_models_shaded,true);
	MAKE_PARM(human_collide_feet_only,false);
	MAKE_PARM(human_double_support,false);
	MAKE_PARM(human_stance_state,true);
	MAKE_PARM(human_mass,0.5f);

	_lines = new SnGroup;
	grp	= new SnGroup;


	sk = skel;
	
	world = wrld;

	
	GsVec origin = sk->root()->pos()->value();

	//All dimensions are x,y,z
	float size = 0.05f;

	for(int i=0;i<sk->joints().size();i++)
	{
		KnJoint* current_joint = sk->joints().get(i);
                if(current_joint->children()>0)
		{
		//only joints with children need to be made
			GsVec jointPoint = current_joint->gcenter();
			GsVec boxDim = guessJointDim(current_joint,0.1f);
			GsVec boxPos = guessJointCenter(current_joint);
			GsString name = current_joint->name();
			PhysicalJoint* prnt = 0;
			bool looking = true;

			phout<<"finding parent for "<<name<<gsnl;
			while( current_joint->parent() && looking)
			{
				if(ignoreJoint( current_joint->parent() ) )
				{

					current_joint = current_joint->parent();
				}
				else
				{
					looking = false;
					prnt = joint(GsString(current_joint->parent()->name()));
					phout<<" found parent "<<prnt->name()<<gsnl;
				}
			}
			


			//PhysicalHuman* hm,GsString jName, GsVec dim, GsVec boxPos, GsVec jointPos, PhysicalJoint* prnt
			PhysicalJoint* j = new PhysicalJoint(this,name,jointPoint,boxPos,boxDim,prnt);
			joints.push(j);
		}
	}

	for(int i=0;i<joints.size();i++)
	{
		 if(ISJOINT(i,"LeftFoot")) leftFootJoint = joints.get(i);
		else if(ISJOINT(i,"RightFoot")) rightFootJoint  = joints.get(i);
		else if(ISJOINT(i,"Neck")) shoulderJoint = joints.get(i);
		else if(ISJOINT(i,"LeftHand")) leftHandJoint= joints.get(i);
		else if(ISJOINT(i,"RightHand")) rightHandJoint= joints.get(i);
		 
	}
	setP(human_mass,computeMass());
	
	GsArray<GsString*> list; 
	for(int i=0;i<numJoints();i++)
	{
		list.push(new GsString(joint(i)->name()));
	}
	MAKE_PARM(human_joint_list,list);


	setP(human_mass, computeMass());

	grp->separator(true);
	



	for(int i=0;i<joints.size();i++)
	{
		grp->add(joints.get(i)->grp);
		
	}

	controllers.push(new ReferenceModule(this,"../data/configurations/default.ph"));
	controllers.push(new COMModule(this,"../data/configurations/default.ph"));
	controllers.push(new ContactModule(this,"../data/configurations/default.ph"));
	controllers.push(new RootModule(this,"../data/configurations/default.ph"));
	controllers.push(new GravityModule(this,"../data/configurations/default.ph"));
	controllers.push(new IKModule(this,"../data/configurations/default.ph"));
	controllers.push(new WalkController(this,"../data/configurations/default.ph"));
	controllers.push(new BalanceModule(this,"../data/configurations/default.ph"));


	ik_controller()->deactivate();

	GsArray<GsString*> clist; 
	for(int i=0;i<controllers.size();i++)
	{
		clist.push(new GsString(controllers.get(i)->name()));
		controllers.get(i)->init();
		grp->add(controllers.get(i)->getGroup());
	}

	MAKE_PARM(human_module_list,clist);

	phout<<"Human made:\n "<<toString();
	
	}
*/


PhysicalHuman::~PhysicalHuman()
{
	_joint_group->remove_all();
	_joint_group->unref();
	_module_group->remove_all();
	_module_group->unref();
	_visualization_group->remove_all();
	_visualization_group->unref();
	_grp->remove_all();
	_grp->unref();
	delete _motion_manager;
	delete _state_manager;
	for(int i=0;i<_joints.size();i++)
		delete _joints.get(i);
	
	
	for(int i=0;i<_modules.size();i++)
		delete _modules.get(i);

	//delete _knsref;
	//delete skref;

}
