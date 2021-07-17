#pragma once

#include "common.h"
#include "util_manipulator.h"
#include "util_serializable.h"
#define NUM_START_MANIPS 1
class HumanManager;
enum
{
	env_manager_default_manip_size,
	env_manager_friction,
	env_manager_bounce,
	env_manager_density
};
class EnvManipulator:public Manipulator
{
public:
	EnvManipulator(GsVec pos,GsVec size);
	bool dynamic;
	bool uniqueProperties;
	GsColor color;
	float density;
	float bounce;
	float friction;
};
class EnvBuilder:public Serializable
{
		GsArray<EnvManipulator*> manips;
		EnvManipulator* _selected_manip;
		SnGroup* manip_group;
		HumanManager* manager;
		bool _clicked;
	public:
		SnGroup* getGroup(){return manip_group;}
		EnvBuilder(const GsString&  file);
		~EnvBuilder();

		void select( EnvManipulator* manip );
		EnvManipulator* currentManip(){return _selected_manip;}
		void makeEnv();
		void set_size( GsVec sze );
		void init( HumanManager* mgr );
		void editEnv();
		bool manipClicked()
		{
			if(_clicked)
			{
				_clicked = false;
				return true;
			}
			return false;
		}
		void duplicateBlock();
		void addBlock();
		
};
