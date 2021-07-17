#pragma once

#include "common.h"
#include "util_manipulator.h"
#include "util_serializable.h"
#define NUM_START_MANIPS 1
class HumanManager;
class PhysicalHuman;

enum
{
	char_manager_default_manip_size,
	char_manager_friction,
	char_manager_bounce,
	char_manager_density
};
class CharManipulator:public Manipulator
{
public:
	CharManipulator(GsVec pos,GsVec size,const GsString&  name, bool isJoint = false);
	
	GsString jointName;
	GsVec offset;
	bool isJoint;
	bool dynamic;
	bool uniqueProperties;
	GsColor color;
	float density;
	float bounce;
	float friction;
	CharManipulator* parent;
	GsArray<CharManipulator*> children;
};

class CharBuilder:public Serializable
{
		GsArray<CharManipulator*> manips;
		CharManipulator* _selected_manip;
		SnGroup* manip_group;
		HumanManager* manager;
		PhysicalHuman* human;

		bool _clicked;
	public:
		bool effectHeirarchy;
		bool mirrorChanges;
		bool orientParent;
		
		GsVec startPos;
		
		SnGroup* getGroup(){return manip_group;}
		CharBuilder(const GsString&  file);
		void select( CharManipulator* manip );
		CharManipulator* currentManip(){return _selected_manip;}
	
		CharManipulator* getManip(const GsString&  name,bool isJoint = false);
		void createCharacter();
		void makeEnv();
		void set_size( GsVec sze );
		void init( HumanManager* mgr );
		void editEnv();
		void editCharacter(PhysicalHuman* hm);

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
		void moveJoint(CharManipulator* manip);
		void translatHeirarchy(CharManipulator* manip);
		void release();
};
