#pragma once
#include "common.h"

#include "util_serializable.h"

#include "ph_human.h"

class HumanManager;




class HumanState
{
	/*relative path to the directory that holds the state information*/
	GsString _directory;
	
public:
	
	/*the group that holds the human definitions and base controller definitions*/
	SerializableGroup* g;
	/*the group that holds the PhysicalJoint definitions*/
	SerializableGroup* jointDefs;
	/*the group that hold the ik manipulator definitions*/
	SerializableGroup* ikDefs;
	/*the group that holds the virtual manipulator definitions*/
	SerializableGroup* virtDefs;
	GsString stateShortName()
	{
		GsString nme = _directory;
		remove_path(nme);
		return nme;
	}
	GsString getDirectory(){return _directory;}
	bool saved;
	/*pass in the directory that holds the state info. the name of the state contains the directory */
	bool load(const GsString&  stateDirName);
	void apply(PhysicalHuman* h);
	void capture(PhysicalHuman* h);
	HumanState();
	~HumanState();
	void setDirectory( const GsString&  s );
	
};


class HumanStateManager : public Serializable
{
private:
	
	HumanManager* manager;
	PhysicalHuman* human;
	GsArray<HumanState*> states;
	HumanState* _selected_state;
	GsVec _stancePointStart;
public:
	GsVec getStancePointStart(){return _stancePointStart;}
	HumanStateManager(const GsString&  file);
	~HumanStateManager();
	void init(PhysicalHuman* hum,HumanManager* mgr);

	HumanState* getState(int i){return states[i];}
	HumanState* getSelectedState(){return _selected_state;}

	int loadState(const GsString&  name,bool forceReload = false);
	HumanState* openState(const GsString&  dirName);
	void selectState( const GsString&  dirName,bool forceReload = false);
	void reset();
	GsString currentStateName();
	void selectCurrentState();

	HumanState* snapshot();
	HumanState* snapshot(const GsString&  s);	
	void deleteState( const GsString&  name );
	void loadDefaultState();
	void selectCharacterState( const GsString& dirName,bool forceReload = false);
	void loadInitialState();
};
