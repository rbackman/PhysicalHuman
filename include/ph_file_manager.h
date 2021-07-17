# pragma once
#include "common.h"
#include "util_serializable.h"
#include "util_file_manager.h"

/*
The file manager is responsible for loading and saving files for the project which is sort of a mess right now.
right now it must be initialized to the data directory that stores all the config files for this project.
*/

class PhysicalHuman;
class Channel;
class IkManipulator;
class Motion;
class HumanManager;
class Controller;


enum file_manager_parms
{
	files_character_name,
	files_data_dir,
	files_motion_dir,
	files_scene_dir,
	files_base_motion_dir,
	files_config_dir,
	files_state_dir,
	files_kinematics_dir,
	files_motion_group_dir,
	files_motion_fmt,
	files_config_fmt,
	files_scene_fmt,
	files_state_fmt,
	files_default_config,
	files_plan_file,
	files_scene_list,
};

class HumanFileManager : public FileManager
 {
 private:
	 GsString _prefFileName;

 public :
    HumanFileManager ( GsString file);

	/*the short name of the plan file*/
	GsString planFile(){return pString(files_plan_file);}
	/*the name of the main preference file including the directory*/
	GsString getPrefFile();
	/*the directory that contains the configuration files*/
	GsString getCharacterDirectory();
	/*input name of the file outputs the relative path to config directory and adds the fmt .cfg*/
	GsString getConfigFile(const GsString& charName,const GsString&  fileName);
	/*the relative path to the directory containing motions*/
	GsString getMotionDirectory(const GsString&  character);
	/*input name of motion outputs file with directory and fmt .motion*/
	GsString getMotionFile(const GsString&  character,const GsString&  name,const GsString&  file = "control");
	/*the relative path to the directory containing the state definitions*/
	GsString getStateDirectory(const GsString&  character);
	/*input name of state outputs file with directory and fmt .state*/
	GsString getStateFile(const GsString&  state,const GsString&  file);
	/*input name of file including format outputs with directory appended*/
	GsString getKinematicsFile( const GsString&  name );
	/*the relative path to the directory containing kinematic motions .bvh or .sm*/
	GsString getKinematicsDirectory();
	

	/*input a motion including relative directory then saves it to a file*/
	void saveMotion(const GsString&  fileName, Motion* m);
	/*input short name of configuration then saves it to a file in the config directory*/
	void saveConfigurationNamed(PhysicalHuman* human,const GsString&  cnfgName);
	/*input short name of state and human then saves it to a file in the state directory*/
	void saveStateNamed(PhysicalHuman* human,const GsString&  stateName);
	void saveMotionStateNamed(PhysicalHuman* human,const GsString&  stateName,const GsString&  motionName);
	void saveMotionTrajectories(const GsString&  fileName, Motion* m);
	GsString makeControllerDirectory(const GsString&  character,const GsString&  controllerName);
	void saveBounds( const GsString&  dir, Motion* motion );
	void saveBaseMotion( const GsString&  dir, Motion* m );
	void saveScene( const GsString&  name ,HumanManager* manager);
	GsString getModelDirectory();
	GsString getControlFile(const GsString&  name);
	GsString getSceneDirectory();
	GsString getSceneFile(const GsString&  fileName);
	void saveStateToDirectory(PhysicalHuman* human,const GsString&  dir);
};


