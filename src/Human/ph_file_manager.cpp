


#include "ph_file_manager.h"
#include "ph_manager.h"
#include "ph_human.h"
#include "util_motion.h"
#include "ph_mod_puppet.h"
#include "ph_mod_ik.h"
#include "ph_mod_virtual.h"
#include "ph_mod_com.h"
#include "ph_mod_contact.h"
#include "ph_mod_balance.h"
#include "ph_controller.h"

#include <gsim/gs_scandir.h>

HumanFileManager::HumanFileManager (GsString dir) : FileManager(dir)
{
	_prefFileName = "pref.dat";

	loadParametersFromFile(getPrefFile());

	
	CHECK_STRING(files_character_name);
	CHECK_STRING(files_data_dir);
	CHECK_STRING(files_motion_dir);
	CHECK_STRING(files_scene_dir);
	CHECK_STRING(files_base_motion_dir);
	CHECK_STRING(files_config_dir);
	CHECK_STRING(files_state_dir);
	CHECK_STRING(files_kinematics_dir);
	CHECK_STRING(files_motion_group_dir);
	CHECK_STRING(files_config_fmt);
	CHECK_STRING(files_motion_fmt);
	CHECK_STRING(files_state_fmt);
	CHECK_STRING(files_scene_fmt);
	CHECK_STRING(files_default_config);
	CHECK_STRING(files_plan_file);
	CHECK_STRING(files_scene_list);

	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif  

}
GsString HumanFileManager::getPrefFile()
{
	return getDataDir() <<_prefFileName;
}
GsString HumanFileManager::getSceneDirectory()
{
	return getDataDir() <<pString(files_scene_dir)<<SLASH;
}
GsString HumanFileManager::getSceneFile(const GsString& fileName)
{
	return getSceneDirectory() <<fileName<<pString(files_scene_fmt);
}

GsString HumanFileManager::getCharacterDirectory()
{
	return getDataDir() << pString(files_config_dir)<<SLASH;
}
GsString HumanFileManager::getConfigFile(const GsString& charName,const GsString& fileName)
{
	return getCharacterDirectory() << charName <<SLASH<<fileName<<pString(files_config_fmt);
}
GsString HumanFileManager::getMotionDirectory(const GsString& character)
{
	return getCharacterDirectory()<<character<<SLASH<<pString(files_motion_dir)<<SLASH;
}
GsString HumanFileManager::getModelDirectory()
{
	return getDataDir() <<"models"<<SLASH;
}
GsString HumanFileManager::getControlFile(const GsString& name)
{

	return getDataDir() <<SLASH<<name;
}
GsString HumanFileManager::getMotionFile(const GsString& character,const GsString&name,const GsString&file)
{
	return getDataDir() << getMotionDirectory(character) << name <<SLASH<<file<<pString(files_motion_fmt);
}
GsString HumanFileManager::getStateDirectory(const GsString&character)
{
	return getCharacterDirectory() <<character<<SLASH<<pString(files_state_dir)<<SLASH ;
}
GsString HumanFileManager::getStateFile(const GsString&state,const GsString&file)
{
	GsString stateFile = state;
	stateFile  <<SLASH<<file<<pString(files_state_fmt);
	return  stateFile;
}
GsString HumanFileManager::getKinematicsFile( const GsString&file )
{
	return getDataDir() << getKinematicsDirectory() << file;
}
GsString HumanFileManager::getKinematicsDirectory( )
{
	return getDataDir() << pString(files_kinematics_dir) <<SLASH;
}

void HumanFileManager::saveScene(const GsString& name ,HumanManager* manager )
{
	GsString fileName = getSceneFile(name);

	GsOutput f;
	if(f.open(fileName))
	{
		f<<"#scene for physical humanoid\n";
		f<<"#Robert Backman 8/15/12\n\n";
		f <<"Scene\n{\n\tscene_object_list = ";
		for (int i=0;i<manager->numObjects();i++)
		{
			if(manager->getObject(i)->isGround())
			{
				f<<" "<<manager->getObject(i)->name();
			}
			
		}
		f<<";\n}\n";
		
		for (int i=0;i<manager->numObjects();i++)
		{
			ODEObject* obj = manager->getObject(i);
			if(obj->isGround())
			{
			//	f<<obj->toString();

				f<<obj->name()<<"\n{\n";
				f<<obj->parameterAsString(ode_position);
				f<<obj->parameterAsString(ode_box_dim);
				f<<obj->parameterAsString(ode_dynamic);
				if(obj->pBool(ode_unique_properties))
				{
					f<<obj->parameterAsString(ode_unique_properties);
					f<<obj->parameterAsString(ode_density);
					f<<obj->parameterAsString(ode_bounce);
					f<<obj->parameterAsString(ode_friction);
				}
				f<<obj->parameterAsString(ode_object_color);
				f<<"}\n";
			}

		}
		f<<"end\n";
		f.close();

	}else phout<<"failed to save motion "<<fileName<<gsnl;

}

void HumanFileManager::saveMotion(const GsString& fileName, Motion* m)
{
	m->updateChannelList();
	GsOutput f;
	if(f.open(fileName))
	{
		f<<"#motion for physical humanoid\n";
		f<<"#Robert Backman 10/28/11\n\n";
		f <<m->toString();
		f.close();
		
	}else phout<<"failed to save motion "<<fileName<<gsnl;

}
void HumanFileManager::saveBounds( const GsString& fileName, Motion* motion )
{
	GsOutput f;
	if(f.open(fileName))
	{
		f<<"#trajectories motion for physical humanoid\n";
		f<<"#Robert Backman 3/15/12\n\n";
		f <<motion->boundsString();
		f.close();

	}else phout<<"failed to save motion bounds"<<fileName<<gsnl;
}
void HumanFileManager::saveBaseMotion(const GsString& fileName,Motion* motion )
{
	GsOutput f;
	if(f.open(fileName))
	{
		f<<"#trajectories motion for physical humanoid\n";
		f<<"#Robert Backman 3/15/12\n\n";
		f <<motion->baseString();
		f.close();

	}else phout<<"failed to save motion bounds"<<fileName<<gsnl;
}


void HumanFileManager::saveMotionTrajectories(const GsString& fileName, Motion* m)
{
	m->updateChannelList();
	GsOutput f;
	if(f.open(fileName))
	{
		f<<"#trajectories motion for physical humanoid\n";
		f<<"#Robert Backman 3/15/12\n\n";
		f <<m->trajectoryString();
		f.close();

	}else phout<<"failed to save motion trajectories"<<fileName<<gsnl;
}
void HumanFileManager::saveConfigurationNamed(PhysicalHuman* human,const GsString& cnfgName)
{
	human->setP(human_character_name,cnfgName);
	
	GsString cfgDir = pString(files_config_dir)<<SLASH;
	
	makeSubDirectory(cfgDir,cnfgName);
	cfgDir<<cnfgName<<SLASH;
	gsout<<"cfgDir "<<cfgDir<<gsnl;
	makeSubDirectory(cfgDir,"motions");
	makeSubDirectory(cfgDir,"states");

	GsOutput fo;
	if(fo.open(getConfigFile(cnfgName,"human")))
	{
		fo<<"#parameters for physical humanoid\n";
		fo<<"#Robert Backman 11/4/11\n\n";
		fo << human->toString();
		fo.close();
	}else phout<<"failed to save configuration";

	if(fo.open(getConfigFile(cnfgName,"controllers")))
	{
		
		fo<<"#CONTROLLER_DEFINITIONS#############################\n";
		fo<<human->moduleString();
		fo.close();

	}else phout<<"failed to save configuration";
	if(fo.open(getConfigFile(cnfgName,"joints")))
	{

		fo<<"#JOINT_DEFINITIONS#############################\n";
		fo<<human->jointString();
		fo.close();

	}else phout<<"failed to save configuration";

	if(fo.open(getConfigFile(cnfgName,"ik")))
	{
		fo<<human->ik_module()->manipString();
		fo.close();
	}else phout<<"failed to save  ik configuration";

	if(fo.open(getConfigFile(cnfgName,"virtual")))
	{
		fo<<human->virtual_module()->manipString();
		fo.close();
	}else phout<<"failed to save virtual configuration";

	if(fo.open(getConfigFile(cnfgName,"puppet")))
	{
		fo<<human->puppet_module()->manipString();
		fo.close();
	}else phout<<"failed to save puppet configuration";
	
	gsout<<"made configs "<<cnfgName<<gsnl;
}

GsString HumanFileManager::makeControllerDirectory(const GsString&character,const GsString&controllerName)
{
	GsString dirName = getMotionDirectory(character);
	dirName<<controllerName<<SLASH;
	makeDirectory(dirName);
	return dirName;
}
void HumanFileManager::saveStateToDirectory(PhysicalHuman* human,const GsString&dr)
{
	makeDirectory(dr);
	GsString dir;
	dir<<getDataDir()<<dr;
	
	GsString fileName = getStateFile(dir,"human");

	GsOutput f;
	if(f.open(fileName))
	{

		f<<"#state file for physical humanoid\n";
		f<<"#Robert Backman 11/4/2011\n\n";
		f<<human->stateString();
		human->ik_module()->matchToSkeleton();
		for(int i=0;i<human->numModules();i++)
		{
			f<<	human->getModule(i)->stateString();
		}

		f<<"\n\nend";
		f.close();

	}else phout<<"couldn't save human state"<<fileName<<gsnl;

	fileName = getStateFile(dir,"ik");
	if(f.open(fileName))
	{

		f<<"#state file for physical humanoid\n";
		f<<"#Robert Backman 11/4/2011\n\n";
		f<< human->ik_module()->manipStateString();
		f<<"\n\nend";
		f.close();

	}else phout<<"couldn't save ik state "<<fileName<<gsnl;

	fileName = getStateFile(dir,"virtual");
	if(f.open(fileName))
	{

		f<<"#state file for physical humanoid\n";
		f<<"#Robert Backman 11/4/2011\n\n";
		f<< human->virtual_module()->manipStateString();
		f<<"\n\nend";
		f.close();

	}else phout<<"couldn't save virtual state "<<fileName<<gsnl;;

	fileName = getStateFile(dir,"joints");
	if(f.open(fileName))
	{
		f<<"#state file for physical humanoid\n";
		f<<"#Robert Backman 11/4/2011\n\n";
		f<< human->jointStateString();
		f<<"\n\nend";
		f.close();
	}else phout<<"couldn't save joint state"<<fileName<<gsnl;;;
}
void HumanFileManager::saveMotionStateNamed(PhysicalHuman* human,const GsString&stateName,const GsString&motionName)
{

	GsString dir = getMotionDirectory(human->characterName());
	dir<<stateName<<SLASH;
	makeDirectory(dir);

	saveStateToDirectory(human,dir);
}
void HumanFileManager::saveStateNamed(PhysicalHuman* human,const GsString&stateName)
{
	//GsString sdir =  pString(files_config_dir);
	//gsout<<"for some reason this only works if directory is already made??";
	GsString sdir;
	sdir<<pString(files_config_dir)<<SLASH<<human->characterName()<<SLASH<<"states"<<SLASH<<stateName;
	saveStateToDirectory(human,sdir);
}


