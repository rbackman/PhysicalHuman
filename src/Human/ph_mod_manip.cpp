#include "ph_mod_manip.h"
#include "util_manipulator.h"
#include "ph_manip_frame.h"
#include "ph_manager.h"
#include "ph_file_manager.h"

ManipulatorModule::ManipulatorModule(PhysicalHuman* human,GsString controllerName,GsString file):Module(human,controllerName,file)
{
	_shortName = "noName";
	CHECK_BOOL(manip_module_list);
	CHECK_BOOL(manip_module_match);
	CHECK_BOOL(manip_module_match_frame);
	CHECK_VEC(manip_module_origin);
	CHECK_QUAT(manip_module_frame);

	_manip_controller=true;
}

void ManipulatorModule::lockSwing( bool val )
{
	getManip(h->swingFoot()->name())->setP(manipulator_active,val);
	getManip(h->swingFoot()->name())->setP(virtual_spring_use_orientation,val);
}


HumanManipulator*  ManipulatorModule::makeManip(GsString nme,GsString file)
{
	phout<<name()<<"should implement its own makeManip method\n";
	return new HumanManipulator(this,nme,"DefaultManip",file);
}

HumanManipulator* ManipulatorModule::getManip(GsString l)
{
	for(int i=0;i<manips.size();i++)
	{
		if(manips[i]->name()==l)return manips[i];
	}
	return 0;
}
GsArray<Serializable*>  ManipulatorModule::getSerializables()
{
	GsArray<Serializable*> sav;
	sav.push(Module::getSerializables());
	for(int i=0;i<manips.size();i++)
		sav.push((Serializable*)manips[i]);
	return sav;
}
void ManipulatorModule::matchFrameToHuman()
{
	GsVec p = GsVec(h->hips()->getCOMPosition().x,pVec(manip_module_origin).y, h->hips()->getCOMPosition().z);
	GsQuat q = h->getHeading();
	setOrigin(p,q);
	frameManip->translation(p);
	frameManip->rotation(q);
	
}
GsString ManipulatorModule::manipString()
{
	GsString str;
	str<<"#config file for "<< name() <<"  manips.. they are in a separate file since they have the same names as the joints\n";
	for(int i=0;i<manips.size();i++)
	{
		str << manips.get(i)->toString();
	}
	str<<frameManip->toString();
	return str;
}
GsString  ManipulatorModule::stateString()
{
	GsString f = name();
	f<<"\n{\n";
	f<<parameterAsString(module_active);
	f<<parameterAsString(manip_module_list);
	f<<parameterAsString(manip_module_origin);
	f<<parameterAsString(manip_module_frame);
	f<<"}\n";
	return f;
}
GsString ManipulatorModule::manipStateString()
{
	GsString manipString = "#state file for ";
	manipString<< name() <<"  manips.. they are in a separate file since they have the same names as the joints\n";
	for(int i=0;i<manips.size();i++)
	{
		manipString << manips.get(i)->stateString();
	}
	manipString<<frameManip->stateString();
	return manipString;
}

void ManipulatorModule::activateManip(GsString name)
{
	for(int i=0;i<manips.size();i++)
	{
		if(manips[i]->name() == name)
		{
			manips[i]->setP(manipulator_active,true);
			return;
		}
	}
	phout<<"couldn't find manip "<<name<<" to activate\n";
}
void ManipulatorModule::deactivateManip(GsString name)
{
	for(int i=0;i<manips.size();i++)
	{
		if(manips[i]->name() == name)
		{
			manips[i]->setP(manipulator_active,false);
			return;
		}
	}
	phout<<"couldn't find manip "<<name<<" to deactivate\n";
}


void ManipulatorModule::matchManips()
{
	for(int i=0;i<manips.size();i++)
	{
		manips[i]->match();
	}
}

void ManipulatorModule::matchToSkeleton(KnSkeleton* sk)
{
	sk->update_global_matrices();
	for(int i=0;i<manips.size();i++)
	{
		KnJoint* j = sk->joint(manips[i]->name());
		if(j)
			manips[i]->match(j);
	}
}
void ManipulatorModule::matchToSkeleton()
{
	matchToSkeleton( h->skref());
}
void ManipulatorModule::matchToHuman()
{
	for(int i=0;i<manips.size();i++)
	{
		PhysicalJoint* j = h->joint(manips[i]->name());
		if(j)
			manips[i]->match(j);
	}
}


static void manipCallback ( SnManipulator* mnp,const GsEvent& ev, void* udata )
{
	ManipulatorModule* mc = ((HumanManipulator*)mnp)->controller;

	((Manipulator*)mnp)->evaluate();


}


//#include "ph_app_main.h"

bool  ManipulatorModule::evaluate()
{
	Module::evaluate();
	
	if(!isActive())return false;
	if(manager->animationStep())
	{
		if(pBool(manip_module_match_frame))
		{
			matchFrameToHuman();
		}
		if(pBool(manip_module_match))
		{
			matchToSkeleton();
		}
		//GsVec2 p;
		//if(frameManip->wantsPopupMenu(&p))
		//	App->loadPopUpMenu(frameManip,p);
	
		for(int i=0;i<manips.size();i++)
		{
			manips[i]->evaluate();

			/*if (manips[i]->wantsPopupMenu(&p))
			{
			App->loadPopUpMenu(manips[i],p);
			}*/
		}
	}
	return true;
}

void ManipulatorModule::init()
{
	GsString charName = h->characterName();
	GsString shortName = getShortName();
	Module::init();
	manipGroup = new SnGroup;
	manipGroup->separator(true);
	grp->add(manipGroup);
	frameManip = new FrameManipulator(this,manager->getFiles()->getConfigFile(charName,shortName));
	frameManip->callback(manipCallback,this);
	
	manipGroup->add(frameManip);
	grp->add(frameManip->getGroup());
	
	for(int i=0;i<pStringArray(manip_module_list)->size();i++)
	{
		GsString jointName = pString(manip_module_list,i);
		manips.push(makeManip(jointName,manager->getFiles()->getConfigFile(charName,shortName)));
		manips[i]->callback(manipCallback,this);
		manipGroup->add(manips[i]);
		grp->add(manips[i]->getGroup());
	}
	
	applyParameters();
}

void ManipulatorModule::setOrigin( GsVec pos, GsQuat rot )
{
	setP(manip_module_origin,pos);
	setP(manip_module_frame,rot);
	frameManip->setP(manipulator_position,pos);
	frameManip->setP(manipulator_orientation,rot);
	for (int i=0;i<manips.size();i++)
	{
		manips[i]->setOrigin(pos,rot);
	}
}

void ManipulatorModule::applyParameters()
{
	frameManip->translation(pVec(manip_module_origin));
	frameManip->rotation(pQuat(manip_module_frame));
	for (int i=0;i<manips.size();i++)
	{
		manips[i]->setOrigin(pVec(manip_module_origin),pQuat(manip_module_frame));
	}
	evaluate();
	//setOrigin(frameManip->globalPosition(),frameManip->globalOrientation());
	//setOrigin(pVec(manip_module_origin),pQuat(manip_module_frame));
	Module::applyParameters();
}

HumanManager* ManipulatorModule::getManager()
{
	return manager;
}
