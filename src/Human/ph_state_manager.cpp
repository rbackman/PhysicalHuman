#include "ph_state_manager.h"

#include "ph_manager.h"
#include "ph_file_manager.h"
#include "ph_human.h"

HumanStateManager::HumanStateManager(const GsString&  file):Serializable("HumanStateManager")
{

	loadParametersFromFile(file);
	_selected_state = 0;
	
}
void HumanStateManager::init(PhysicalHuman* hum,HumanManager* mgr)
{
	human = hum;
	manager= mgr;
}
int HumanStateManager::loadState(const GsString&  dirName,bool forceReload)
{
	
	//phout<<"trying to open state "<<dirName<<gsnl;
	for (int i=0;i<states.size();i++)
	{
		if (states[i]->getDirectory()==dirName)
		{
			if(forceReload)
			{
				delete states[i];
				states[i]=0;
				states.remove(i);
			}
			else
			{
				//phout<<"state "<<dirName<<" is already loaded "<<gsnl;
				return i;
			}
		}
	}
	
	HumanState* state = openState(dirName);

	if(state)
	{
		states.push(state);
	}
	else
	{
		return -1;
	}
	return states.size()-1;
}
HumanState* HumanStateManager::openState(const GsString&  dirName)
{
	HumanState* state = new HumanState();
#ifdef DEBUG
	phout<<"openState "<<dirName<<gsnl;
#endif

	if(state->load(dirName))
	{
		//phout<<"opened state "<<dirName<<gsnl;
	}
	else
	{
		phout<<"state open failed "<<dirName<<gsnl;
		delete state;
		state = 0;
	}
	return state;
}
void HumanStateManager::selectCharacterState( const GsString&  stateName,bool forceReload)
{
	GsString dirName = manager->getFiles()->getCharacterDirectory();
	dirName<<manager->selectedCharacter()->characterName()<<SLASH<<"states"<<SLASH<<stateName;
	selectState(dirName,forceReload);
}
void HumanStateManager::selectState( const GsString&  dirName,bool forceReload)
{
	human->reset();
	if(dirName=="none")
	{
		manager->message("trying to load 'none' as state");
		return;
	}
	int stateID = loadState(dirName,forceReload);
	if(stateID==-1)
	{
		manager->message("State load failed:",dirName);
		return;
	}
	gsout<<"selected state is "<<dirName<<gsnl;
	_selected_state = states[stateID];
	_selected_state->apply(human);
	_stancePointStart = human->stancePoint();
}



HumanStateManager::~HumanStateManager()
{
	for(int i=0;i<states.size();i++)
	{
		delete states[i];
	}


}

void HumanStateManager::reset()
{
	if(getSelectedState())
		selectState(getSelectedState()->getDirectory());
}

GsString HumanStateManager::currentStateName()
{
	if(_selected_state)
		return _selected_state->getDirectory();
	else
		return "none";
}

void HumanStateManager::selectCurrentState()
{
	if(currentStateName()!="none")
		selectState(currentStateName());
}

HumanState* HumanStateManager::snapshot()
{

	GsString s = manager->getFiles()->getStateDirectory(human->characterName());
	s<<"Snapshot_";
	int sub = ((int)(gs_time()/100))*100;
	double t = gs_time() - sub;

	s<< (int)(t*10);

	return snapshot(s);
}
HumanState* HumanStateManager::snapshot(const GsString&  s)
{
	states.push() = new HumanState();
	states.top()->saved = false;
	states.top()->setDirectory(s);
	states.top()->capture(human);

	return states.top();
}

void HumanStateManager::deleteState( const GsString&  name )
{
	for(int i=0;i<states.size();i++)
	{
		if(states[i]->getDirectory() == name)
		{
			if(!states[i]->saved)
			{
				delete states[i];
				states.remove(i);
				return;
			}
			manager->message("trying to delete saved state");
		}
	}
	manager->message("couldn't find state to delete");
}

void HumanStateManager::loadDefaultState()
{
	selectCharacterState(manager->selectedCharacter()->pString(human_default_state));
}

void HumanStateManager::loadInitialState()
{
	selectCharacterState("initial");
}

#include "ph_mod_ik.h"
#include "ph_mod_virtual.h"
#include "ph_manip.h"
#include "ph_mod_com.h"
#include "ph_mod_contact.h"
#include "ph_mod_balance.h"
#include "ph_mod_root.h"

bool HumanState::load( const GsString&  stateDirectory )
{
	_directory = stateDirectory;
	g = new SerializableGroup();
	GsString fileName = stateDirectory;
	fileName<<SLASH<<"human.state";

	if(!g->loadFromFile(fileName))
		return false;

	if(g->getSerializable("Human")==0)
	{
		phout<<fileName<<"has no 'Human' group\n";
		return false;
	}

	jointDefs = new SerializableGroup();
	fileName = stateDirectory;
	fileName<<SLASH<<"joints.state";
	if(!jointDefs->loadFromFile(fileName))
	{	
		delete jointDefs;
		jointDefs = 0;
	}
	if(g->getSerializable("IKModule"))
	{
		ikDefs = new SerializableGroup();
		fileName = stateDirectory;
		fileName<<SLASH<<"ik.state";
		if(!ikDefs->loadFromFile(fileName))
		{
			delete ikDefs;
			ikDefs = 0;
		}
	}
	if(g->getSerializable("VirtualModule"))
	{
		virtDefs = new SerializableGroup();
		fileName = stateDirectory;
		fileName<<SLASH<<"virtual.state";
		if(!virtDefs->loadFromFile(fileName))
		{
			delete virtDefs;
			virtDefs = 0;
		}

	}
	return true;
}

HumanState::HumanState()
{

	g = 0;
	ikDefs =0;
	virtDefs=0;
	jointDefs=0;
	saved = false;
	_directory = "none";
}

void HumanState::apply(PhysicalHuman* h)
{

	h->setParametersFromSerializable(g->getSerializable("Human"));
	h->applyParameters();
	for(int i=0;i<h->numJoints();i++)
	{
		PhysicalJoint* j = h->joint(i);
		KnJoint* knj = h->skref()->joint(j->name());
		Serializable* jSave = jointDefs->getSerializable(j->name());
		j->setParametersFromSerializable(jSave);
		j->getBody()->setParametersFromSerializable(jSave);
		j->getBody()->applyParameters();
		if(knj)
		{
			if(j->getParameter(joint_setpoint_rot))
				knj->rot()->value(j->pQuat(joint_setpoint_rot));
		}
	}

	if(g->getSerializable("IKModule"))
	{
		h->ik_module()->setParametersFromSerializable(g->getSerializable("IKModule"));
		h->ik_module()->applyParameters();
		for(int i=0;i<h->ik_module()->numManips();i++)
		{
			HumanManipulator* manip = h->ik_module()->getManip(i);
			manip->setParametersFromSerializable(ikDefs->getSerializable(manip->name()));
			manip->applyParameters();
		}
		h->ik_module()->solve();
	}

	if(g->getSerializable("VirtualModule"))
	{
		h->virtual_module()->setParametersFromSerializable(g->getSerializable("VirtualModule"));
		for(int i=0;i<h->virtual_module()->numManips();i++)
		{
			HumanManipulator* manip = h->virtual_module()->getManip(i);
			manip->setParametersFromSerializable(ikDefs->getSerializable(manip->name()));
			manip->applyParameters();
		}
		h->virtual_module()->matchToHuman();
	}
	if(g->getSerializable("COMModule"))
	{
		h->com_module()->setParametersFromSerializable(g->getSerializable("COMModule"));
		h->com_module()->applyParameters();
	}
	if(g->getSerializable("ContactModule"))
	{
		h->contact_module()->setParametersFromSerializable(g->getSerializable("ContactModule"));
		h->contact_module()->applyParameters();
	}
	if(g->getSerializable("BalanceModule"))
	{
		h->balance_module()->setParametersFromSerializable(g->getSerializable("BalanceModule"));
		h->balance_module()->applyParameters();
	}
	h->root_module()->setParametersFromSerializable(g->getSerializable("RootModule"));
	h->update();
	
}

void HumanState::capture( PhysicalHuman* h )
{
	if(g==0)
	{
		g = new SerializableGroup;

		GenericSerializable* human = new GenericSerializable("human");
		g->addSerializable(human);
		human->makeParameter(h->getParameter(human_stance_state));
		human->makeParameter(h->getParameter(human_manual_com));
		human->makeParameter(h->getParameter(human_gain_mult));

		GenericSerializable* ik = new GenericSerializable("IKModule");
		ik->makeParameter(h->ik_module()->getParameter(module_active));
		g->addSerializable(ik);

		GenericSerializable* virtualC = new GenericSerializable("VirtualModule");
		virtualC->makeParameter(h->virtual_module()->getParameter(module_active));
		g->addSerializable(virtualC);

		GenericSerializable* comC = new GenericSerializable("COMModule");
		comC->makeParameter(h->com_module()->getParameter(com_desired_pos));
		comC->makeParameter(h->com_module()->getParameter(com_desired_support_vector));
		g->addSerializable(comC);

		GenericSerializable* conC = new GenericSerializable("ContactModule");
		conC->makeParameter(h->contact_module()->getParameter(contact_toe_heel_ratio));
		conC->makeParameter(h->contact_module()->getParameter(contact_stance_swing_ratio));
		conC->makeParameter(h->contact_module()->getParameter(contact_stance_offset));

		g->addSerializable(conC);

		GenericSerializable* balC = new GenericSerializable("BalanceModule");
		balC->makeParameter(h->balance_module()->getParameter(balance_jcom_velocity_desired));
		balC->makeParameter(h->balance_module()->getParameter(balance_simbicon_active));
		g->addSerializable(balC);

		jointDefs = new SerializableGroup;
		for(int i=0;i<h->numJoints();i++)
		{
			PhysicalJoint* jt = h->joint(i);
			GenericSerializable* j = new GenericSerializable(jt->name());
			j->makeParameter(jt->getBody()->getParameter(ode_position));
			j->makeParameter(jt->getBody()->getParameter(ode_velocity));
			j->makeParameter(jt->getBody()->getParameter(ode_orientation));
			j->makeParameter(jt->getBody()->getParameter(ode_rotational_velocity));
			j->makeParameter(jt->getParameter(joint_setpoint_rot));
			j->makeParameter(jt->getParameter(joint_desired_rotational_velocity));
			j->makeParameter(jt->getParameter(joint_char_frame));
			jointDefs->addSerializable(j);
		}

		ikDefs = new SerializableGroup;
		for(int i=0;i<h->ik_module()->numManips();i++)
		{
			Manipulator* manip = h->ik_module()->getManip(i);
			GenericSerializable* m = new GenericSerializable(manip->name());
			m->makeParameter(manip->getParameter(manipulator_position));
			m->makeParameter(manip->getParameter(manipulator_orientation));
			m->makeParameter(manip->getParameter(manipulator_active));
			m->makeParameter(manip->getParameter(manipulator_visible));
			m->makeParameter(manip->getParameter(ik_manip_orbit));

			ikDefs->addSerializable(m);
		}


		virtDefs = new SerializableGroup;
		for(int i=0;i<h->virtual_module()->numManips();i++)
		{
			Manipulator* manip = h->virtual_module()->getManip(i);
			GenericSerializable* m = new GenericSerializable(manip->name());
			m->makeParameter(manip->getParameter(manipulator_position));
			m->makeParameter(manip->getParameter(manipulator_orientation));
			m->makeParameter(manip->getParameter(manipulator_active));
			m->makeParameter(manip->getParameter(manipulator_visible));

			virtDefs->addSerializable(m);
		}
	}
	else
	{

		g->getSerializable("Human")->setParametersFromSerializable(h);
		g->getSerializable("IKModule")->setParametersFromSerializable(h->ik_module());
		g->getSerializable("VirtualModule")->setParametersFromSerializable(h->virtual_module());
		g->getSerializable("COMModule")->setParametersFromSerializable(h->com_module());
		g->getSerializable("ContactModule")->setParametersFromSerializable(h->contact_module());


		for(int i=0;i<h->numJoints();i++)
		{
			PhysicalJoint* jt = h->joint(i);
			GenericSerializable* j = jointDefs->getSerializable(jt->name());

			j->setParametersFromSerializable(jt);
			j->setParametersFromSerializable(jt->getBody());
		}

		
		for(int i=0;i<h->ik_module()->numManips();i++)
		{
			Manipulator* manip = h->ik_module()->getManip(i);
			GenericSerializable* m = ikDefs->getSerializable(manip->name());
			m->setParametersFromSerializable(manip);
		}

		for(int i=0;i<h->virtual_module()->numManips();i++)
		{
			Manipulator* manip = h->virtual_module()->getManip(i);
			GenericSerializable* m = virtDefs->getSerializable(manip->name());
			m->setParametersFromSerializable(manip);
		}
	}
}

HumanState::~HumanState()
{
	
	if(jointDefs)delete jointDefs;
	if(ikDefs)delete ikDefs;
	if(virtDefs)delete virtDefs;

	if(g) delete g;
	

}

void HumanState::setDirectory( const GsString&  s )
{
	_directory = s;
}
