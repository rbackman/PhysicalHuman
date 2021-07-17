#pragma once
#include "common.h"
#include "util.h"
#include "ph_human.h"
#include "ph_joint.h"
#include "util_serializable.h"
#include "ph_parameters.h"

class SnGroup;

class Module : public Serializable
{
protected:
	bool _manip_controller;
	SnGroup* grp; //for displaying anything you wish ;-)
	HumanManager* manager;
public:
	Module(PhysicalHuman* human,GsString cname,const char* file);
	~Module(void);
	virtual void init();
	virtual bool evaluate();
	virtual GsArray<Serializable*> getSerializables();
	virtual GsString stateString();
	virtual void loadStateFromFile(GsString file);
	void applyParameters();
	PhysicalHuman* h;
	SnGroup* getGroup(){return grp;}
	void visible(bool v); 
	bool manipController(){return _manip_controller;}
	bool isActive(){return pBool(module_active);}
	void deactivate(){setP(module_active,false);}
	void activate(){ setP(module_active,true);}
	void toggleActive(){isActive()?deactivate():activate();}	
};
