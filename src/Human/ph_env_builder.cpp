#include "ph_env_builder.h"
#include "ode_world.h"
#include "ph_manager.h"
#include "ode_object.h"
static void manipCallback ( SnManipulator* mnp,const GsEvent& ev, void* udata )
{
	EnvBuilder* mc = ((EnvBuilder*)udata);
	EnvManipulator* manip = (EnvManipulator*)mnp;
	mc->select(manip);
	manip->setColor(GsColor::red);
	manip->evaluate();
}
EnvBuilder::~EnvBuilder()
{
	manip_group->remove_all();
	for(int i=0;i<manips.size();i++)
	{
		delete	manips[i];
		manips[i] = 0;
	}
	manips.size(0);
	manip_group->unref();

}
EnvBuilder::EnvBuilder(const GsString&  file):Serializable("EnvBuilder")
{
	loadParametersFromFile(file);
	CHECK_VEC(env_manager_default_manip_size);
	CHECK_FLOAT(env_manager_friction);
	CHECK_FLOAT(env_manager_bounce);
	CHECK_FLOAT(env_manager_density);

	_clicked = false;
	manip_group = new SnGroup;
	manip_group->ref();
	
	_selected_manip = 0;
	for(int i=0;i<NUM_START_MANIPS;i++)
	{
		EnvManipulator* m = new EnvManipulator(GsVec(0,1,1+i),pVec(env_manager_default_manip_size));
		m->visible(false);
		manips.push(m);
		m->callback(manipCallback,this);
		manip_group->add(m);
	}
}

void EnvBuilder::select( EnvManipulator* manip )
{
	if(manip!=_selected_manip)
	{
		if(_selected_manip!=0)
		{
			_selected_manip->setColor(GsColor::blue);
		}
		_selected_manip = manip;
		_selected_manip->setColor(GsColor::red);
		_clicked = true;
	}
}

void EnvBuilder::makeEnv()
{
	manager->loadScene("Empty");
	manager->setP(human_manager_scene,"Editing");
	for(int i=0;i<manips.size();i++)
	{
		EnvManipulator* manip = manips[i];

		GsVec p = manip->globalPosition();
		GsVec s = manip->size();
		bool active = manip->dynamic;
		ODEObject* b = (ODEObject*)manager->addBox(p,s,active);
		if(manip->name()=="Manip")
		{
			GsString newName = "Box_";
			newName<<randomString(10000);
			b->setName(newName);
		}
		else
			b->setName(manip->name());
		
		b->setP(ode_unique_properties,manip->uniqueProperties);
		b->setP(ode_dynamic,manip->dynamic);
		b->setP(ode_object_color,manip->color);
		if(manip->uniqueProperties)
		{
			b->setP(ode_friction,manip->friction);
			b->setP(ode_bounce,manip->bounce);
			b->setP(ode_density,manip->density);	
		}
		else //use the global env properties
		{
			b->setP(ode_friction,pFloat(env_manager_friction));
			b->setP(ode_bounce,pFloat(env_manager_bounce));
			b->setP(ode_density,pFloat(env_manager_density));
		}
		
		manips[i]->visible(false);
	}
}

void EnvBuilder::set_size( GsVec sze )
{
	if(_selected_manip)
	{
		_selected_manip->setSize(sze);
		_selected_manip->setColor(GsColor::red);
	}
}

void EnvBuilder::init( HumanManager* mgr )
{
	manager = mgr;
}
#include "ode_box.h"
void EnvBuilder::editEnv()
{
	
	manip_group->remove_all();

	for(int i=0;i<manips.size();i++)
	{
		 manips[i]->unref();
	}
	manips.size(0);
	_selected_manip = 0;

	for (int i=0;i<manager->numObjects();i++)
	{
		ODEObject* obj = manager->getObject(i);
		if(obj->isGround())
		{
			if(obj->type() == "Box")
			{
				ODEBox* box = (ODEBox*)obj;
				EnvManipulator* m = new EnvManipulator(box->getPosition(),box->dimension());
				m->setName(box->name());
				m->uniqueProperties = box->pBool(ode_unique_properties);
				if(box->pBool(ode_unique_properties))
				{
					m->density = box->pFloat(ode_density);
					m->bounce = box->pFloat(ode_bounce);
					m->friction = box->pFloat(ode_friction);
				}
				else
				{
					m->density = pFloat(env_manager_density);
					m->bounce = pFloat(env_manager_bounce);
					m->friction = pFloat(env_manager_friction);
				}
				m->dynamic = box->isDynamic();
				m->color = box->pColor(ode_object_color);
				
				m->visible(true);
				manips.push(m);
				m->callback(manipCallback,this);
				manip_group->add(m);
				select(m);
			}
		}

	}

	manager->loadScene("Empty");
}

void EnvBuilder::duplicateBlock()
{
	if(_selected_manip)
	{
		EnvManipulator* m = new EnvManipulator(_selected_manip->globalPosition(),_selected_manip->size());
		m->density = _selected_manip->density;
		m->bounce = _selected_manip->bounce;
		m->friction = _selected_manip->friction;
		m->color = _selected_manip->color;
		m->setColor(_selected_manip->color);
		m->visible(true);
		manips.push(m);
		m->callback(manipCallback,this);
		manip_group->add(m);
		select(m);
	}
}

void EnvBuilder::addBlock()
{
	EnvManipulator* m = new EnvManipulator(GsVec(0,1,1),pVec(env_manager_default_manip_size));
	m->visible(true);
	manips.push(m);
	m->density = pFloat(env_manager_density);
	m->bounce = pFloat(env_manager_bounce);
	m->friction = pFloat(env_manager_friction);
	m->callback(manipCallback,this);
	manip_group->add(m);
}



EnvManipulator::EnvManipulator(GsVec pos,GsVec size):Manipulator(pos,size)
{
	bounce = 0;
	friction = 30;
	color = GsColor::blue;
	dynamic =true;
	density = 1;
	uniqueProperties = false;
}
