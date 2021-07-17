#include "ph_char_builder.h"
#include "ode_world.h"
#include "ph_manager.h"
#include "ode_object.h"
#include "ph_human.h"
#include "ph_joint.h"
#include "ode_box.h"
#include "ph_mod_ref.h"

static void manipCallback ( SnManipulator* mnp,const GsEvent& ev, void* udata )
{
	CharBuilder* mc = ((CharBuilder*)udata);
	CharManipulator* manip = (CharManipulator*)mnp;
	
	manip->setColor(GsColor::red);
	manip->evaluate();

	if(ev.type == GsEvent::Release)
	{
		gsout<<"release: ";
		mc->release();
	}
	else if(ev.type == GsEvent::Drag)
	{

		//gsout<<"drag: ";
		mc->moveJoint(manip);

	}
	else if(ev.type == GsEvent::Push)
	{
		mc->startPos = manip->globalPosition();
		//gsout<<"push: ";
		mc->select(manip);
	}

	
	
}

CharManipulator::CharManipulator(GsVec pos,GsVec size,const GsString&  name, bool isJnt):Manipulator(pos,size)
{
	jointName = name;
	isJoint = isJnt;
	bounce = 0;
	friction = 30;
	color = GsColor::blue;
	dynamic =true;
	density = 1;
	uniqueProperties = false;
	parent = 0;
}


void CharBuilder::translatHeirarchy(CharManipulator* manip)
{
		for (int i = 0;i<manip->children.size();i++)
		{
			CharManipulator* child = manip->children[i];
			//gsout<<"translateHeirarchy "<<child->jointName<<gsnl;
			child->translation(manip->globalPosition()+ child->localOrientation().apply(child->offset));
			translatHeirarchy(child);
		}
}
GsString complementName(const GsString&  nme)
{
	GsString name = nme;
	 if(name.search("Left")!=-1)
	{
		name.replace("Left","Right");
	}
	else if(name.search("Right")!=-1)
	{
		name.replace("Right","Left");
	}
	else
	{
		gsout<<"no complement for "<<name<<gsnl;
	}
	 return name;
}
void CharBuilder::moveJoint(CharManipulator* manip)
{
//gsout<<"moving joint "<<manip->jointName<<" to position "<<manip->localPosition()<<gsnl;
		if(orientParent && manip->isJoint && manip->parent)
		{
			CharManipulator* parentBodyManip = manip->parent;
			CharManipulator* previousJoint = manip->parent->parent;

			GsVec newDis = manip->localPosition() - previousJoint->localPosition();

			GsVec newPos = previousJoint->localPosition() + newDis*0.5;

			parentBodyManip->translation(newPos);
			parentBodyManip->offset = newPos - previousJoint->localPosition();
			
			manip->parent->setSize(GsVec(0.06f,newDis.len(),0.06f));
			newDis.normalize();
			GsQuat q;
			q.set(GsVec(0,1,0),newDis);
			//newPos.normalize();
			//q = decomposeRotation(q,newPos);
	
			manip->parent->rotation(q); //manip->parent->getStartOrientation()*

		}
		if(effectHeirarchy)
		{
			if(manip->parent)
				manip->offset = manip->localPosition() - manip->parent->localPosition();
			else
				manip->offset = manip->localPosition();

			translatHeirarchy(manip);
		}
		if(mirrorChanges)
		{

			GsString comp = complementName(manip->jointName);
			if(comp != manip->jointName)
			{
			
				CharManipulator* compManip = getManip(comp,manip->isJoint);
				GsVec mirrorP = manip->globalPosition();
				mirrorP.x = -mirrorP.x;
				compManip->translation(mirrorP);
				
				if(compManip->isJoint)
				{
					GsVec newDis = compManip->localPosition() - compManip->parent->parent->localPosition();
					GsVec startDis = compManip->getStartPosition() - compManip->parent->parent->getStartPosition();
					GsVec dis = compManip->offset;
					dis.normalize();
					GsVec newPos = compManip->parent->parent->localPosition() + newDis*0.5;

					compManip->parent->translation(newPos);
					startDis.normalize();
					GsQuat q;
					q.set(startDis,dis);
					//newPos.normalize();
					//q = decomposeRotation(q,newPos);

					compManip->parent->rotation(compManip->parent->getStartOrientation()*q);

					if(fabs(startDis.y)> fabs(startDis.x))
						compManip->parent->setSize(1,newDis.len());
					else
						compManip->parent->setSize(0,newDis.len());
				}

				if(compManip->parent)
					compManip->offset = mirrorP - compManip->parent->localPosition();

				translatHeirarchy(compManip);

			}
		}
	
}
CharBuilder::CharBuilder(const GsString&  file):Serializable("CharBuilder")
{
	loadParametersFromFile(file);
	CHECK_VEC(char_manager_default_manip_size);
	CHECK_FLOAT(char_manager_friction);
	CHECK_FLOAT(char_manager_bounce);
	CHECK_FLOAT(char_manager_density);

	_clicked = false;
	manip_group = new SnGroup;
	_selected_manip = 0;
	/*
	for(int i=0;i<NUM_START_MANIPS;i++)
	{
		CharManipulator* m = new CharManipulator(GsVec(0,1,1+i),pVec(char_manager_default_manip_size));
		m->visible(false);
		manips.push(m);
		m->callback(manipCallback,this);
		manip_group->add(m);
	}*/
}

void CharBuilder::select( CharManipulator* manip )
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

void CharBuilder::makeEnv()
{
	manager->loadScene("Empty");
	manager->setP(human_manager_scene,"Editing");
	for(int i=0;i<manips.size();i++)
	{
		CharManipulator* manip = manips[i];

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
			b->setP(ode_friction,pFloat(char_manager_friction));
			b->setP(ode_bounce,pFloat(char_manager_bounce));
			b->setP(ode_density,pFloat(char_manager_density));
		}
		
		manips[i]->visible(false);
	}
}

void CharBuilder::set_size( GsVec sze )
{
	if(_selected_manip)
	{
		_selected_manip->setSize(sze);
		_selected_manip->setColor(GsColor::red);
	}
}

void CharBuilder::init( HumanManager* mgr )
{
	manager = mgr;
	effectHeirarchy = true;
	mirrorChanges = true;
	orientParent = true;
}


void CharBuilder::duplicateBlock()
{
	if(_selected_manip)
	{
	/*	CharManipulator* m = new CharManipulator(_selected_manip->globalPosition(),_selected_manip->size());
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
		*/
	}
}

void CharBuilder::addBlock()
{
	/*
	CharManipulator* m = new CharManipulator(GsVec(0,1,1),pVec(char_manager_default_manip_size));
	m->visible(true);
	manips.push(m);
	m->density = pFloat(char_manager_density);
	m->bounce = pFloat(char_manager_bounce);
	m->friction = pFloat(char_manager_friction);
	m->callback(manipCallback,this);
	manip_group->add(m);
	*/
}
CharManipulator* CharBuilder::getManip(const GsString&  name,bool isJoint)
{

	for (int i=0;i<manips.size();i++)
	{
		if(manips[i]->jointName == name && manips[i]->isJoint == isJoint)
		{

			return manips[i];
		}
	}

	gsout<<"couldn't find manip "<<name<<" which isJoint "<<isJoint<<" in the "<<manips.size()<<" joints:"<<gsnl;
	for (int i=0;i<manips.size();i++)
	{
		gsout<<manips[i]->jointName<<"  isJoint:"<<manips[i]->isJoint<<gsnl;
	}
	return 0;
}
void CharBuilder::editCharacter( PhysicalHuman* hm )
{
	manip_group->visible(true);
	human = hm;

	//CharManipulator* lastBone = 0;
	for (int i=0;i<human->numJoints();i++)
	{
		PhysicalJoint* joint = human->joint(i);
		GsVec jointP = joint->getAnchor();
		GsVec bodyPos = jointP + joint->getBoxOffset(); // joint->getBody()->getPosition();
		GsVec size = joint->getBody()->pVec(ode_box_dim);
		

		

		CharManipulator* jointManip = new CharManipulator(jointP,GsVec(0.1,0.1,0.1),joint->name(), true);
		
	//	gsout<<"making joint manip "<<joint->name()<<gsnl;

		jointManip->visible(true);
		jointManip->setColor(GsColor::green);
		manips.push(jointManip);
		jointManip->callback(manipCallback,this);
		manip_group->add(jointManip);
		if(joint->getParent())
		{
			jointManip->parent = getManip(joint->getParent()->name(),false);
			jointManip->parent->children.push(jointManip);
			jointManip->offset = bodyPos - joint->getBoxOffset() - jointManip->parent->localPosition();
		}
		
		CharManipulator* bodyManip = new CharManipulator(bodyPos,size,joint->name());
		
		bodyManip->visible(true);
		manips.push(bodyManip);
		bodyManip->density = pFloat(char_manager_density);
		bodyManip->bounce = pFloat(char_manager_bounce);
		bodyManip->friction = pFloat(char_manager_friction);
		bodyManip->callback(manipCallback,this);
		manip_group->add(bodyManip);
		bodyManip->parent = jointManip;
		jointManip->children.push(bodyManip);
		bodyManip->offset = bodyPos - jointManip->localPosition();

	}
	gsout<<gsnl;
}

void CharBuilder::createCharacter()
{
	for (int i=0;i<manips.size();i++)
	{
		CharManipulator* manip = manips[i];
		PhysicalJoint* j = human->joint(manip->jointName);
		if(manip->isJoint)
		{
			gsout<<"joint "<<manip->jointName<<" anchor point: "<<manip->localPosition()<<gsnl;
			j->setP(joint_anchor_point,manip->localPosition());
			if(manip->parent && manip->parent->parent)
				human->skref()->joint(manip->jointName)->offset(manip->localPosition() - manip->parent->parent->localPosition());
		
			
		}
		else
		{
			CharManipulator* jointManip = getManip(manip->jointName,true);

			
			if(j)
			{
				
				GsVec offset =   manip->localPosition() - jointManip->localPosition();   

				j->setP(joint_box_offset,offset);
				j->setP(joint_box_dim,manip->size());
				
				j->setP(joint_box_orientation,manip->localOrientation());
				j->getBody()->setOrientation(manip->localOrientation());
				
			
			}
			else
			{
				//need to create new joint
			}
		}
	}
	human->skref()->update_global_matrices();
	human->reference_module()->getKnScene()->rebuild();
	human->makeJoints();
	
	manip_group->visible(false);
	for (int i=0;i<manips.size();i++)
	{
		delete manips[i];
	}
	manips.size(0);
	manip_group->remove_all();
	_selected_manip = 0;

}

void CharBuilder::release()
{
	//gsout<<"manips: "<<gsnl;
	for (int i=0;i<manips.size();i++)
	{
		//manips[i]->setInitial();
		//gsout<<manips[i]->jointName<<"  isJoint:"<<manips[i]->isJoint<<gsnl;
	}
}



