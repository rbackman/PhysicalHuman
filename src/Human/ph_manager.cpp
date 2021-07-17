
#include "ph_manager.h"
#include "ph_joint.h"
#include "ph_human.h"
#include "ph_file_manager.h"
#include "ph_motion_manager.h"
#include "ph_mod_ik.h"
#include "ode_sphere.h"
#include "ode_world.h"
#include "ph_motion_segmenter.h"
#include "ph_state_manager.h"
#include "ph_trajectory_planner.h"
#include "ph_mod_contact.h"
#include "ph_motion.h"
#include "ph_env_builder.h"
#include "ph_char_builder.h"
#include "util_server.h"

//#include "phw_events.h"

HumanManager::HumanManager( const GsString& dir ):Serializable("HumanManager")
{
	_files = new HumanFileManager(dir);

	loadParametersFromFile(_files->getPrefFile());

	MAKE_TEMP_PARM(human_manager_simulation_step,false);
	CHECK_BOOL(human_manager_simulation_running);
	
	CHECK_BOOL(human_manager_graphics_active);
	CHECK_BOOL(human_manager_auto_reset_from_hip_height);
	CHECK_STRING(human_manager_scene);
	_root = new SnGroup; _root->ref();
	_groundObjects = new SnGroup; _groundObjects->ref();
	_root->add(_groundObjects);

	_world = new ODEWorld(_files->getPrefFile());
	env_builder = new EnvBuilder(_files->getPrefFile());
	char_builder = new CharBuilder(_files->getPrefFile());

	_root->add(char_builder->getGroup());
	_root->add(env_builder->getGroup());

	
	_motion_segmenter = new HumanMotionSegmenter(_files->getPrefFile());


	_planner = new TrajectoryPlanner(_files->getPrefFile());

	envGrp = 0;
    _oversample_counter = 0;
    _frame_count = 0;
	_selectedCharacter = -1;
	_message_count = 0;
	_waiting_to_stop = false;
	client = 0;

}
void HumanManager::init()
{
	env_builder->init(this);
	char_builder->init(this);
	
	_motion_segmenter->init(this);
	
	
	
	_planner->init(this);
	message("ManagerReady");
	 envOffY = -0.0575f;
	envOffZ = 0.0215f;
}
Serializable* HumanManager::findSerializable( const GsString& name,const GsString& type )
{

	Serializable* s = 0;
	int count = 0;
	for(int i=0;i<_serializables.size();i++)
	{
		if(_serializables.get(i)->name()==name)
			{
				if(s==0)
				{
					count++;
						s = _serializables.get(i);
				}

				if(_serializables.get(i)->type()==type)
				{
					
					return _serializables.get(i);
				}
				
			}

	}
	if(s)
	{
		phout<<"found the name: "<<name<<"  but not the type:" <<type<<" instead it has type:"<<s->type()<<"  there were "<<count<<" objects with the name\n";
		
		//return s;
		return 0;
	}
	else
	{	
		GsString m ="HumanManager::findSerializable() couldn't find serializable ";
		m<<name<<" type "<<type<<" out of "<< _serializables.size()<<" serializables "<<gsnl;
		message(m);
	}
	return 0;
}
PhysicalHuman* HumanManager::selectedCharacter()
{
	if(_selectedCharacter!=-1 && _selectedCharacter < _characters.size())
		return _characters[_selectedCharacter];
	else
		return 0;
}
void HumanManager::pushCharacter( PhysicalHuman* guy )
{
	_characters.push(guy);
	_selectedCharacter = _characters.size()-1;
	_serializables.push(guy->getSerializables());
	_root->add(guy->getGroup());
}

void HumanManager::step()
{
	setP(human_manager_simulation_step,true);
}

void HumanManager::setRunning( bool val )
{
	setP(human_manager_simulation_running,val);
	
}

Serializable* HumanManager::checkRay(const GsLine& ray )
{
	PhysicalHuman* human = selectedCharacter();

	if(!human) return 0;

	phout<<" HumanManager::checkRay( GsLine ray )  only checks joints for now\n";

		float a,b,c;
		for(int i=0;i<human->numJoints();i++)
		{
			PhysicalJoint* joint = human->joint(i);
			GsModel* mod = joint->getBody()->getModel()->getModel();
			for(int j=0;j<mod->F.size();j++)
			{
				GsModel::Face f = mod->F.get(j);

				GsVec va = mod->V[f.a];
				GsVec vb = mod->V[f.b];
				GsVec vc = mod->V[f.c];
				GsMat m =  joint->getBody()->getModel()->getTfm()->get();

				va = va*m;
				vb = vb*m;
				vc = vc*m;

				if(ray.intersects_triangle(va,vb,vc,a,b,c))
				{
					return joint;
				}

			}

		}
		message("ray cast didn't hit anything");

	return 0;
}

bool HumanManager::isRunning()
{
	return pBool(human_manager_simulation_running);
}


void HumanManager::dropBalls()
{
	PhysicalHuman* human = selectedCharacter();
	if(!human)return;

	GsVec pp = human->getCOM();

	for(float i=-2;i<2;i++)
	{
		for(float j=1;j<8;j++)
		{
			GsVec rp = GsVec(pp.x - 0.1f + i*0.2f,1.0f + j*0.5f, pp.z + gs_random()*0.2f-0.1f);
			ODESphere*	sphere = new ODESphere(_world,true,rp,0.05f);
			_objects.push(sphere);
			_groundObjects->add(sphere->getModel()->getGrp());
		}
	}
}
void HumanManager::loadJumpScene(float z,float y)
{
	for(int i=0;i<_objects.size();i++)
	{
		delete _objects.get(i);
	}
	_objects.remove(0,_objects.size());

	_groundObjects->remove_all();

	ODEBox* b = new ODEBox(_world, false, GsVec(0.0f,-0.1f,0.05f), GsVec(0.5f, 0.05f , 0.6f)  );
	b->setP(ode_is_ground,true);
	b->setColor(GsColor(0.5f,0.5f,0.5f));
	_groundObjects->add(b->getModel()->getGrp());
	_objects.push(b);
	
	b = new ODEBox(_world, false, 
			GsVec(0.0f,y-0.1f,z+0.05f),
			GsVec(0.5f, 0.05f , 0.5f)
			);
		b->setP(ode_is_ground,true);
		b->setColor(GsColor(0.0f,0.5f,0.0f));
		_groundObjects->add(b->getModel()->getGrp());
		_objects.push(b);
}
void HumanManager::toggleEnv()
{
	if(envGrp==0)
	{
		envGrp = new SnGroup;
		for (int i=0;i<getMotionManager()->currentController()->numMotions();i++)
		{

			float jz = getMotionManager()->currentController()->getMotion(i)->pFloat(motion_descriptor_env,0);
			float jy = getMotionManager()->currentController()->getMotion(i)->pFloat(motion_descriptor_env,1);
			GsVec endP = selectedCharacter()->contact_module()->stancePoint();
			endP += GsVec(0.0f,envOffY,envOffZ);
			endP += GsVec(0.0f,jy,jz);
	
			Box* b = new Box( endP,GsVec(0.5f, 0.05f , 0.6f) );

			b->setColor(GsColor(0.0f,0.5f,0.0f));
			envGrp->add(b->getGrp());
		}
		getRoot()->add(envGrp);
	}
	else
	{
		envGrp->visible(!envGrp->visible());

	}

}
ODEBox* HumanManager::addBox(GsVec p,GsVec s,bool dynamic)
{
	ODEBox* b = new ODEBox(_world, dynamic, p, s);
	b->setP(ode_is_ground,true);
	_objects.push(b);
	_groundObjects->add(b->getModel()->getGrp());
	return b;
}
void HumanManager::loadScene( const GsString& sceneName)
{
	setP(human_manager_scene,sceneName);
	_groundObjects->remove_all();
	for(int i=0;i<_objects.size();i++)
	{
		//gsout<<"obj "<<i<<" is "<<(int)(_objects.get(i))<<gsnl;
		delete _objects.get(i);
	}
	_objects.size(0);
		
	
	
	
	GsString scene_file = getFiles()->getSceneFile(sceneName);
	Serializable* s = new Serializable("Scene");
	if(s->loadParametersFromFile(scene_file))
	{
		//phout<<"found file "<<scene_file<<gsnl;
		GsStrings* str = &((StringParameter*)s->getParameter("scene_object_list"))->val;

		for (int i=0;i<str->size();i++)
		{
			ODEBox* b = new ODEBox(_world,str->get(i),scene_file);
		
			b->setP(ode_is_ground,true);
			if(b->pBool(ode_dynamic))
			{
				dBodySetAngularDamping(b->getBodyID(),0.0001f); 
				dBodySetAngularDampingThreshold(b->getBodyID(),0.5);
			}
			_objects.push(b);
			SnGroup* g = b->getModel()->getGrp();
			_groundObjects->add(g);
		}
		delete s;
	}
	else
	{
			delete s;
		 if(GsString::compare(sceneName,"RandomPillars")==0)
		{
			ODEBox* b = new ODEBox(_world, false, GsVec(0.0f,-0.1f,0.0f), GsVec(0.6f, 0.05f , 0.6f) );
			b->setP(ode_is_ground,true);
			b->setColor(GsColor(0.5f,0.5f,0.5f));
			_groundObjects->add(b->getModel()->getGrp());
			_objects.push(b);

			int numPillars = 15;
			for(int i=1;i<numPillars;i++)
			{
				b = new ODEBox(_world, false, 
					GsVec(0.0f,-0.1f + gs_random(-0.2f,0.2f),i*1.0f + ( (float)i/1.2f)*gs_random(-0.2f,0.2f)),
					 GsVec(0.6f, 0.05f , 0.6f)
					);
				b->setP(ode_is_ground,true);
				b->setColor(GsColor(0.5f,0.5f,0.5f));
				_groundObjects->add(b->getModel()->getGrp());
				_objects.push(b);
			}
		}
		else if( GsString::compare(sceneName,"RandomJump")==0)
		{
			ODEBox* b = new ODEBox(_world, false, GsVec(0.0f,-0.1f,0.05f), GsVec(0.5f, 0.05f , 0.9f)  );
			b->setP(ode_is_ground,true);
			b->setColor(GsColor(0.5f,0.5f,0.5f));
			_groundObjects->add(b->getModel()->getGrp());
			_objects.push(b);
			int numP = 2;
			for(int i=1;i<numP;i++)
			{
				b = new ODEBox(_world, false, 
					GsVec(0.0f,-0.1f + gs_random(-0.5f,0.5f),0.2f+i*1.0f + ( (float)i/1.2f)*gs_random(-0.4f,0.4f)),
					GsVec(0.5f, 0.05f , 0.5f)
					);
				b->setP(ode_is_ground,true);
				b->setColor(GsColor(0.5f,0.5f,0.5f));
				_groundObjects->add(b->getModel()->getGrp());
				_objects.push(b);
			}
		}
		else if( GsString::compare(sceneName,"Platform")==0)
		{
			ODEBox* b = new ODEBox(_world, false, GsVec(0.0f,-0.1f,0.05f), GsVec(0.5f, 0.05f , 0.9f)  );
			b->setP(ode_is_ground,true);
			b->setColor(GsColor(0.5f,0.5f,0.5f));
			_groundObjects->add(b->getModel()->getGrp());
			_objects.push(b);
		
		}
		else if( GsString::compare(sceneName,"MotionJump")==0)
		{
	
			GsVec startP;
			if(getMotionManager()->pBool(human_motion_manager_relative_jump))
				startP = selectedCharacter()->contact_module()->stancePoint();
			else
				startP = getStateManager()->getStancePointStart();

			startP += GsVec(0.0f,envOffY,envOffZ);
		
			//phout<<" after offset "<<startP<<gsnl;
			ODEBox* b = new ODEBox(_world, false, startP, GsVec(0.5f, 0.05f , 0.6f)  );
			b->setP(ode_is_ground,true);
			b->setColor(GsColor(0.5f,0.5f,0.5f));
			SnGroup* g = b->getModel()->getGrp();
			
			_groundObjects->add(g);
		
			_objects.push(b);
		
		
			GsVec jumpTo;
			jumpTo.z = 1.0f;

			if(getMotionManager()->currentMotion())
			{
			
				jumpTo = getMotionManager()->currentMotion()->getEnv();

			}
			else
			{
				phout<<"controller not loaded\n";
			}
		
		
			GsVec endP = startP; //selectedCharacter()->contact_module()->stancePoint();
			//endP += GsVec(0,envOffY,envOffZ);
			endP += jumpTo;
		
		//	phout<<"jumpTo "<<jumpTo<<gsnl<<gsnl;

			//phout<<"stance point start "<<getStateManager()->getStancePointStart()<<gsnl;
			b=0;
			b = new ODEBox(_world, false, endP, GsVec(0.5f, 0.05f , 0.6f)  );
			b->setP(ode_is_ground,true);
			b->setColor(GsColor(0.5f,0.5f,0.5f));
			_groundObjects->add(b->getModel()->getGrp());
	
			_objects.push(b);
		
	// 		for (int i=0;i<_envObjects.size();i++)
	// 		{
	// 			delete _envObjects[i];
	// 		}
	//		_envObjects.size(0);

	// 		GsString modelFile = getFiles()->getModelDirectory();
	// 		modelFile<<"platform.obj";
	// 		OBJModel* m = new OBJModel(modelFile);
	// 		m->setPosition(startP);
	// 		_envObjects.push(m);
	// 
	// 		m = new OBJModel(modelFile);
	// 		m->setPosition(endP);
	// 		_envObjects.push(m);

		}
		else if(GsString::compare(sceneName,"Ground")==0)
		{
			//do nothing for empty scene
		}
		else if(GsString::compare(sceneName,"Empty")==0)
		{
			//do nothing for empty scene
		}
		else
		{
			phout<<"error couldn't load scene "<<sceneName<<gsnl;
		}
	}
}

void HumanManager::loadSkeleton( const GsString& filename ) /*"../data/Mentar.s" */
{
	KnScene* _kns = new KnScene;
	KnSkeleton* sk=new KnSkeleton;
	bool ok = sk->load(filename);
	if ( !ok ) phout.fatal("Could not load skeleton file!");
	sk->root()->pos()->value(0,1.5,0);
	sk->update_global_matrices();
	_root->add(_kns);
	_kns->connect(sk);
	_kns->set_visibility(1,0,0,0);
	_kns->set_skeleton_radius(0.5f);
}

bool HumanManager::update()
{
	bool ret = false;
	if(client)
	{
		marker_position p;
		if(client->update(&p))
		{
			phout<<"got marker "<<p.pos<<gsnl;
		}

	}
		if(pBool(human_manager_simulation_running) || pBool(human_manager_simulation_step) )
		{
			_oversample_counter--;
			_animation_step = false;
			if(_oversample_counter <0)
			{
				_animation_step = true;
				_oversample_counter = _world->pInt(world_simulation_over_sample);
			}

			//this keeps the scene from rendering every frame so for every one frame rendered it will 
			//run world->pInt(world_simulation_over_sample) extra dynamics timesteps 
			
			if(animationStep())
			{
				if(pBool(human_manager_auto_reset_from_hip_height))
				{
					for (int i=0;i<_characters.size();i++)
					{
						if(!_characters[i]->isStanding())
						_characters[i]->getMotionManager()->resetAnimation();
					}
					
				}
				//signal to redraw graphics
				for (int i=0;i<_characters.size();i++)
				{
					_characters[i]->getMotionManager()->update(getAnimationTimeStep());
				}
	// 				if(_waiting_to_stop)
	// 				{
	// 					if(!_motion_manager->running())
	// 					{
	// 						stopReactiveWalk();
	// 						_waiting_to_stop = false;
	// 					}
	// 				}
				if(pBool(human_manager_graphics_active))
				{
					for (int i=0;i<_characters.size();i++)
					{
						_characters[i]->redraw(); 
					}

				}

				if(pBool(human_manager_simulation_step))
				{
					setP(human_manager_simulation_step,false);	
					for (int i=0;i<_characters.size();i++)
					{
						_characters[i]->getMotionManager()->setP(human_motion_manager_playing,false);
					}
				}
				ret = true;
			}
			
			for (int i=0;i<_characters.size();i++)
			{
				_characters[i]->update();
			}

			for(int i=0;i<_objects.size();i++)
			{
				_objects.get(i)->update(animationStep());
			}

			_world->ODELoop();

			if(_planner->update())
				ret = true;
		}
// 		else
// 		{				
// 			_motion_manager->update(getAnimationTimeStep());
// 			for (int i=0;i<_characters.size();i++)
// 			{
// 				_characters[i]->ik_module()->evaluate();
// 				
// 				if(pBool(human_manager_graphics_active))
// 					_characters[i]->redraw(); 
// 			}
// 		}

		
			

		return ret;
}




HumanManager::~HumanManager()
{
	_groundObjects->remove_all(); _groundObjects->unref();
	_root->remove_all(); _root->unref();
	delete _world; 
	delete _files;
	
	for (int i=0;i<_characters.size();i++)
	{
		delete _characters[i];
	}
	_characters.size(0);
	for (int i=0;i<_objects.size();i++)
	{
		delete _objects[i];
	}
	_objects.size(0);
	_serializables.size(0);
}

HumanFileManager* HumanManager::getFiles()
{
	return _files;
}

HumanMotionManager* HumanManager::getMotionManager()
{
	if(selectedCharacter())
		return selectedCharacter()->getMotionManager();
	return 0;
}

ODEWorld* HumanManager::getWorld()
{
	return _world;
}
HumanMotionSegmenter* HumanManager::getMotionSegmenter()
{
	return _motion_segmenter;
}
void HumanManager::listCommands()
{
	phout<<"\n------HumanManager------"<<gsnl;
	phout<<"precede all commands to manager with 'human'"<<gsnl;
	phout<<"hello:		say hi to dynoman"<<gsnl;

}
GsString HumanManager::processCmd(const GsString& cm )
{
	GsString response;
	GsString cmd = cm;
	cmd.trim();
	if(cmd == "hello")
	{
		response<<"hey how is it going ?";
	}
	else if(cmd == "dance")
	{
		response<<"teach me to dance then..";
	}
	else
	{
		//response<<"human got "<<cmd<<gsnl;
	}
	return response;
}
#include "ph_motion.h"
void HumanManager::resetState()
{
	getFiles()->saveConfigurationNamed(selectedCharacter(),"current");
	for (int i=0;i<_characters.size();i++)
	{
		_characters[i]->getStateManager()->reset();
	}
	
}

bool HumanManager::animationStep()
{
	return _animation_step;
}

float HumanManager::getAnimationTimeStep()
{
	return getWorld()->getAnimationStep();
}


bool HumanManager::hasMessage()
{
	if(_has_message)
	{
		_has_message = false;
		return true;
	}
	return false;
}

GsString HumanManager::getMessage()
{
	if(_message_count>0)
	{
		_message_count = 0;
		return _last_message;
	}
	return GsString("no message");
}
void HumanManager::message(const GsString& m,const GsString& s)
{
	GsString ms = m;
	ms<<s;
	message(ms);
}
void HumanManager::message(const GsString& m,float s)
{
	GsString ms = m;
	ms<<s;
	message(ms);
}
void HumanManager::message(const GsString& mg)
{
	GsString m = mg;
	if(m.search("cmd")==0)
	{
		m.substring(4,m.len()-1);
		phout<<"trying to process command "<<m<<gsnl;
		m = processCmd(m);
	}
	_message_count++;
	if(_message_count>1)
	{
	//	phout<<"overwriting message "<<_last_message<<gsnl;
		_last_message<<" : "<<m;
	}
	else
	{
		_last_message = m;
	}
	_has_message = true;

}

int HumanManager::numObjects()
{
	return _objects.size();
}

ODEObject* HumanManager::getObject( int i )
{
	return _objects[i];
}

void HumanManager::renameMotion( int i )
{
	Controller* current_controller = getMotionManager()->currentController();
	HumanMotion* m = current_controller->getMotion(i);
	GsString newName; //=current_controller->pString(controller_name);
	int numParms = current_controller->sizeOfParameter(controller_parameters);
	for (int i=0;i<numParms;i++)
	{
		newName<<abs((int)(100*m->pFloat(motion_descriptor_avg,i)));
		if(i<numParms-1)
			newName<<"_";
	}
	GsString oldName = current_controller->getDirectoryName();
	oldName<<m->getMotionName()<<".motion";
	getFiles()->deleteFile(oldName);
	GsString newFileName = current_controller->getDirectoryName();
	newFileName<<newName<<".motion";
	m->setMotionName(newName);
	getFiles()->saveMotionTrajectories(newFileName,m);

	phout<<"renamed motion---\n\t old name: "<<oldName<<gsnl;
	phout<<"\t new name:"<<newName<<gsnl;
}

EnvBuilder* HumanManager::getEnvBuilder()
{
	return env_builder;
}
CharBuilder* HumanManager::getCharBuilder()
{
	return char_builder;
}
void HumanManager::startClient()
{
	client = new Client;
	client->connectToServer();
}

void HumanManager::newConfig( const GsString& ans )
{
	PhysicalHuman* human = selectedCharacter();
	if(!human)
	{
		human = new PhysicalHuman(this,"man");
		human->init();
		
	}
	human->getStateManager()->loadInitialState();
	human->setP(human_default_state,"initial");
	getFiles()->saveConfigurationNamed(human,ans);
	getFiles()->saveStateNamed(human,"initial");
	clearCharacters();
	_groundObjects->remove_all();
	for (int i = 0; i < numObjects(); i++)
	{
		delete getObject(i);
	}
	_objects.size(0);
	_world->reset();
	
	human = new PhysicalHuman(this,ans);
	human->init();
	pushCharacter(human);
	human->getStateManager()->loadDefaultState();
	setRunning(false);
	step();

	human->reset();
	human->setP(human_show_collision_geo,true);
	human->setP(human_show_visual_geo,false);
	human->setP(human_show_skeleton,false);
	human->applyParameters();
	
}

PhysicalHuman* HumanManager::getCharacter( const GsString& configName )
{
//	phout<<"trying to find character "<<configName<<gsnl;
	for (int i=0;i<_characters.size();i++)
	{
		if(_characters[i]->characterName()==configName)
			return _characters[i];
	}
	
	return 0;
}

void HumanManager::setCharacter( const GsString& configName )
{
	
	for (int i=0;i<_characters.size();i++)
	{
		if(_characters[i]->characterName()==configName)
		{
			_selectedCharacter = i; 
			showSelectedController();
			return;
		}
	}
	gsout<<"couldn't set character "<<configName<<gsnl;
	_selectedCharacter = -1;
}

void HumanManager::clearCharacters()
{
	for (int i=0;i<_characters.size();i++)
	{
		getRoot()->remove(_characters[i]->getGroup());
		delete _characters[i];
	}
	_characters.size(0);
	_selectedCharacter = -1;
}

HumanStateManager* HumanManager::getStateManager()
{
	if(selectedCharacter())
		return selectedCharacter()->getStateManager();
	return 0;
}

void HumanManager::showSelectedController()
{
	for (int i=0;i<_characters.size();i++)
	{
		if(i==_selectedCharacter)
			_characters[i]->setControllerVis(true);
		else
			_characters[i]->setControllerVis(false);
	}
}

