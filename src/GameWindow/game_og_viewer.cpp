
#include "game_og_viewer.h"
#include "game_window.h"
using namespace OgGSim;

GameViewer::GameViewer( const char* cfgPath ,GameWindow* window):Viewer(cfgPath)
{
	_window = window;


//	createAmbientLight(Ogre::ColourValue(0.8,0.8,0.8));
// 	Ogre::Light* ambient = _SceneMgr->createLight("ambientLight");
// 	ambient->setType(Ogre::Light::LT_DIRECTIONAL);
// 	ambient->setPosition( Ogre::Vector3(0,1,3) );
// 	ambient->setDiffuseColour(0.85,0.85,0.85);
// 	ambient->setSpecularColour(0.65,0.65,0.65);
// 	ambient->setDirection(Ogre::Vector3(0,-0.2,1));
// 	ambient->setAttenuation(1500.0f, 1500.0f, 1500.0f, 1500.0f);
// 	ambient->setCastShadows(true);


    /* Shadow initialization
    _SceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE);
    _SceneMgr->setShadowTextureSize(2048);
    _SceneMgr->setShadowFarDistance(500.0);
    _SceneMgr->setShadowColour(Ogre::ColourValue(0.6, 0.6, 0.6));

    Agent  --->>>> getMaterialList()



    Ogre::MaterialPtr ptr =  Ogre::MaterialManager::getSingleton().getByName();
    ptr->setReceiveShadows();

    */
}

bool GameViewer::processUnbufferedKeyInput()
{

	Viewer::processUnbufferedKeyInput();
	bool moveScene = true;
	float inc = 0.05f;
	if(_Keyboard->isKeyDown(OIS::KC_SPACE))
	{
		_window->reset();
	}
	if(_Keyboard->isKeyDown(OIS::KC_RETURN))
	{
		_window->play();
	}
	if(_Keyboard->isKeyDown(OIS::KC_1) )
	{
		_window->_jump_select_mode = select_closest_mode;
		phout<<"select closest mode\n";
	}
	if(_Keyboard->isKeyDown(OIS::KC_2) )
	{
		_window->_jump_select_mode = move_env_mode;
		phout<<"move mode\n";
	}
	if(_Keyboard->isKeyDown(OIS::KC_3) )
	{
		_window->_jump_select_mode = interpolate_mode;
		phout<<"interpolate mode\n";
	}
	if(_Keyboard->isKeyDown(OIS::KC_4))
	{
		_window->_jump_select_mode = env_hull_mode;
		phout<<"hull mod\n";
	}
	if(_Keyboard->isKeyDown(OIS::KC_LCONTROL) )
	{
		inc = 0.01f;
	}
	if(_Keyboard->isKeyDown(OIS::KC_NUMPAD4) ) 
	{
		_window->_jumpP.z+=inc;
	}
	else if(_Keyboard->isKeyDown(OIS::KC_NUMPAD6) ) 
	{
		_window->_jumpP.z-=inc;
	}
	else if(_Keyboard->isKeyDown(OIS::KC_NUMPAD8) ) 
	{
		_window->_jumpP.y+=inc;
	}
	else if(_Keyboard->isKeyDown(OIS::KC_NUMPAD2) ) 
	{
		_window->_jumpP.y-=inc;
	}

	else
	{
		moveScene = false;
	}

	if(moveScene)
	{
		_window->makeJump();
	}
	return true;
}
bool GameViewer::update()
{
	OgGSim::Viewer::update();
	Ogre::Vector3 pos = Ogre::Vector3(0.4f,2.0f,0.5f);
	 _HeadLight->setPosition(pos);
	 _HeadLight->setDirection(-pos);
	 return true;
}

void GameViewer::init( bool askconfig )
{
	OgGSim::Viewer::init(askconfig);
	createAmbientLight(Ogre::ColourValue(0.8f,0.8f,0.8f));

	Ogre::Light* ambient = _SceneMgr->createLight("ambientLight");
	ambient->setType(Ogre::Light::LT_DIRECTIONAL);
	ambient->setPosition( Ogre::Vector3(0,1,3) );
	ambient->setDiffuseColour(0.85f,0.85f,0.85f);
	ambient->setSpecularColour(0.65f,0.65f,0.65f);
	ambient->setDirection(Ogre::Vector3(0.0f,-0.2f,1.0f));
	ambient->setAttenuation(1500.0f, 1500.0f, 1500.0f, 1500.0f);
	ambient->setCastShadows(true);
}
