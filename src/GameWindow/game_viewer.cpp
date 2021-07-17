#pragma once
# include <gsim/fl.h>
#include <gsim/sn_group.h>
# include "game_viewer.h"
# include "game_window.h"

#include "util_serializable.h"

GameFLViewer::GameFLViewer ( int x, int y, int w, int h, const char *l ) : FlViewer ( x, y, w, h, l ),Serializable("GameFLViewer")
 {
	zoomfactor(0.04f);
	_root = new SnGroup;
	view_all();
	// FlViewer::cmd ( FlViewer::CmdPlanar );
	FlViewer::root ( _root );
 }

void GameFLViewer::applyParameters()
{
	camera().init();
	camera().fovy = pFloat(camera_fovy);
	camera().eye	=  pVec(camera_eye);
	camera().center	=  pVec(camera_center);
	camera().up =  pVec(camera_up);
	camera().znear = pFloat(camera_znear);
	camera().zfar = pFloat(camera_zfar);
	camera().aspect = pFloat(camera_aspect);
	camera().scale = pFloat(camera_scale);
	camera().rotation = pQuat(camera_rotation);
	camera().translation = pVec(camera_translation);
	redraw();
}
GameFLViewer::~GameFLViewer ()
 {

 }

void GameFLViewer::init ( GameWindow* win ,GsString file)
 {
	loadParametersFromFile(file);

	CHECK_VEC(camera_eye);
	CHECK_VEC(camera_center);
	CHECK_VEC(camera_up);
	CHECK_FLOAT(camera_fovy);
	CHECK_FLOAT(camera_znear);
	CHECK_FLOAT(camera_zfar);
	CHECK_FLOAT(camera_aspect);
	CHECK_FLOAT(camera_scale);
	CHECK_QUAT(camera_rotation);
	CHECK_VEC(camera_translation);
	CHECK_BOOL(camera_follow);
	
   _mainwin = win;

   applyParameters();
   
 }

void GameFLViewer::draw ()
 {
	
	
		setP(camera_fovy,camera().fovy);
		setP(camera_eye,camera().eye);
		setP(camera_center,camera().center);
		setP(camera_scale,camera().scale);
		setP(camera_up,camera().up);
		setP(camera_znear,camera().znear);
		setP(camera_zfar,camera().zfar);
		setP(camera_aspect,camera().aspect);
		setP(camera_rotation,camera().rotation);
		if(pBool(camera_follow))
		{

		}
		else
		{
			setP(camera_translation,camera().translation);
		}

   FlViewer::draw ();
 }


int GameFLViewer::handle_scene_event ( const GsEvent &e )
 {
	_mainwin->handle_viewer_event(e);
   // now let the viewer process the remaining events:
   return FlViewer::handle_scene_event ( e );
 }

