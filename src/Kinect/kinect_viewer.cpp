
# include <gsim/fl.h>
# include <gsim/fl_vars_win.h>
# include <gsim/fl_skeleton_win.h>
#include <gsim/gs_euler.h>

#include "kinect_marker.h"
# include "kinect_viewer.h"
# include "kinect_main_win.h"
#include "kinect_manager.h"

# define CHKVAR(s) if(_vars->search(s)<0) { fltk::alert("Missing parameter [%s]!",s); exit(0); }

KinectViewer::KinectViewer ( int x, int y, int w, int h, const char *l ) : FlViewer ( x, y, w, h, l )
{
	// we build here an example scene graph for polygon edition
	_root = new SnGroup;
	_markerGroup = new SnGroup;
	_surfGroup= new SnGroup;
	_surfGroup->separator(true);
	_markerGroup->separator(true);
	_markerPoints = new SnPoints;
	_root->add(_markerPoints);
	_root->add(_markerGroup);
	_root->add(_surfGroup);


	FlViewer::root ( _root );
	_meshNum=0;


}
void KinectViewer::setTrans(int k,float x,float y,float z,float rx,float ry,float rz)
{
	if(k < _surfs.size())
	{
		phout<<"transform surf "<<k<<" :"<<GsVec(x,y,z)<<" rot: "<<GsVec(rx,ry,rz)<<gsnl;

		GsMat m;
		GsQuat rot;
		gs_rot(gsXYZ,rot,GS_TORAD(rx),GS_TORAD(ry),GS_TORAD(rz));
		compose ( rot, GsVec(x,y,z), m );
		_surf_trans.get(k)->get().set(m);	

	}
}


KinectViewer::~KinectViewer ()
{

}

void KinectViewer::init ( KinectMainWin* win )
{
	_mainwin = win;
	view_all();
	zoomfactor(0.03f);

}

bool KinectViewer::mdist(int a,int b,int surf)
{
	GsVec d = _surfs.get(surf)->model()->V.get(a) - _surfs.get(surf)->model()->V.get(b);
	return d.len() < _kinectManager->cutoff();
}


void KinectViewer::updatePoints(int surfNum, point3D * points)
{



	if(_kinectManager->pBool(kinect_manager_draw_mesh))
	{
		SnColorSurf* surf = _surfs.get(surfNum);
		for( int y = 0 ; y < DEPTH_SIZE_Y ; y++ )
		{
			for( int x = 0 ; x < DEPTH_SIZE_X ; x++ )
			{
				surf->model()->V.get(element(x,y)) = points[element(x,y)].pos;
				surf->model()->M.get(element(x,y)).diffuse = points[element(x,y)].col;
			}
		}


		surf->model()->F.remove(0,surf->model()->F.size());

		for( int y = 0 ; y < DEPTH_SIZE_Y-1 ; y++ )
		{
			for( int x = 0 ; x < DEPTH_SIZE_X-1 ; x++ )
			{

				GsModel::Face f;
				int ul = element(x,y);
				int ur = element(x+1,y);
				int ll = element(x,y+1);
				int lr = element(x+1,y+1);


				if(_mainwin->ui_all_points->value())
				{
					f.set(ul,ur,ll);

					surf->model()->F.push(f);

					f.set(ur,lr,ll);
					surf->model()->F.push(f);

				}
				else
				{
					KinectManager* k = _kinectManager;
					if(k->getPointActivity(ul) && k->getPointActivity(ll))
					{
						if(k->getPointActivity(ur))
						{
							if(mdist(ul,ur,surfNum) &&  mdist(ur,ll,surfNum)  && mdist(ll,ul,surfNum)  )
							{
								f.set(ul,ur,ll);
								surf->model()->F.push(f);	
							}
						}
						if(k->getPointActivity(lr))
						{
							if(mdist(ul,lr,surfNum)  &&  mdist(lr,ll,surfNum)  && mdist(ll,ur,surfNum))
							{

								f.set(ur,lr,ll);
								surf->model()->F.push(f);

							}
						}
					}
				}
			}
		}
	}


}


int KinectViewer::handle_scene_event ( const GsEvent &e )
{
	// window events can be custom-processed here:
	if ( e.button1 )
	{
	}

	if(e.type == GsEvent::Push)
	{
		GsLine ray = e.ray;

		//  Line* l = new Line(ray.p1,ray.p2);
		//  _fingerGroup->add(l->getGrp());


		float a,b,c;
		for(int i=0;i<_surfs.size();i++)
		{
			GsModel* surf = _surfs.get(i)->model();

			for(int j=0;j<surf->F.size();j++)
			{
				GsModel::Face f = surf->F.get(j);

				if(ray.intersects_triangle(surf->V[f.a],surf->V[f.b],surf->V[f.c],a,b,c))
				{
					/*  GsColor cl = GsColor(
					(surf->M[f.a].diffuse.r + surf->M[f.b].diffuse.r+surf->M[f.c].diffuse.r)/3.0f,
					(surf->M[f.a].diffuse.g + surf->M[f.b].diffuse.g+surf->M[f.c].diffuse.g)/3.0f,
					(surf->M[f.a].diffuse.b + surf->M[f.b].diffuse.b+surf->M[f.c].diffuse.b)/3.0f
					);*/

					GsVec pos = surf->V[f.a]; //*a + surf->V[f.b]*b + surf->V[f.c]*c ;
					GsColor cl = surf->M[f.a].diffuse;
					//  Ball* b = new Ball(pos,0.2f,cl);

					// _mainwin->ui_viewer->_fingerGroup->add(b->getGrp());

					phout<<"color "<< cl <<" pos "<< pos<<gsnl;
					
					if(_mainwin->ui_make_finger->value())
					{
						_mainwin->ui_make_finger->value(0);
						GsString mname = _mainwin->ui_marker_type->child(_kinectManager->numMarkers())->label();
						_kinectManager->makeMarker(mname,cl,pos);
						_mainwin->updateUI();
						_kinectManager->getCurrentMarker()->setP(blob_type,mname);
					}
					else if(_mainwin->ui_set_clip_box->value())
					{
						_mainwin->ui_set_clip_box->value(false);
						_kinectManager->setClipManip(pos);
					}

				}

			}

		}
	}
	// now let the viewer process the remaining events:
	return FlViewer::handle_scene_event ( e );
}

void KinectViewer::setKinectManager( KinectManager* k )
{
	_kinectManager = k;

	for(int i=0;i<k->getNumKinects();i++)
	{

		GsModel* model = new GsModel;

		for( int y = 0 ; y < DEPTH_SIZE_Y ; y++ )
		{
			for( int x = 0 ; x < DEPTH_SIZE_X ; x++ )
			{

				GsMaterial m;
				m.diffuse = GsColor(x/DEPTH_SIZE_X,y/DEPTH_SIZE_Y,(x+y)/(DEPTH_SIZE_X+DEPTH_SIZE_Y));

				model->V.push(GsVec((float)(5*x)/DEPTH_SIZE_X,(float)(5*y)/DEPTH_SIZE_X,0.0f));
				model->M.push(m);


				if(y<DEPTH_SIZE_Y-1 && x < DEPTH_SIZE_X-1)
				{
					GsModel::Face f;
					int ul = element(x,y);
					int ur = element(x+1,y);
					int ll = element(x,y+1);
					int lr = element(x+1,y+1);

					f.set(ul,ur,ll);
					model->F.push(f);
					f.set(ur,lr,ll);
					model->F.push(f);
				}
			}
		}

		model->culling = false;


		SnColorSurf* surf = new SnColorSurf(model);
		SnGroup* surfGroup = new SnGroup;
		surfGroup->separator(true);
		SnTransform* tran = new SnTransform;


		surfGroup->add(tran);
		surfGroup->add(surf);
		_surfGroup->add(surfGroup);

		_surfs.push(surf);
		_surf_trans.push(tran);
	}

}

SnGroup* KinectViewer::getRoot()
{
	return _root;
}


