

#include "util_primitives.h"

#include <gsim/gs_ogl.h>
#include <gsim/gs_string.h>
#include <fltk/events.h>

#include "phw_graph_viewer.h"
#include "phw_window.h"
#include "ph_manager.h"
#include "ph_motion_manager.h"
#include "ph_mod_ik.h"
#include "util_channel_traj.h"
# include <fltk/gl.h>
#include <fltk/SharedImage.h>

#include "util_channel.h"
#include "ph_motion.h"

HumanWindowGraphViewer::HumanWindowGraphViewer ( int x, int y, int w, int h ): FlViewer ( x, y, w, h," " ),Serializable("GraphViewer")
{
	pickprec = 0.1f;
	W = w;
	H = h;
	
	Origin = H/2;
	gridW = W/4;
	gridH = H/6;
	_edit_mode = CURVE_MOVE_POINT;
	mainwin = 0;
	viewScale=1.0f;

	
//	buttonImage = fltk::pngImage::get("../data/bluebar.png");
	

	//buttonImage->fetch();

	//phout<<si->get_filename()<<gsnl;
	//buttonImage->fe();
}


void HumanWindowGraphViewer::drawGrid ()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

	float offset =0;
	float duration = motion_manager->duration();
	bool labels = false;
	float minV = -1.0f;
	float maxV = 1.0f;
	if(motion_manager->getSelectedChannels()->size()==1)
	{	
		labels = true;

		offset = motion_manager->getSelectedChannels()->get(0)->crest();
		
		minV = motion_manager->getSelectedChannels()->get(0)->cmin();
		maxV = motion_manager->getSelectedChannels()->get(0)->cmax();
	}
	glLineWidth(0.08f);
	glColor4f(0.7f,0.0f,0.0f,0.5f);
	drawLine(GsVec2(0,0),GsVec2(duration,0.0f) );


	glColor4f(0.7f,0.7f,0.7f,0.1f);
	glLineWidth(0.01f);

	//draw light gray lines
	for(float it = 0; it <= maxV; it += 0.5f)
	{
		GsVec2 pt = win2sceneGraph(GsVec2(2.0f,(float)it));
		drawLine(GsVec2(0.0f,it),GsVec2(duration,it) );
	}
	for(float it = 0; it >= minV; it -= 0.5f)
	{
		GsVec2 pt = win2sceneGraph(GsVec2(2.0f,(float)it));
		drawLine(GsVec2(0.0f,it),GsVec2(duration,it) );
	}

	for(float it = 0; it <= duration; it += 0.5f)
	{
		drawLine(GsVec2(it,minV),GsVec2(it,maxV) );
	}
// 	if(labels)
// 	{
// 		glLineWidth(0.05f);
// 		for(float it = minV; it < maxV; it += 0.5f)
// 		{
// 			GsVec2 pt = win2sceneGraph(GsVec2(2.0f,(float)it));
// 				GsString label;
// 				label<<" "<<it;
// 				glDrawString (label , pt.x , pt.y+0.01f, GsColor::black );
// 			
// 		}
// 	}
	
}
void HumanWindowGraphViewer::init (HumanWindow* win,HumanMotionManager* mgr,GsString file)
{
	loadParametersFromFile(file);
	CHECK_BOOL(graph_viewer_draw_grid);
	CHECK_FLOAT(graph_viewer_background_color);
	CHECK_COLOR(graph_viewer_default_curve_color);
	CHECK_INT(graph_viewer_default_curve_width);
	CHECK_INT(graph_viewer_default_point_width);
	CHECK_INT(graph_viewer_sample_width);
	CHECK_BOOL(graph_viewer_draw_bounds);
	//   manager->graph = this;
	mainwin = win;
	
	FlViewer::view_all();
	
	manager = win->manager;

	
	
}


void HumanWindowGraphViewer::overrideCurveLook()
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

	Motion* m = motion_manager->currentMotion();
	if(m)
	{
		for( int it=0;it<m->numChannels();it++)
		{
			if(m->getChannel(it)->isTrajectory())
			{
				m->getChannel(it)->setP(trajectory_channel_curve_width,pInt(graph_viewer_default_curve_width));
				m->getChannel(it)->setP(trajectory_channel_curve_color,pColor(graph_viewer_default_curve_color));
				m->getChannel(it)->setP(trajectory_channel_point_size,pInt(graph_viewer_default_point_width));
				m->getChannel(it)->applyParameters();
			}
		}
	}

}
void HumanWindowGraphViewer::draw ()
{

	/*
	Track* t = manager->timeslider->timeline;
	if(t) { drawC = true; cvs = t->cvs;}
	*/
	W = FlViewer::w();
	H = FlViewer::h();

	Origin = H/2;
	gridW = W/4;
	gridH = H/6;

	//glFinish();
	//----- init OpenGL if needed -----
	if ( !valid() )
	{ 
		glViewport ( -W, 0, W*2, H );
		glEnable ( GL_DEPTH_TEST );
		glCullFace ( GL_BACK );
		glFrontFace ( GL_CCW );
		
		glDisable(GL_LIGHTING);

	}
	glPushMatrix();
	//----- Clear Background -----
	glClearColor (pColor(graph_viewer_background_color));
	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glTranslate(_view_translate);
	glScale(viewScale);
	if(mainwin->ui_draw_basis->value())
	{
		float height = (float)mainwin->ui_rbds_height->value();
		float support = (float)mainwin->ui_rbds_support->value();
		GsArray<GsVec2> pts;
		for(float i=-2;i<2;i+=0.01f)
		{
			pts.push(GsVec2(i,RB(i,support,height)));
		}
		glLineWidth((float)pInt(graph_viewer_default_curve_width));
		glColor(pColor(graph_viewer_default_curve_color));

		drawPolyline(pts);
	}
	
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return;

		Motion* m = motion_manager->currentMotion();
		if(m)
		{
			if(pBool(graph_viewer_draw_bounds))
			{
				glColor(pColor(graph_viewer_default_curve_color));
				glLineWidth((float)pInt(graph_viewer_sample_width));
				for( int it=0;it<m->numChannels();it++)
				{
					if(m->getChannel(it)->isTrajectory())
					{
						TrajectoryChannel* ch =  (TrajectoryChannel*)m->getChannel(it);
						Trajectory* cv = ch->getCurve();
						if(cv==0)continue;

						if(ch->curveVisible())
						{
							for(int j = 0;j < ch->getCurve()->numSamplePoints() ;j++)
							{
								sample_data* p = ch->getCurve()->sample(j);
								drawLine( GsVec2(p->rest.x,p->rest.y) + GsVec2(p->max.x,p->max.y) , GsVec2(p->rest.x,p->rest.y) +GsVec2(p->max.x,p->min.y) ); //right
								drawLine( GsVec2(p->rest.x,p->rest.y) +GsVec2(p->min.x,p->max.y),GsVec2(p->rest.x,p->rest.y) +GsVec2(p->max.x,p->max.y) ); //top
								drawLine( GsVec2(p->rest.x,p->rest.y) +GsVec2(p->min.x,p->min.y),GsVec2(p->rest.x,p->rest.y) +GsVec2(p->max.x,p->min.y) ); //bottom
								drawLine( GsVec2(p->rest.x,p->rest.y) +GsVec2(p->min.x,p->max.y),GsVec2(p->rest.x,p->rest.y) +GsVec2(p->min.x,p->min.y)  );
							}
						}
					}
				}
			}
			GsArray<GsVec2> points;

			for( int it=0;it<m->numChannels();it++)
			{
				TrajectoryChannel* ch =  (TrajectoryChannel*)m->getChannel(it);
				if(ch->curveVisible() || ch == mainwin->ui_node_viewer->getSelectedNode()  )
				{

					if(ch->isTrajectory())
					{
					
						Trajectory* cv = ch->getCurve();
						if(cv==0)continue;

						cv->draw();
						if(ch->getChannelMode()!= channel_trajectory)
						{
							GsArray<GsVec2> pts;
						
							for (float t=0;t<m->duration();t+=manager->getAnimationTimeStep())
							{
								pts.push(GsVec2(t,ch->getVal(t)));
							}
							glLineWidth((float)pInt(graph_viewer_default_curve_width));
							glColor(pColor(graph_viewer_default_curve_color));

							drawPolyline(pts);
						}

						
					}
					else if(ch->getChannelMode()!= channel_trajectory)
					{
						if(ch->numInputs()==0)
						{
							drawLine(GsVec2(0.0f,ch->getControlVal()),GsVec2(1.0f,ch->getControlVal()));
						}

						else
						{
							GsArray<GsVec2> pts;
							glColor3f(1,0,0);

							for (float t=0;t<m->duration();t+=manager->getAnimationTimeStep())
							{
								pts.push(GsVec2(t,ch->getVal(t)));
							}
							glLineWidth((float)pInt(graph_viewer_default_curve_width));
							glColor(pColor(graph_viewer_default_curve_color));

							drawPolyline(pts);
						}

					}
					points.push(ch->getLastPoint());
				}
				
				
			}
			
			glColor3f(0,1,0);
			drawPoints(points);
		}
		if(pBool(graph_viewer_draw_grid))
		{
			drawGrid();
			glLineWidth(5);
			glColor3f(0.5,0.5,0);
			float time = motion_manager->getTime();
			drawLine(GsVec2(time,-1.0f),GsVec2(time,1.0f) );
		}

	
	
	

//glDrawString ( ptInfoX, mouseP.x -0.1f, mouseP.y + 0.20f , GsColor::blue );
//	glDrawString ( ptInfoY, mouseP.x -0.1f , mouseP.y + 0.08f , GsColor::blue );
	//glLineWidth ( 1 );
	//glDrawString ( ptInfoX, 0.01f, 0.8f , GsColor::blue );
	//glDrawString ( ptInfoY, 0.01f , 0.7f , GsColor::blue );

	//glTranslate(-_view_translate);
	// Draw selection as a circle of radius pickprec:
	//drawCircle(pickprec,mouseP);
	//drawCircle(0.2, mseGraph);
	glPopMatrix();
	glLineWidth(0.01f);
	fltk::GlWindow::redraw();

	
	//----- Fltk will then automatically flush and swap buffers -----
	

}




//Transform coordinates
GsVec2 HumanWindowGraphViewer::curveToGrid(GsVec2 input)
{
	GsVec2 newp = scene2winGraph(input);
	newp.y = newp.y ;
	newp.y = GS_TORAD(Origin-newp.y);
	newp.x=input.x; //(input.x+1.0f)/2.0f; //go from [0,1]=>[0,1]
	return newp;
}


int HumanWindowGraphViewer::handle ( int ev )
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return 0;

	 mouseP = win2sceneGraph(GsVec2((float)fltk::e_x , (float)fltk::e_y ));
	ptInfoX = " ";
	ptInfoY = " ";
	bool updateCvs = false;
	int retVal = 0;

	switch(ev)
	{		
	case fltk::PUSH :	
		{
			_moving = false;
			
			mouseP.x =	mouseP.x - _view_translate.x;
			mouseP.y =	mouseP.y - _view_translate.y;
			
			mouseP.x = mouseP.x/viewScale;
			mouseP.y = mouseP.y/viewScale;
			
			mouseStart = mouseP;

		
				TrajectoryChannel* ch = motion_manager->push(mouseP,_edit_mode);
				if(ch)
				{

					Trajectory* cv = ch->getCurve();
					//ptInfo = scvs.get(j)->name;
					ptInfoX << mouseP.x ;
					ptInfoY << mouseP.y; // scvs.get(j)->convertVal(mouseP.y);

					updateCvs = true;

					bool found_sample = false;


					if(cv->selectionState == CURVE_TANG_SELECTED)
						retVal = 1;
					if(cv->selectionState == CURVE_POINT_SELECTED)
					{

						retVal = 1;
					}


					mainwin->updateSampleUI();
				}
				else
				{
					retVal = 1;
					_moving = true;
					mainwin->ui_makeSample->deactivate();
					for(int i=0;i<6;i++)
						mainwin->ui_pt[i]->deactivate();

				}	
			
		}break;
	case fltk::DRAG :
		{
	
			if(_moving)
			{
					mouseP.x = mouseP.x/viewScale;
					mouseP.y = mouseP.y/viewScale;
					GsVec2 tr = mouseP-mouseStart;
					//if(tr.x>0)tr.x=0;
					tr.x *= viewScale;
					tr.y *= viewScale;
					_view_translate.x = tr.x;
					_view_translate.y = tr.y;
			}
			else
			{
					updateCvs = true;
				mouseP.x =	mouseP.x - _view_translate.x;
				mouseP.y =	mouseP.y - _view_translate.y;
				mouseP.x = mouseP.x/viewScale;
				mouseP.y = mouseP.y/viewScale;

			
				TrajectoryChannel* ch = motion_manager->drag(mouseP);
				if(ch)
				{
					Trajectory* cv = ch->getCurve();
					if(cv->complement)
					{
						cv->complement->update();
					}
					ptInfoX << cv->getPoint(cv->selection).x <<" "<<cv->getPoint(cv->selection).y;
					if(cv->selectionState == CURVE_POINT_SELECTED)
					{
						
							mainwin->ui_curve_pt[0]->value(cv->getPoint(cv->selection).x);
							mainwin->ui_curve_pt[1]->value(cv->getPoint(cv->selection).y);
						
					}
				}
			}
		}
		
		break;
	case fltk::RELEASE:
		_moving = false;
	
		//updateCvs = true;
		/*for(unsigned j=0;j<cvs.size();j++)
		{
		Trajectory* cv = cvs.get(j)->curve;
		if(cv==0)continue;
		cv->selectionState = 0;
		}*/

		break;
	case fltk::MOUSEWHEEL:

		
		viewScale = viewScale + (0.1f*fltk::event_dy());
		if(viewScale<0.1f)
			viewScale = 0.1f;
		if(viewScale>5)
			viewScale = 5;
	//	phout<<"clicks: "<<fltk::event_dy()<<" scale:"<<viewScale<<gsnl;
	
		break;



	case fltk::KEY:
		phout<<"key";
		break;

	default:  break;
	}
	if(updateCvs)
	{
		manager->getMotionManager()->updateCurves();
		//manager->getMotionManager()->setPhase(manager->getMotionManager()->getPhase());//  
		mainwin->setUIFromCharacter();
	//	redraw();
	}
	
	if(retVal)
		return retVal;
	
	return fltk::GlWindow::handle(ev);
}


/*!screen space to scene  pixels to pos x = [0,1]=>[0,W] y=[-1,1]=>[H,0]*/
GsVec2 HumanWindowGraphViewer::scene2winGraph(GsVec2 input)
{
	float px = W * input.x; //(input.x+1.0f)/ 2.0f;
	float py = H - H*(input.y+1.0f)/2.0f;
	return GsVec2(px,py);
}

const float AppXmin = 0;
const float AppXmax = 1.0;
const float AppYmin = -1.0;
const float AppYmax = 1.0;

/*!screen space to scene  pixels  pos x = [0,W]=>[0,1] y=[H,0]=>[-1,1]*/
GsVec2 HumanWindowGraphViewer::win2sceneGraph ( GsVec2 input )
{
	float px = AppXmin + (AppXmax-AppXmin)*(float)input.x/(float)W;
	float py = -AppYmin - (AppYmax-AppYmin)*(float)input.y/(float)H;
	return GsVec2 ( px, py );
}



