

#include "util_primitives.h"

#include <gsim/gs_ogl.h>
#include <gsim/gs_string.h>
#include <fltk/events.h>

#include "phw_node_viewer.h"
#include "phw_window.h"
#include "ph_manager.h"
#include "ph_motion_manager.h"
#include "ph_mod_ik.h"
#include "util_channel_traj.h"
# include <fltk/gl.h>
#include <fltk/SharedImage.h>

#include "util_channel.h"
#include "ph_motion.h"

GsVec2 HumanWindowNodeViewer::getOutputDrawPosition(GsVec2 drawP)
{
	float nsize = pVec(node_viewer_node_size).x;

	return drawP + GsVec2(nsize/2.0f,0.0f);
}

GsVec2 HumanWindowNodeViewer::getInputDrawPosition(GsVec2 drawP,int i)
{
	GsVec2 node_size = GsVec2(pVec(node_viewer_node_size).x,pVec(node_viewer_node_size).y);

	if(i==0)
		return drawP + GsVec2(0.0f,node_size.y/2);
	if(i==1)
		return drawP - GsVec2(node_size.x/2.0f,node_size.y/3);
	if(i==2)
		return drawP - GsVec2(node_size.x/2.0f,-node_size.y/3);


		return drawP - GsVec2(node_size.x/2.0f,0.0f);

}



HumanWindowNodeViewer::HumanWindowNodeViewer ( int x, int y, int w, int h ): FlViewer ( x, y, w, h," " ),Serializable("NodeViewer")
{
	pickprec = 0.1f;
	W = w;
	H = h;
	
	Origin = H/2;


	mainwin = 0;
	viewScale=1.0f;
	
	editState = node_not_editing;
	_edit_index = -1;

}


void HumanWindowNodeViewer::init (HumanWindow* win,HumanMotionManager* mgr,const GsString& file)
{
	//   manager->graph = this;
	mainwin = win;
	
	FlViewer::view_all();
	
	manager = win->manager;

	loadParametersFromFile(file);
	CHECK_FLOAT(node_viewer_text_size);
	CHECK_VEC(node_viewer_node_size);
	CHECK_VEC(node_viewer_control_size);
	CHECK_VEC(node_viewer_object_size);
	CHECK_VEC(node_viewer_text_offset);
	CHECK_VEC(node_viewer_control_offset);
	CHECK_FLOAT(node_viewer_text_line_offset);
	CHECK_FLOAT(node_viewer_control_text_offset);
	CHECK_FLOAT(node_viewer_feedback_text_offset);
	CHECK_COLOR(node_viewer_background_color);
	CHECK_COLOR(node_viewer_node_color);
	CHECK_COLOR(node_viewer_object_color);
	CHECK_COLOR(node_viewer_text_color);
	CHECK_COLOR(node_viewer_temp_node_color);
}



void HumanWindowNodeViewer::draw ()
{
	if ( !valid() )
	{ 
		W = FlViewer::w();
		H = FlViewer::h();
		glViewport ( -W, 0, W*2, H );
		glEnable ( GL_DEPTH_TEST );
		glCullFace ( GL_BACK );
		glFrontFace ( GL_CCW );
		glDisable(GL_LIGHTING);
	
	}
	if(W!=FlViewer::w() )
	{
		W = FlViewer::w();
		H = FlViewer::h();
		glViewport ( -W, 0, W*2, H );
		camera().aspect = (2*(float)W)/(float)H;
	}

	glPushMatrix();
	//----- Clear Background -----
	glClearColor ( pColor(node_viewer_background_color));
	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glTranslate(_view_translate);
	glScale(viewScale);

	GsVec2 object_size = GsVec2(pVec(node_viewer_object_size).x,pVec(node_viewer_object_size).y);
	GsVec2 control_size = GsVec2(pVec(node_viewer_control_size).x,pVec(node_viewer_control_size).y);
	GsVec2 node_size = GsVec2(pVec(node_viewer_node_size).x,pVec(node_viewer_node_size).y);
	GsVec textOffset = pVec(node_viewer_text_offset);
	float lineOff = pFloat(node_viewer_text_line_offset);
	float textScale = pFloat(node_viewer_text_size) * 0.5f;
	float control_text_offset = pFloat(node_viewer_control_text_offset);
	glLineWidth(2.0f);
	if(editState == node_connecting)
	{
		Channel* input = _channels[_edit_index];
		GsVec2 curveS = getOutputDrawPosition(_channels[_edit_index]->getNodePosition());
		GsVec2 curveE =  mouseP;
		GsVec2 curveSt = curveS;
		GsVec2 curveET = curveE;
		curveET.x = curveS.x;
		curveSt.x = curveE.x;

		GsArray<GsVec2> pts;
		for(float t=0.0;t<=1.05f;t+=0.05f)
		{
			pts.push(evalBez(t,curveS,curveSt,curveET,curveE));
		}
		glColor3f(0,1,0);
		drawPolyline(pts);		
	}
	if(editState == node_removing)
	{
		GsVec2 curveS = getInputDrawPosition(_channels[_edit_index]->getNodePosition(),input_removing);			
		glColor3f(1,0,0);
		drawLine(curveS,mouseP);
	}
		glLineWidth(0.01f);

	for(int i=0;i<temp_nodes.size();i++)
	{
		//first draw any detached channels
		GsVec2 pnt = temp_nodes[i].pos;

		float lx = (pnt.x-textOffset.x+ 0.01f)*viewScale + _view_translate.x;
		float ly = (pnt.y-textOffset.y)*viewScale + _view_translate.y ;
		//GsVec2 pt = pnt*viewScale;

	
		glDrawString(temp_nodes[i].obj->name(),lx,ly+lineOff*viewScale,pColor(node_viewer_text_color),true,viewScale*textScale );	
		glDrawString(channelTypeToString((channel_dof_types)temp_nodes[i].channelType),lx,ly, pColor(node_viewer_text_color),true,viewScale*textScale);	
		glDrawString(temp_nodes[i].obj->getParameter(temp_nodes[i].parmID)->name,lx,ly-lineOff*viewScale, pColor(node_viewer_text_color),true,viewScale*textScale);	



		/*glDrawString(label,lx,ly, GsColor::white );		*/
		glColor(pColor(node_viewer_temp_node_color));
		drawRec(temp_nodes[i].pos,object_size);
	}


	for( int it=0;it<_channels.size();it++)
	{	
		if(_channels[it]->nodeVisible())
		{
			drawChannel(_channels[it]);
		}
	}		

	for( int it=0;it<_channels.size();it++)
	{	
		Channel* ch = _channels[it];
		GsVec2 p = ch->getNodePosition();
		if(ch->nodeVisible())
		{

		
		for(int i=0;i<ch->numInputs();i++)
		{
			
			Channel* input = ch->getInput(i);
			if(input && (input->nodeVisible()))
			{

				GsVec2 curveS = getOutputDrawPosition(input->getNodePosition());
				GsVec2 curveE;
				if(ch->getChannelMode() == channel_modulate || ch->getChannelMode() == channel_switch)
					curveE =  getInputDrawPosition(p,i);
				else
					curveE =  getInputDrawPosition(p);
				float ds = abs(curveE.x - curveS.x);
				GsVec2 curveSt = curveS+GsVec2(ds,0.0f);

				GsVec2 curveEt;

				if( (ch->getChannelMode() == channel_modulate || ch->getChannelMode() == channel_switch) && i==0 )
				{
					//for these cases make the input come up from the middle
					curveEt = curveE+GsVec2(0.0f,0.3f);
					//curveE = curveE;
				}
				else
					curveEt = curveE-GsVec2(ds,0.0f);

				GsArray<GsVec2> pts;
				for(float t=0.0;t<=1.1f;t+=0.1f)
				{
					pts.push(evalBez(t,curveS,curveSt,curveEt,curveE));
				}
				drawPolyline(pts);
			}
		}
		}
	}

	glPopMatrix();

	fltk::GlWindow::redraw();
}




//Transform coordinates
GsVec2 HumanWindowNodeViewer::curveToGrid(GsVec2 input)
{
	GsVec2 newp = scene2winGraph(input);
	newp.y = newp.y ;
	newp.y = GS_TORAD(Origin-newp.y);
	newp.x=input.x; //(input.x+1.0f)/2.0f; //go from [0,1]=>[0,1]
	return newp;
}

bool pointHitsRect(GsVec2 p,GsVec2 rc,GsVec2 dim)
{
	dim /= 2.0f;
	if(p.x <rc.x+dim.x && p.x > rc.x-dim.x)
	{
		if(p.y>rc.y-dim.y && p.y < rc.y+dim.y)
		{
			
			return true;

		}
	}
	return false;
}
int HumanWindowNodeViewer::handle ( int ev )
{
	HumanMotionManager* motion_manager = manager->getMotionManager();
	if(!motion_manager)
		return false;

	 mouseP = win2sceneGraph(GsVec2((float)fltk::e_x , (float)fltk::e_y ));
	ptInfoX = " ";
	ptInfoY = " ";
	bool updateCvs = false;
	int retVal = 0;
	
	GsVec2 node_size = GsVec2(pVec(node_viewer_node_size).x,pVec(node_viewer_node_size).y);
	GsVec2 control_size = GsVec2(pVec(node_viewer_control_size).x,pVec(node_viewer_control_size).y);
	GsVec2 temp_node_size = GsVec2(pVec(node_viewer_object_size).x,pVec(node_viewer_object_size).y);

	switch(ev)
	{		
	case fltk::PUSH :	
		{
			mouseP.x =	mouseP.x - _view_translate.x;
			mouseP.y =	mouseP.y - _view_translate.y;
			mouseP.x = mouseP.x/viewScale;
			mouseP.y = mouseP.y/viewScale;
			mouseStart = mouseP;
			
			retVal = 1;
				
			editState = node_view_moving;
			_edit_index = -1;

			for( int it=0;it<_channels.size();it++)
			{
				Channel* ch = _channels[it];
				if(ch->nodeVisible())
				{
					if(pointHitsRect(mouseP,ch->getNodePosition(), node_size))
					{
						mainwin->ui_node_val->value(ch->getControlVal());
						nodeOffset = mouseP - ch->getNodePosition();
						
						_edit_index = it;
						editState = node_moving;
						lastNodeSelected = ch;
						motion_manager->selectChannel(ch);
						mainwin->selectChannel(ch->name());
					}
					else if(pointHitsRect(mouseP,getOutputDrawPosition(ch->getNodePosition()),control_size))
					{
						editState = node_connecting;
						_edit_index = it;
					}
					else 
					{
						if(ch->getChannelMode() == channel_modulate || ch->getChannelMode() == channel_switch)
						{
							for(int i=0;i<ch->numInputs();i++)
							{
								if(pointHitsRect(mouseP,getInputDrawPosition(ch->getNodePosition(),i),control_size))
								{
									input_removing = i;
									editState = node_removing;
									_edit_index = it;
								}
							}
						}
						else
						{
							if(pointHitsRect(mouseP,getInputDrawPosition(ch->getNodePosition()),control_size))
							{
								input_removing = 0;
								editState = node_removing;
								_edit_index = it;
							}
						}
					}
				}	
			}
			for(int i=0;i<temp_nodes.size();i++)
			{
				if(pointHitsRect(mouseP,temp_nodes[i].pos, temp_node_size))
				{
					editState = node_temp_selected;
					_edit_index = i;
					nodeOffset = mouseP - temp_nodes[i].pos;
				}
			}
		}break;
	case fltk::DRAG :
		{
	
			if(editState == node_view_moving)
			{
					mouseP.x = mouseP.x/viewScale;
					mouseP.y = mouseP.y/viewScale;
					GsVec2 tr = mouseP-mouseStart;
					
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

				if(_edit_index!=-1)
				{
					if(editState == node_moving)
					{
						_channels[_edit_index]->setP(channel_node_position,GsVec( mouseP-nodeOffset));
					}
					else if(editState == node_temp_selected)
					{
						temp_nodes[_edit_index].pos = mouseP-nodeOffset;
					}
				}
			}
		}
		break;
	case fltk::RELEASE:
		
		mouseP.x =	mouseP.x - _view_translate.x;
		mouseP.y =	mouseP.y - _view_translate.y;
		mouseP.x = mouseP.x/viewScale;
		mouseP.y = mouseP.y/viewScale;
		if(editState == node_connecting)
		{
			for(int i=0;i<temp_nodes.size();i++)
			{
				if(pointHitsRect(mouseP,temp_nodes[i].pos, temp_node_size))
				{
					_channels[_edit_index]->setParameter(temp_nodes[i].obj,temp_nodes[i].parmID,(channel_dof_types)temp_nodes[i].channelType,temp_nodes[i].arrID);
					temp_nodes.remove(i);
				}
			}
			for( int it=0;it<_channels.size();it++)
			{
				if(it!=_edit_index)
				{
					Channel* ch = _channels[it];
					if(ch->nodeVisible())
					{
						if(ch->getChannelMode() == channel_modulate || ch->getChannelMode() == channel_switch)
						{
							for(int i=0;i<3;i++)
							{
								if(pointHitsRect(mouseP,getInputDrawPosition(ch->getNodePosition(),i), control_size))
								{

									ch->setInput(_channels[_edit_index],i);
									ch->updateInputList();
									motion_manager->updateCurves();
								}
							}
						}
						else if(pointHitsRect(mouseP,getInputDrawPosition(ch->getNodePosition()), control_size))
						{
							if(it!=_edit_index)
							{
								ch->pushInput(_channels[_edit_index]);
								ch->updateInputList();
							}
						}
					}
				}
			}
		}
		else if(editState == node_removing)
		{
			for( int it=0;it<_channels.size();it++)
			{
				
					Channel* ch = _channels[it];
					if(ch->nodeVisible())
					{
						if(_channels[_edit_index]->getInput(input_removing) == ch)
						{
							if(pointHitsRect(mouseP,getOutputDrawPosition(ch->getNodePosition()), control_size))
							{
								if(it!=_edit_index)
								{
										_channels[_edit_index]->removeInput(input_removing);
										_channels[_edit_index]->updateInputList();
								}
							}
						}
					}
			}
		}
		editState = node_not_editing;

		break;
	case fltk::MOUSEWHEEL:
		viewScale = viewScale + (0.1f*fltk::event_dy());
		if(viewScale<0.1f)
			viewScale = 0.1f;
		if(viewScale>5)
			viewScale = 5;
		break;
	case fltk::KEY:
		phout<<"key";
		break;

	default:  break;
	}
// 	if(updateCvs)
// 	{
// 		manager->getMotionManager()->updateCurves();
// 		mainwin->setUIFromCharacter();
// 	}
	
	if(retVal)
		return retVal;
	
	return fltk::GlWindow::handle(ev);
}


/*!screen space to scene  pixels to pos x = [0,1]=>[0,W] y=[-1,1]=>[H,0]*/
GsVec2 HumanWindowNodeViewer::scene2winGraph(GsVec2 input)
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
GsVec2 HumanWindowNodeViewer::win2sceneGraph ( GsVec2 input )
{
	float px = AppXmin + (AppXmax-AppXmin)*(float)(input.x)/(float)W;
	float py = -AppYmin - (AppYmax-AppYmin)*(float)input.y/(float)H;
	return GsVec2 ( px, py );
}

void HumanWindowNodeViewer::arrange_nodes()
{
	
	int count = 0;
	
	

	for( int it=0;it<_channels.size();it++)
	{

		if(_channels[it]->nodeVisible())
		{
			if(_channels[it]->numInputs()==0)
			{
				GsVec v = _channels[it]->getNodePosition();
				v.y = - 1.0f+ count*0.3f;
				_channels[it]->setP(channel_node_position,v);
				
				count++;
			}
		}

	}
	for(int i=0;i<3;i++) 
	{
		for( int it=0;it<_channels.size();it++)
		{
			if(_channels[it]->nodeVisible())
			{
				if(_channels[it]->numInputs()>0)
				{
					GsVec p;
					for(int j=0;j<_channels[it]->numInputs();j++)
					{
						if(_channels[it]->getInput(j))
							p+=_channels[it]->getInput(j)->getNodePosition();
					}
					p /= (float)_channels[it]->numInputs();
					p.y+=0.05f;
					p.x+=0.4f;
					_channels[it]->setP(channel_node_position,p);
					
				}
			}
		}
	}

}
void HumanWindowNodeViewer::pushChannels(const GsArray<Channel*>& chs)
{
	_channels.push(chs);
}
void HumanWindowNodeViewer::initChannels()
{
	lastNodeSelected = 0;
	_channels.size(0);
	_edit_index = -1;
	editState = node_not_editing;
}
void HumanWindowNodeViewer::detach_node()
{
	if(_edit_index !=-1)
	{
		Channel* ch = _channels[_edit_index];
		if(ch->getObject())
		{
			temp_nodes.push().obj = ch->getObject();
			temp_nodes.top().pos = getOutputDrawPosition(ch->getNodePosition()) +GsVec2(0.15f,0.0f);
			temp_nodes.top().arrID = ch->pInt(channel_float_index);
			temp_nodes.top().parmID = ch->getParameterID();
			temp_nodes.top().channelType = ch->getChannelType();
			ch->setObject(0);
		}
		
	}
}
void HumanWindowNodeViewer::addNode(Serializable* sav, int parmID,channel_dof_types dofType,int arrID)
{
	temp_nodes.push().obj = sav;
	temp_nodes.top().arrID = arrID;
	temp_nodes.top().parmID = parmID;
	temp_nodes.top().channelType = dofType;
	temp_nodes.top().pos.set(0,0);
	phout<<"adding node with dof "<<channelTypeToString((channel_dof_types)dofType);
	
}
GsVec2 HumanWindowNodeViewer::scaleVec( GsVec2 p )
{
	return GsVec2(p.x, p.y * (float)(2*W) / (float)H);
}
void HumanWindowNodeViewer::drawChannel( Channel* ch )
{
	GsVec2 p = ch->getNodePosition();
	GsVec2 temp_node_size = GsVec2(pVec(node_viewer_object_size).x,pVec(node_viewer_object_size).y);
	GsVec2 node_size = GsVec2(pVec(node_viewer_node_size).x,pVec(node_viewer_node_size).y);
	GsVec2 control_size = GsVec2(pVec(node_viewer_control_size).x,pVec(node_viewer_control_size).y);
	GsVec textOffset = pVec(node_viewer_text_offset);
	float lineOff = pFloat(node_viewer_text_line_offset);
	float textScale = pFloat(node_viewer_text_size);
	GsVec controlOffset = pVec(node_viewer_control_offset);
	float control_text_offset = pFloat(node_viewer_control_text_offset);
	float feedback_text_offset = pFloat(node_viewer_feedback_text_offset);

	GsString label = " ";
	
	switch(ch->getChannelMode())
	{
	case channel_trajectory:
	
		break;
	case channel_additive:
		label << "+";
		break;
	case channel_multiply:
		label << "X";
		break;
	case channel_modulate:
		label << "%";
		break;
	case channel_switch:
		label << "_-_";
		break;
	case channel_inverse:
		label << "!";
		break;
	}
	label <<":"<<ch->name();

	float lx = (p.x-textOffset.x)*viewScale + _view_translate.x;
	float ly = (p.y-textOffset.y)*viewScale + _view_translate.y ;
	GsVec2 pt = p*viewScale;

	glDrawString(label,lx,ly, pColor(node_viewer_text_color), true,viewScale*textScale);

	if(ch->getControlVal()!=0 && (ch->getChannelMode() == channel_additive || ch->getChannelMode() == channel_multiply || ch->getChannelMode() == channel_feedback))
	{	
		label = ch->getControlVal();
		glDrawString(label,lx,ly-lineOff*viewScale, pColor(node_viewer_text_color), true,viewScale*textScale);
	}

	glColor(pColor(node_viewer_node_color));
	drawRec(p,node_size);

	if(ch == lastNodeSelected)
	{
		glColor(GsColor(255,0,0));
		drawRec(p,node_size*1.1f);

	}
	//if the node connects to an object draw a rectangle with the object description
	if(ch->getObject())
	{
		lx = p.x*viewScale  + _view_translate.x;
		ly = p.y*viewScale + _view_translate.y;
		GsVec2 po;
		//if it is a feedback node draw the object to the left with a different color
		if(ch->getChannelMode() == channel_feedback)
		{
			float contOff = (controlOffset.x + feedback_text_offset)*viewScale;
			po = getInputDrawPosition(p) - GsVec2(controlOffset.x,0.0f);
			glDrawString(ch->getObject()->name(),		lx-contOff	,	ly + lineOff*viewScale		, pColor(node_viewer_text_color),true,viewScale*textScale );	
			glDrawString(ch->pString(channel_dof),				lx-contOff	,	ly							,pColor(node_viewer_text_color),true,viewScale*textScale);	
			//glDrawString(ch->pString(channel_parameter_name),	lx-contOff	,	ly-lineOff*viewScale		, pColor(node_viewer_text_color),true,viewScale*textScale);	
			glColor(pColor(node_viewer_feedback_color));
			drawRec(po,temp_node_size);
		}
		else //draw the object rect to the right
		{
			float contOff = (controlOffset.x - control_text_offset)*viewScale;
			po= getOutputDrawPosition(p) + GsVec2(controlOffset.x,0.0f);
			glDrawString(ch->getObject()->name(),				lx+contOff	,	ly+lineOff*viewScale	, pColor(node_viewer_text_color),true,viewScale*textScale );	
			glDrawString(ch->pString(channel_dof),				lx+contOff	,	ly						, pColor(node_viewer_text_color),true,viewScale*textScale );	
			//glDrawString(ch->pString(channel_parameter_name),	lx+contOff	,	ly-lineOff*viewScale	, pColor(node_viewer_text_color),true,viewScale*textScale);	
			glColor(pColor(node_viewer_object_color));
			drawRec(po,temp_node_size);
		}

		
	}

	if(ch->getObject())
		glColor3f(0,0.9f,0.0f);
	else
		glColor3f(0.9f,0.0f,0.0f);

	drawRec(getOutputDrawPosition(p),control_size); 

	if(ch->isTrajectory())
		glColor3f(0,0.9f,0.0f);
	else
	{
		if(ch->numInputs() == 0)
			glColor3f(0.9f,0.0f,0.0f);
		else
			glColor3f(0,0.9f,0.0f);
	}

	switch(ch->getChannelMode())
	{
		case channel_modulate:
		case channel_switch:
		{	


			for (int i=0;i<3;i++)
			{
			
				drawRec(getInputDrawPosition(p,i),control_size); 
			}
		}
		break;
	
		default:
			drawRec(getInputDrawPosition(p),control_size); 
		break;
	}

	if(ch->isTrajectory())
	{
		GsVec2 sp = getInputDrawPosition(p) - GsVec2(0.05f,0.0f);
		lx = (sp.x-0.02f)*viewScale + _view_translate.x;
		ly = (sp.y-0.03f)*viewScale + _view_translate.y;
		glDrawString("~",lx,ly, pColor(node_viewer_text_color),true,viewScale*textScale );
		glColor3f(0,0,1);
		drawRec(sp,control_size);
	}
	glColor3f(0.0f,0.0f,0.0f);

}

Channel* HumanWindowNodeViewer::getSelectedNode()
{
	return lastNodeSelected;
}



