#include "ph_mod_contact.h"

#include "ph_manager.h"
#include "util_models.h"
#include "util.h"


static void manipCB ( SnManipulator* mnp,const GsEvent& ev, void* udata )
 {
	
	 ContactModule* step = (ContactModule*)udata;

	 //step->makeCurves();
	GsVec p = mnp->translation();
	phout<<"move swing foot to "<<p<<gsnl;
	

 }
void ContactModule::FixFeet()
{

	if(h->stanceFoot()->getBody()->Rotating() ) 
	{
		h->stanceFoot()->clearTorque();
	}

	if( h->swingFoot()->getBody()->Rotating() ) 
	{
			h->swingFoot()->clearTorque();
	}

}
float ContactModule::rightFootStrength()
{
	if(!rightFootPlanted())return 0;
	float val = 0;
	
	GsVec cop;
	int contactCount = 0;
		float totalVal = 0;
	for(int i=4;i<8;i++)
	{
		if(sensors[i]->isActive())
		{
			totalVal += sensors[i]->getVal().len();
			cop += sensors[i]->getPos()*sensors[i]->getVal().len();
			contactCount++;
		}
	}
	if(contactCount==0)return 0;

	cop /= (contactCount*totalVal);

	GsVec dis = cop - h->rightFoot()->getCOMPosition();
	dis.y=0;

	float len = dis.len();
	if(len==0)
	{
		phout<<"spot on right \n";
		return 10000.0;
	}
	GsVec comDis = h->getCOM() - h->rightFoot()->getCOMPosition();
//	if(comDis.len()>1)return 0;

	return 1.0f/( len ); //+ 0.2*comDis.len());

}
float ContactModule::leftFootStrength()
{
	if(!leftFootPlanted())return 0;
	float val = 0;

	GsVec cop;
	int contactCount = 0;
	float totalVal = 0;
	for(int i=0;i<4;i++)
	{
		if(sensors[i]->isActive())
		{
			totalVal += sensors[i]->getVal().len();
			cop  = cop + sensors[i]->getPos()*sensors[i]->getVal().len();
			contactCount++;
		}
	}
	if(contactCount<4)return 0;

	cop /=(contactCount*totalVal);
	GsVec lfp = h->leftFoot()->getCOMPosition();
	GsVec dis = cop - lfp;
	dis.y=0;
	float len = dis.len();
	if(len==0)
	{
		phout<<"spot on left \n";
			return 10000.0;
	}
	GsVec comDis = h->getCOM() - lfp;
	
	//phout<<"left comDis "<<comDis.len()<<gsnl;

return 1.0f/( len ); //+ 0.2*comDis.len());

}
GsVec ContactModule::getIPStepPos()
{

	GsVec step;
	GsVec comPos = h->getCOM();
	GsVec footPos = h->stanceFoot()->getCOMPosition();
	GsVec v = h->getCOMVelocity();
	float ht = fabs(comPos.y - footPos.y);
	float g = manager->getWorld()->GetGravity().len();
	step.x =  v.x * sqrt(ht/g + v.x * v.x / (4*g*g)) * 1.3f;
	step.z =  v.z * sqrt(ht/g + v.z * v.z / (4*g*g)) * 1.1f;	
	step.y = 0;
	


	if(step.len()>0.5f)
	{
		step.len(0.5f);
	}

	return h->getHeading().conjugate().apply(step);
}
bool ContactModule::flying()
{
	return _flying;
}
void ContactModule::init()
{
	setP(contact_floor_height, 0.0f);
}
void ContactModule::applyParameters()
{
	copBall->visible(pBool(contact_show_cop));
	ip_model->visible(pBool(contact_show_ip_prediction));
	_contact_lines->visible(pBool(contact_show_contact_region));
	_sensors_lines->visible(pBool(contact_show_contact_forces));
	Module::applyParameters();

}
ContactModule::ContactModule(PhysicalHuman* human,const char* file):Module(human,"ContactModule",file)
{
	if(pBool(module_active))
	{
		gsout<<"contact module active\n";
	}
	else
	{
		gsout<<"contact no active\n";
	}
	CHECK_FLOAT(contact_root_mix);
	CHECK_FLOAT(contact_root_transition_speed);
	CHECK_FLOAT(contact_transition_speed);
	CHECK_FLOAT(contact_stance_swing_ratio);



	MAKE_TEMP_PARM(contact_last_pos,GsVec());
	MAKE_TEMP_PARM(contact_next_pos,GsVec());
	MAKE_TEMP_PARM(contact_pos,GsVec());
	MAKE_TEMP_PARM(contact_proj_pos,GsVec());
	MAKE_TEMP_PARM(contact_transition,0.0f);
	MAKE_TEMP_PARM(contact_is_transitioning,false);
	MAKE_TEMP_PARM(contact_floor_height,0.0f);
	MAKE_TEMP_PARM(contact_is_balanced,false);
	MAKE_TEMP_PARM(contact_cop_pos,GsVec());
	MAKE_TEMP_PARM(contact_pressure,0.0f);

	CHECK_BOOL(contact_show_contact_forces);
	CHECK_BOOL(contact_show_cop);
	CHECK_BOOL(contact_show_ip_prediction);
	CHECK_BOOL(contact_show_contact_region);
	CHECK_FLOAT(contact_toe_heel_ratio);
	CHECK_VEC(contact_stance_offset);
	CHECK_BOOL(contact_double_support);
	

	#ifdef VERIFY_PARAMETERS
		verifyParameters();
	#endif  

	_flying = true;

	
	//this assumes that the left and right foot have the same dimensions
	GsVec sze = h->leftFoot()->getDimension();
	float lr =  sze.x/2.0f; 
	float f =  sze.z/2.0f; 
	float b =  -f; //-0.55f;
	float y = -sze.y/2.0f; 

	sensors[LFL] = new Sensor( h->leftFoot(),GsVec(-lr,y,f) );
	sensors[LFR] = new Sensor( h->leftFoot(),GsVec(lr,y,f));
	sensors[LBL] = new Sensor( h->leftFoot(),GsVec(-lr,y,b));
	sensors[LBR] = new Sensor( h->leftFoot(),GsVec(lr,y,b));
	sensors[RFL] = new Sensor( h->rightFoot(),GsVec(-lr,y,f));
	sensors[RFR] = new Sensor( h->rightFoot(),GsVec(lr,y,f));
	sensors[RBL] = new Sensor( h->rightFoot(),GsVec(-lr,y,b));
	sensors[RBR] = new Sensor(h->rightFoot(),GsVec(lr,y,b));

	ip_model = new Box(h->swingFoot()->getCOMPosition(),h->swingFoot()->getDimension()*0.2f);
	ip_model->setColor(GsColor(0,200,0));
	
	_sensors_lines = new SnGroup;
	_contact_lines = new SnLines;
	_contact_proj_lines = new SnLines;

	grp->add(_contact_proj_lines);
	grp->add(_contact_lines);
	grp->add(_sensors_lines);
		



	for(int i=0;i<8;i++)
		{
			_sensors_lines->add(sensors[i]->line->getGrp());
			_sensors_lines->add(sensors[i]->ball->getGrp());
		}

	contact_model = new Ball(pVec(contact_pos),0.02f); 
	grp->add(contact_model->getGrp());
	contact_model->setColor(GsColor::darkred);
	copBall = new Ball(GsVec(),0.03f);
	copBall->setColor(GsColor::red);
	grp->add(copBall->getGrp());
	grp->add(ip_model->getGrp());

	applyParameters();
}
bool ContactModule::rightFootContact()
{
	for(int i=4;i<8;i++)
	{
		if(sensors[i]->isActive())
		{
			return true;
		}
	}


	return false;
}
bool ContactModule::leftFootContact()
{

	for(int i=0;i<4;i++)
	{
		if(sensors[i]->isActive())
		{
			return true;
		}
	}


	return false;
}

bool ContactModule::leftFootPlanted()
{
	for(int i=0;i<4;i++)
	{
		if(sensors[i]->getVal().len() < 0.1f)
		{
			#ifdef COLOR_FEEDBACK
				h->leftFoot()->_body->setColor(GsColor::red);
			#endif
			return false;
		}
	}
	#ifdef COLOR_FEEDBACK
		h->leftFoot()->_body->setColor(GsColor::blue);
	#endif

	return true;
}

GsVec ContactModule::leftComOffset()
{
	GsVec p =  h->getCOMProjection() - h->leftFoot()->getCOMPosition();
	p.y = 0;
	return p;
}
GsVec ContactModule::rightComOffset()
{
	GsVec p = h->getCOMProjection() - h->rightFoot()->getCOMPosition();
	p.y=0;
	return p; 
}
GsVec ContactModule::stanceComOffset()
{
	if(h->getStanceState() == STANCE_LEFT)
		return leftComOffset();
	else
		return rightComOffset();
}
GsVec ContactModule::swingComOffset()
{
	if(h->leftStance())
		return rightComOffset();
	else
		return  leftComOffset();
}

bool ContactModule::stanceFootContains(GsVec p)
{
	if(h->leftStance())
		return leftFootContains(p);
	else
		return rightFootContains(p);
}
bool ContactModule::swingFootContains(GsVec p)
{
	if(h->leftStance())
		return rightFootContains(p);
	else
		return leftFootContains(p);
}
bool ContactModule::stanceFootPlanted()
{
	if(h->leftStance())
		return leftFootPlanted();
	else
		return rightFootPlanted();
}
bool ContactModule::swingFootPlanted()
{
	if(h->leftStance())
		return rightFootPlanted();
	else
		 return leftFootPlanted();
}
bool ContactModule::rightFootPlanted()
{
	for(int i=4;i<8;i++)
	{
		if(sensors[i]->getVal().len() < 0.1f)
		{
		#ifdef COLOR_FEEDBACK
			h->rightFoot()->_body->setColor(GsColor::red);
		#endif
			return false;
		}
	}
	#ifdef COLOR_FEEDBACK
		h->rightFoot()->_body->setColor(GsColor::blue);
	#endif

	return true;
}
bool ContactModule::bothFeetPlanted()
{
	return leftFootPlanted() && rightFootPlanted();
}
ContactModule::~ContactModule(void)
{

}
bool ContactModule::leftFootContains(GsVec p)
{

	GsVec np;
	GsPolygon pol;
	GsPolygon c_poly;
	GsModel* lf = h->leftFoot()->getBody()->getModel()->getModel();

GsMat lmat = h->leftFoot()->getBody()->getModel()->getTfm()->get();
	for (int i=0; i<lf->V.size(); i++ )
	{ 
		np = lf->V[i]*lmat;
		pol.push ( GsVec2(np.x, np.z) );  // project to floor
	}

	pol.convex_hull ( c_poly );  // get convex hull


	if(c_poly.contains( GsPnt2(p.x,p.z) )) 
		return true;

	return false;

}
bool ContactModule::rightFootContains(GsVec p)
{
	GsVec np;
	GsPolygon pol;
	GsPolygon c_poly;
	GsModel* rf = h->rightFoot()->getBody()->getModel()->getModel();
	GsMat rmat =h->rightFoot()->getBody()->getModel()->getTfm()->get();


	for (int i=0; i<rf->V.size(); i++ )
	{ 
		np = rf->V[i]*rmat;
		pol.push ( GsVec2(np.x, np.z) );  // project to floor
	}

	pol.convex_hull ( c_poly );  // get convex hull


	if(c_poly.contains( GsPnt2(p.x,p.z) )) 
		return true;

	return false;

}
bool ContactModule::bothFeetContain(GsVec p)
{
	GsVec np;
	GsPolygon pol;
	GsPolygon c_poly;

	GsModel* lf = h->leftFoot()->getBody()->getModel()->getModel();
	GsModel* rf = h->rightFoot()->getBody()->getModel()->getModel();

	for (int i=0; i<lf->V.size(); i++ )
	{ 
		p = lf->V[i]*h->leftFoot()->getBody()->getModel()->getTfm()->get();
		pol.push ( GsVec2(p.x, p.z) );  // project to floor
	}


	for (int i=0; i<rf->V.size(); i++ )
	{ 
		p = rf->V[i]*h->rightFoot()->getBody()->getModel()->getTfm()->get() ;
		pol.push ( GsVec2(p.x, p.z) );  // project to floor
	}

	pol.convex_hull ( c_poly );  // get convex hull

	if(contact_poly.contains( GsPnt2(p.x,p.z) )) 
		return true;

	return false;

}

GsPolygon ContactModule::updateContactRegion()
{
		GsPolygon pol_proj;
		GsModel* lf = h->leftFoot()->getBody()->getModel()->getModel();
		GsModel* rf = h->rightFoot()->getBody()->getModel()->getModel();

		GsVec2 contact_proj;
		for(int i=0;i<8 ;i++)
		{
			GsVec2 p = GsVec2(sensors[i]->getPos().x,sensors[i]->getPos().z);
			pol_proj.push (p);
			contact_proj +=p;
			
		}
		contact_proj/=8.0f;
		setP(contact_proj_pos,GsVec(contact_proj.x,pFloat(contact_floor_height), contact_proj.y));

		pol_proj.convex_hull(contact_proj_poly);

		if(manager->animationStep())
		{
			_contact_proj_lines->init();

			if(contact_proj_poly.size()>0)
			{
				_contact_proj_lines->color(GsColor(255,0,0));
				_contact_proj_lines->begin_polyline ();
				for(int j=0;j<contact_proj_poly.size();j++)
				{
					_contact_proj_lines->push(contact_proj_poly.get(j).x,pFloat(contact_floor_height),contact_proj_poly.get(j).y);
				}
				_contact_proj_lines->push(contact_proj_poly.get(0).x,pFloat(contact_floor_height),contact_proj_poly.get(0).y);
				_contact_proj_lines->end_polyline ();
			}
		}


	if(flying())return 0;


	int i;
	GsPnt p;

	GsPolygon pol;


	
	bool looking = true;

	for(int i=0;i<8 && looking;i++)
	{
		if(sensors[i]->isActive())
		{
			looking = false;
			setP(contact_floor_height,sensors[i]->getPos().y);
		}

	}
	float sensorHeight = pFloat(contact_floor_height)+0.01f;
	
	for ( i=0; i<lf->V.size(); i++ )
	{ 
		p = lf->V[i]*h->leftFoot()->getBody()->getModel()->getTfm()->get();
		if(p.y < sensorHeight)pol.push ( GsVec2(p.x, p.z) );  // project to floor

		
	}
	
	for ( i=0; i<rf->V.size(); i++ )
	{ 
		p = rf->V[i]*h->rightFoot()->getBody()->getModel()->getTfm()->get() ;
		if(p.y < sensorHeight)pol.push ( GsVec2(p.x, p.z) );  // project to floor
		
	}
	


	pol.convex_hull ( contact_poly );  // get convex hull
	

	if(manager->animationStep())
	{
		_contact_lines->init();
		if(contact_poly.size()>0)
		{
			_contact_lines->color(GsColor(0,255,0));
			_contact_lines->begin_polyline ();
			for(int j=0;j<contact_poly.size();j++)
			{
				_contact_lines->push(contact_poly.get(j).x,pFloat(contact_floor_height),contact_poly.get(j).y);
			}
			_contact_lines->push(contact_poly.get(0).x,pFloat(contact_floor_height),contact_poly.get(0).y);
			_contact_lines->end_polyline ();
		}
	
	}
	
	GsVec com = h->getCOM();

	if(contact_poly.contains( GsPnt2(com.x,com.z) )) 
		setP(contact_is_balanced,true);
	else setP(contact_is_balanced,false);

	return pol;
}

bool ContactModule::balanced()
{
	if(flying())return false;
	return pBool(contact_is_balanced);
}

GsVec ContactModule::stancePoint()
{
	if(h->doubleSupported())
	{
		float len = h->stanceFoot()->getBody()->dimension().z/2.0f;
		GsVec frontBack =  	h->getHeading().apply( GsVec(0.0f,0.0f, pFloat(contact_toe_heel_ratio)*len + (1- pFloat(contact_toe_heel_ratio))*-len));
		return frontBack + pFloat(contact_stance_swing_ratio)*h->stanceFoot()->getCOMPosition()+(1.0f-pFloat(contact_stance_swing_ratio))*h->swingFoot()->getCOMPosition();
	}
	else
	{
		GsVec offset = pVec(contact_stance_offset);
		if(h->getStanceState() == STANCE_LEFT)
		{
			offset.x = -offset.x;
		}
		
		offset = h->getHeading().apply(offset);
		return h->stanceFoot()->getCOMPosition() + offset;
	}
}

GsVec ContactModule::updateContactCenter()
{
	//if transitioning interpolate the last and next contact centers
	if(pBool(contact_is_transitioning))
	{
		if(pFloat(contact_transition)>1.0f)
		{
			setP(contact_transition,1.0f); setP(contact_is_transitioning,false);
		}
		setP(contact_pos, interp( pVec(contact_last_pos),pVec(contact_next_pos),pFloat(contact_transition)));
	}
	
	return pVec(contact_pos);
}
GsVec ContactModule::getStableContactCenter()
{
	GsPolygon p = updateContactRegion();
	GsVec avg;
	for(int i=0;i<p.size();i++)
	{
		avg += GsVec(p.get(i).x,0.0f,p.get(i).y);
	}
	avg/=(float)p.size();
	return avg;
}

bool ContactModule::evaluate()
{
	bool valid = Module::evaluate();
	//automatically check what the contact state is when flying otherwise it is user determined
	
		updateContactRegion();
	if(manager->animationStep())
	{
		ip_model->setPosition(getIPStepPos()+h->getCOMProjection());
	}

	for(int i=0;i<manager->getWorld()->contactPoints.size();i++)
	{

		ODEContact* cont = manager->getWorld()->contactPoints.get(i);

		for(int i=0;i<8;i++)
		{
			sensors[i]->checkCollision(cont);
		}

	}

	_flying = true;
	for(int i=0;i<8;i++)
	{
		if(sensors[i]->update())_flying = false;
	}

	//contact_cop_pos
	GsVec loc;
	float totalVal = 0;
	for(int i=0;i<8;i++)
	{
			loc-=sensors[i]->getPos()*sensors[i]->getVal().y;
			totalVal-=sensors[i]->getVal().y;
	}
//	phout<<totalVal<<gsnl;
	if(totalVal<1)
	{
		setP(contact_cop_pos,h->getCOMProjection());
		setP(contact_pressure,0.0f);
		copBall->visible(false);
	}
	else
	{
		loc/=totalVal;
		loc.y = pFloat(contact_floor_height);
		setP(contact_cop_pos,loc);
		setP(contact_pressure,totalVal);
		if(manager->animationStep())
		{
			copBall->visible(true);
			copBall->setPosition(loc);
		}
	}

	if(pBool(contact_is_transitioning))
	{
		setP(contact_transition,pFloat(contact_transition) + pFloat(contact_transition_speed));
		
		if(pFloat(contact_transition)>1.0f) 
		{
			setP(contact_is_transitioning,false);
		}
		else
		{
			updateContactCenter();
		}
	}
	
	if(manager->animationStep())
		contact_model->setPosition(stancePoint());
	return valid;
}
GsString ContactModule::stateString()
{
	if(pBool(module_active))
	{
		gsout<<"contact module active\n";
	}
	else
	{
		gsout<<"contact no active\n";
	}

	GsString f = name();
	f<<"\n{\n";
	f<<parameterAsString(module_active);
	f<<parameterAsString(contact_toe_heel_ratio);
	f<<parameterAsString(contact_stance_swing_ratio);
	f<<parameterAsString(contact_stance_offset);
	f<<"}\n";
	return f;
}



Sensor::Sensor(PhysicalJoint* j, GsVec pos)
{
	overSample = 0.0f; //0.9f;
	sensorPos = pos;
	val = GsVec(0,0,0);
	active = false;
	joint = j;
	lastUpdate = 1;
	line = new Line(pos,GsVec(0.0f,0.1f,0.0f));
	ball = new Ball(pos,0.02f,GsColor::green);
}
bool Sensor::update()
{
	ball->setPosition(getPos());
	if(lastUpdate>0)
	{
		val = val*0.8f;
		active = false;
		line->visible(false);
	}
	else
	{
		active = true;
		line->visible(true);
		line->setPoints(getPos(),getPos()-val*0.002f);
	}
	lastUpdate++;
	return active;
}
bool Sensor::checkCollision(ODEContact* cont)
{
	if( cont->involves( joint->getBody()->getBodyID() ))
	{

		GsVec dif = cont->cp - getPos();
		
		if(dif.len()<0.02f)
		{
			lastUpdate=0;
			val = val*overSample + cont->f*(1.0f-overSample);
			
			return true;

		}
		else
		{

		}
	}
	return false;
}


