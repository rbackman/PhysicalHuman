
#include "util_manipulator.h"


static void manipCallback ( SnManipulator* mnp,const GsEvent& ev, void* udata )
{
	Manipulator* scaleManip = ((Manipulator*)udata);
	Manipulator* manip = (Manipulator*)mnp;
	scaleManip->updateScale();
}

void Manipulator::init(const GsString& name,const GsString& typ, const GsString& file)
{
		
	loadParametersFromFile(file);
	CHECK_VEC(manipulator_position);
	CHECK_QUAT(manipulator_orientation);
	CHECK_VEC(manipulator_offset);
	CHECK_VEC(manipulator_size);
	CHECK_BOOL(manipulator_visible);
	CHECK_BOOL(manipulator_active);
	CHECK_BOOL(manipulator_show_box);
	CHECK_COLOR(manipulator_active_color);
	CHECK_BOOL(manipulator_simple);
	init();

	//applyParameters();

}
void Manipulator::init()
{
	for(int i=0;i<6;i++)
		constraints.push(true);
	_wantsMenu = false;
	
	manipulator_mode = BOX_MANIP;
	scaleManipSelected = -1;

	currentDof = -1;
	_hit_active = false;

	grp = new SnGroup;
	grp->ref();
	grp->separator ( true );

	model = new Box(GsVec(),pVec(manipulator_size));
	model->setColor(pColor(manipulator_active_color));

	if(!pBool(manipulator_simple))
	{
			manipMods.size(3);
	
		manipMods[AxisMod::PX].model =  new OBJModel("../data/models/maniptrans.obj");
		
		manipMods[AxisMod::PY].model =  new OBJModel("../data/models/maniptrans.obj");
	
		manipMods[AxisMod::PZ].model =  new OBJModel("../data/models/maniptrans.obj");
	
		manipModelGroup = new SnGroup;
		manipModelGroup->ref();
		for (int i=0;i<manipMods.size();i++)
		{
			manipMods[i].dof = i;
			manipModelGroup->add(manipMods[i].model->getGrp());
		}

		
		manipMods[AxisMod::PX].model->setColor(GsColor::red);
		manipMods[AxisMod::PY].model->setColor(GsColor::green);
		manipMods[AxisMod::PZ].model->setColor(GsColor::blue);
		manipMods[AxisMod::PX].model->setRotation(GsQuat(GsVec::j,GS_TORAD(90)));
		manipMods[AxisMod::PY].model->setRotation(GsQuat(GsVec::i,GS_TORAD(90)));

		


	
		manipCircleGroup = new SnGroup;
		manipCircleGroup->ref();
		manipScaleGroup = new SnGroup;
		manipScaleGroup->ref();
		manipScaleGroup->visible(false);
		grp->add(manipScaleGroup);

	
		scaleManipX= new Manipulator(GsVec(1,0,0),GsVec(0.2,0.2,0.2),true);
		scaleManipX->setColor(GsColor::red);
		scaleManipX->ref();
		scaleManipX->callback(manipCallback,this);
		scaleManipX->setConstraint(0,0,0,1,0,0);

		scaleManipY= new Manipulator(GsVec(0,1,0),GsVec(0.2,0.2,0.2),true);
		scaleManipY->setColor(GsColor::green);
		scaleManipY->ref();
		scaleManipY->callback(manipCallback,this);
		scaleManipY->setConstraint(0,0,0,0,1,0);

		scaleManipZ= new Manipulator(GsVec(0,0,1),GsVec(0.2,0.2,0.2),true);
		scaleManipZ->setColor(GsColor::blue);
		scaleManipZ->ref();
		scaleManipZ->callback(manipCallback,this);
		scaleManipZ->setConstraint(0,0,0,0,0,1);

		manipScaleGroup->add(scaleManipX);
		manipScaleGroup->add(scaleManipY);
		manipScaleGroup->add(scaleManipZ);

		

		manipCircles.size(3);
		manipCircles[AxisCircle::RX].circle = new Circle(GsVec(),0.5f);
		manipCircles[AxisCircle::RY].circle = new Circle(GsVec(),0.5f);
		manipCircles[AxisCircle::RZ].circle = new Circle(GsVec(),0.5f);

		manipCircles[AxisCircle::RX].circle->setColor(GsColor::red);
		manipCircles[AxisCircle::RY].circle->setColor(GsColor::green);
		manipCircles[AxisCircle::RZ].circle->setColor(GsColor::blue);

		manipCircles[AxisCircle::RX].circle->setRotation(GsQuat(GsVec::j,GS_TORAD(90)));
		manipCircles[AxisCircle::RY].circle->setRotation(GsQuat(GsVec::i,GS_TORAD(90)));
		
		

		for (int i=0;i<manipCircles.size();i++)
		{
			manipCircles[i].dof = i;
			manipCircleGroup->add(manipCircles[i].circle->getGrp());
		}
	
	}
	else
	{
		manipCircleGroup=0;
		manipModelGroup = 0;
		manipScaleGroup = 0;
	}

	mat().scale(0.5f);
	child(model->getNode());
	
}
void Manipulator::setInitial()
{
	
	start_position = pVec(manipulator_position);
	start_orientation = pQuat(manipulator_orientation);
	
	/*GsMat m;
	compose(start_orientation,start_position,m);
	initial_mat(m);*/

}
Manipulator::Manipulator(const GsString& name,const GsString& typ, const GsString& file):SnManipulator(),Serializable(name,typ)
{
	SnManipulator::ref();
#ifdef PRINT_CONSTRUCTORS
	gsout<<"Manipulator(GsString name,GsString typ, GsString file)"<<gsnl;
#endif

	init(name,typ,file);
	setInitial();
	applyParameters();
}

Manipulator::Manipulator(const GsVec& pos,const GsVec& size,bool simpleManip ):SnManipulator(),Serializable("Manip","ManipType")
{
	SnManipulator::ref();
	MAKE_PARM(manipulator_position,pos);
	MAKE_PARM(manipulator_orientation,GsQuat());
	MAKE_PARM(manipulator_offset,GsVec());
	MAKE_PARM(manipulator_size,size);
	MAKE_PARM(manipulator_visible,true);
	MAKE_PARM(manipulator_active,true);
	MAKE_PARM(manipulator_show_box,true);
	MAKE_PARM(manipulator_active_color,GsColor::blue);
	MAKE_PARM(manipulator_simple,simpleManip);

	init();
	setInitial();
	
}

void Manipulator::updateScaleOffsets()
{
	
	scaleManipX->translation(GsVec(this->globalPosition().x+scaleValue.x,this->globalPosition().y,this->globalPosition().z));
	scaleManipY->translation(GsVec(this->globalPosition().x,this->globalPosition().y+scaleValue.y,this->globalPosition().z));
	scaleManipZ->translation(GsVec(this->globalPosition().x,this->globalPosition().y,this->globalPosition().z+scaleValue.z));
}


void Manipulator::updateScale()
{

scaleValue.x = scaleManipX->globalPosition().x - this->globalPosition().x;
scaleValue.y = scaleManipY->globalPosition().y - this->globalPosition().y;
scaleValue.z = scaleManipZ->globalPosition().z - this->globalPosition().z;

setSize(scaleValue*2);
}
Manipulator::~Manipulator(void)
{
	grp->remove_all();
	if(!pBool(manipulator_simple))
	{
		for (int i=0;i<3;i++)
		{
			delete manipMods[i].model;
		}
		manipModelGroup->remove_all();
		manipModelGroup->unref();
		manipCircleGroup->remove_all();
		manipCircleGroup->unref();
		manipScaleGroup->remove_all();
		manipScaleGroup->unref();
		scaleManipX->unref();
		scaleManipY->unref();
		scaleManipZ->unref();

		for (int i=0;i<3;i++)
		{
			delete manipCircles[i].circle;
		}
	
	}

	delete model;
	grp->unref();
	SnManipulator::unref();
}
void Manipulator::_transform ( const GsPnt& p, const GsVec& r )
{
	GsVec dt;

	if ( _mode==ModeRotating )
	{ }
	else
	{ dt = p-_firstp; 
	if(!constraints[3])dt.x=0;
	if(!constraints[4])dt.y=0;
	if(!constraints[5])dt.z=0;
	}

	GsMat R;
	GsQuat q;
	if ( r.x && constraints[0]) q.set ( GsVec::i, r.x );
	if ( r.y && constraints[1]) q.set ( GsVec::j, r.y );
	if ( r.z && constraints[2]) q.set ( GsVec::k, r.z );
	_rotation = _rotation * q;

	_rotation.get ( R );
	_translation += dt * R;

	R[12] = _translation.x;
	R[13] = _translation.y;
	R[14] = _translation.z;

	SnEditor::mat ( R * _initmat );
}
bool Manipulator::evaluate()
{
	//SnPos = frame*(p+start) - start + origin
	//pos = frame.inv().apply(SnPos + start - origin) - start
	setP(manipulator_position,	frame.inverse().apply(SnManipulator::translation() - origin + start_position) - start_position); //
	setP(manipulator_orientation,frame.inverse()*SnManipulator::rotation());
	return true;
}
void Manipulator::loadStateFromFile(const GsString& file)
{
	setParametersFromFile(file);
	SnManipulator::translation(pVec(manipulator_position));
	SnManipulator::rotation(pQuat(manipulator_orientation));
	applyParameters();
}
GsString Manipulator::stateString()
{
	GsString ss = name();
	ss<<"{\n";
	ss<<parameterAsString(manipulator_position);
	ss<<parameterAsString(manipulator_orientation);
	ss<<parameterAsString(manipulator_active);
	ss<<parameterAsString(manipulator_visible);
	ss<<"}\n";
	return ss;
}

void Manipulator::translation(const GsVec& p)
{
	setP(manipulator_position,p); 
	GsQuat q = pQuat(manipulator_orientation);

	//SnPos = frame*(p+start) - start + origin
	SnManipulator::translation(-start_position + frame.apply(p+start_position) + origin ); 
}
void Manipulator::rotation(const GsQuat& q )
{
	setP(manipulator_orientation,q); 
	SnManipulator::rotation(frame*q);
}

GsVec Manipulator::globalPosition()
{
	GsVec t;
	 mat().getrans(t);
	return t;
}
GsVec Manipulator::localPosition()
{
	return pVec(manipulator_position);
}
GsQuat Manipulator::globalOrientation()
{
	return frame*start_orientation*pQuat(manipulator_orientation);
}
GsQuat Manipulator::localOrientation()
{
	return pQuat(manipulator_orientation);
}
float Manipulator::lineIntersectCircle(Circle* m,const GsLine& l,GsVec* v)
{
	GsVec pt;
	GsQuat rot = m->getRotation();
	GsPlane p = GsPlane(GsVec(),rot.apply(GsVec(0,0,1)));
	float t;
	pt = globalOrientation().apply(p.intersect(l.p1,l.p2,&t))+globalPosition();

	int closest = 0;
	float dist = fabs(GsVec(pt-globalPosition()).len()-m->radius);
	*v = pt;

	return dist;
}
bool lineIntersectModel(Model* m,const GsLine& l,GsVec* v)
{
	GsBox b;
	GsModel* md = m->getModel();

	md->get_bounding_box(b);
	b = b*m->getTfm()->get();
		
	float t1,t2;
	float a1,b1,c1;
	GsArray<GsVec> hits;
   if(l.intersects_box(b,t1,t2)>0)
   {
	   for (int i=0;i<md->F.size();i++)
	   {
		   GsModel::Face f = md->F.get(i);
		   GsVec va = md->V[f.a];
		   GsVec vb = md->V[f.b];
		   GsVec vc = md->V[f.c];
		   GsMat mt =  m->getTfm()->get();

		   va = va*mt;
		   vb = vb*mt;
		   vc = vc*mt;

		  if( l.intersects_triangle(va,vb,vc,a1,b1,c1))
		  {
			  hits.push(GsVec(a1*va + vb*b1 + vc * c1));
		  }
	   }
   }
   if(hits.size()==0)return false;

   int closest = 0;
   float dist = GsVec(l.p1-hits[0]).len();
   float thisDist = 0;
   for (int i=1;i<hits.size();i++)
   {
		thisDist = GsVec(l.p1-hits[0]).len();
			if (thisDist<dist)
			{
				dist = thisDist;
				closest = i;
			}
   }
   *v = hits[closest];
   return true;
}
GsString dofToString(int d)
{
	switch(d)
	{
		case AxisMod::PX: return "PX"; break;
		case AxisMod::PY: return "PY"; break;
		case AxisMod::PZ: return "PZ"; break;
	}
	return "bad dof";
}



void Manipulator::visible(bool v)
{
	setP(manipulator_visible,v);
	
	applyParameters();
}
void Manipulator::applyParameters()
{
	rotation(pQuat(manipulator_orientation));
	translation(pVec(manipulator_position));
	
	if(pBool(manipulator_active))
		model->setColor(pColor(manipulator_active_color));
	else
		model->setColor(GsColor::red);

	
	
	model->visible(pBool(manipulator_visible));

	manipModelGroup->visible(pBool(manipulator_visible));
	manipCircleGroup->visible(pBool(manipulator_visible));

	SnManipulator::visible(pBool(manipulator_visible));
	
	grp->visible(pBool(manipulator_visible));
	
	if(box())
	{
		box()->visible(pBool(manipulator_visible));
	}
	else
		phout<<"no box\n";


	
	child()->visible(pBool(manipulator_visible));
	
	update();
}

bool Manipulator::wantsPopupMenu(GsVec2* p)
{
	if(_wantsMenu)
	{
		_wantsMenu = false;
		*p = mousep;
		return true;
	}
	return false;
}

void Manipulator::setOrigin(const GsVec& pos, const GsQuat& rot )
{
	origin = pos;
	frame = rot;
	applyParameters();
}

void Manipulator::match()
{

}

int Manipulator::handle_event ( const GsEvent &e , float t )
{
	int res =0;
	//gsout<<"start handle_event\n";
	
	if(pBool(manipulator_simple))
		return SnManipulator::handle_event(e,t);

	if(e.type == GsEvent::Keyboard )
	{
	res =	handleKeyboard(e,t);

		
	}

	if(manipulator_mode == BOX_MANIP )
		return SnManipulator::handle_event(e,t);


	manipHits.size(0);
	
	
	//gsout<<"handle_event\n";
	if ( e.type==GsEvent::Push )
	{ 
		res = handlePush(e,t);

	}
	
	if( e.type==GsEvent::Release)
	{
		gsout<<"released hit\n";
		_hit_active = false;
		setP(manipulator_active,false);
		currentDof = -1;
		res = 1;

		if(manipulator_mode == SCALE_MANIP)
			res = SnManipulator::handle_event(e,t);
	}
	
	if ( e.type==GsEvent::Drag )
	{
		
	res = handleDrag(e,t);
		
	}
	
	return res;
}
int Manipulator::check_event( const GsEvent& e, float& t )
{
	if(manipulator_mode == ROT_TRANS_MANIP || manipulator_mode == CIRCLE_MANIP ||manipulator_mode == SCALE_MANIP )
	{
		if ( e.type==GsEvent::Drag || e.type == GsEvent::Keyboard)
		{
			if(_hit_active)
				return 2;
		}
		if ( e.type==GsEvent::Release)
		{
			_hit_active = false;
			return 0;
		}
	}
	return SnManipulator::check_event(e,t);
}

void Manipulator::setColor(const GsColor& c )
{
	model->setColor(c);
}

void Manipulator::setSize(const GsVec& sze )
{
	((Box*)model)->setSize(sze);
	child(model->getNode());
}
void Manipulator::setSize(int dof,float sze)
{
	((Box*)model)->setSize(dof,sze);
	child(model->getNode());
}

GsVec Manipulator::size()
{
	return ((Box*)model)->getSize();
}


int Manipulator::handlePush( const GsEvent & e, float t )
{
	//gsout<<"handle push\n";
	int res = 0;
	if(manipulator_mode == SCALE_MANIP)
		res = SnManipulator::handle_event(e,t);

	float t1,t2;
	int k = e.ray.intersects_box ( box()->cbox(), t1, t2);
	if ( k==0 )
	{ 
		currentDof = -1;
		_hit_active = false;
		return 0;
	}

	if(manipulator_mode == ROT_TRANS_MANIP)
	{

		GsVec hit;
		for(int i=0;i<manipMods.size();i++)
		{

			if(lineIntersectModel(manipMods[i].model,e.ray,&hit))
			{
				manipHits.push().dof = i;
				manipHits.top().hitP = hit;
			}
		}

		if (manipHits.size()==0)
		{
			currentDof = -1;
			_hit_active = false;
			return 0;
		}

		float dist = 0;
		float smallestDist=100;
		int closest = 0;
		for(int i=0;i<manipHits.size();i++)
		{
			dist = GsVec(manipHits[i].hitP-e.ray.p1).len();
			if (dist<smallestDist)
			{
				smallestDist = dist;
				closest = i;
			}
		}
		currentDof = manipHits[closest].dof;
	}

	if(manipulator_mode == CIRCLE_MANIP)
	{
		GsVec hit;
		for(int i=0;i<manipCircles.size();i++)
		{
			GsVec p;
			float dist = lineIntersectCircle(manipCircles[i].circle,e.ray,&p);
			//gsout<<"dist is "<<dist<<" for dof "<<i<<gsnl;
			manipHits.push().dist = dist;
			manipHits.top().dof = i;
			manipHits.top().hitP = p;


		}

		if (manipHits.size()==0)
		{
			currentDof = -1;
			_hit_active = false;
			return 0;
		}

		float dist = 0;
		float smallestDist=100;
		int closest = -1;
		for(int i=0;i<manipCircles.size();i++)
		{
			if(manipHits[i].dist<0.1)
			{


				dist = GsVec(manipHits[i].hitP-e.ray.p1).len();
				if (dist<smallestDist)
				{
					smallestDist = dist;
					closest = i;
				}
			}
		}
		if(closest!=-1)
		{
			currentDof = manipHits[closest].dof;
			gsout<<"closest dof is "<<currentDof<<gsnl;
			hitPoint = manipHits[closest].hitP;
			Ball* b = new Ball(manipHits[closest].hitP,0.01f);
			b->setColor(manipCircles[closest].circle->getColor());
			grp->add(b->getGrp());
		}
		else
		{
			currentDof = -1;
			_hit_active = false;
			return 0;
		}
	}

	gsout<<"hit active\n";
	_hit_active = true;
	setP(manipulator_active,true);
	initialRot = localOrientation();
	initialPos = localPosition();
	mousep = e.mouse;

	return res;
}

int Manipulator::handleDrag( const GsEvent & e, float t )
{
	int res = 1;
	if(_hit_active)
	{
		if(manipulator_mode == ROT_TRANS_MANIP)
		{
			GsVec2 dv = mousep - e.mouse;
			float d = dv.x+dv.y;
			switch(currentDof)
			{
			case AxisMod::PX: translation(initialPos+globalOrientation().apply( GsVec(-d,0.0f,0.0f))); break;
			case AxisMod::PY: translation(initialPos+globalOrientation().apply(GsVec(0.0f,-d,0.0f))); break;
			case AxisMod::PZ:translation(initialPos+globalOrientation().apply(GsVec(0.0f,0.0f,d))); break;
			}
		}
		else if(manipulator_mode == CIRCLE_MANIP)
		{
			GsVec p;
			float dist = lineIntersectCircle(manipCircles[currentDof].circle,e.ray,&p);
			p = p-globalPosition();
			GsVec p1 = hitPoint-globalPosition();

			GsQuat rotFix;
			switch(currentDof)
			{

			case AxisCircle::RX:rotFix= GsQuat(GsVec::i,(float)GS_PIDIV2); break; //GsQuat(GsVec::i,anglenorm(p1,GsVec::i)-GS_PI)
			case AxisCircle::RY:rotFix= GsQuat(GsVec::j,(float)GS_PIDIV2); break;
			case AxisCircle::RZ:rotFix= GsQuat(GsVec::k,(float)GS_PIDIV2) ;break;

			}

			//p1 = rotFix.apply(p1);

			p.normalize();
			p1.normalize();
			p = rotFix.apply(p);

			float ang = anglenorm(p,p1) - GS_PIDIV2;

			switch(currentDof)
			{
			case AxisCircle::RX: rotation(initialRot*GsQuat(GsVec::i,ang)) ; break;
			case AxisCircle::RY: rotation(initialRot*GsQuat(GsVec::j,ang)); break;
			case AxisCircle::RZ: rotation(initialRot*GsQuat(GsVec::k,ang)) ;break;
			}
		}
		else if(manipulator_mode == SCALE_MANIP)
		{

			res = SnManipulator::handle_event(e,t);
			updateScaleOffsets();

		}
		evaluate();
	}
		return res;
}

int Manipulator::handleKeyboard( const GsEvent & e, float t )
{
	int res = 0;
		float t1,t2;
			int k = e.ray.intersects_box ( box()->cbox(), t1, t2);
			if ( k!=0 )
			{ 
				if(e.ctrl)
				{
					if(manipulator_mode == BOX_MANIP )
					{
						child(manipModelGroup);
						manipulator_mode = ROT_TRANS_MANIP;
					}
					else
					{
						child(model->getNode());
						manipulator_mode = BOX_MANIP;
					}

					toggleBool(manipulator_show_box);
					applyParameters();
				
				}
				if(e.shift)
				{
					if(manipulator_mode == BOX_MANIP )
					{
						//gsout<<"circle mode ";
						child(manipCircleGroup);
						manipulator_mode = CIRCLE_MANIP;
					}
					else
					{
						child(model->getNode());
						manipulator_mode = BOX_MANIP;
					}


					applyParameters();
				}
				if(e.alt)
				{
					if(manipulator_mode == BOX_MANIP )
					{
						//gsout<<"circle mode ";
						//child(model->getNode());
						manipulator_mode = SCALE_MANIP;
						scaleManipSelected = 0;
						manipScaleGroup->visible(true);
						scaleValue = size();
						scaleManipX->translation(this->globalPosition()+GsVec(scaleValue.x,0,0));
						scaleManipY->translation(this->globalPosition()+GsVec(0.0f,scaleValue.y,0));
						scaleManipZ->translation(this->globalPosition()+GsVec(0.0f,0,scaleValue.z));
					}
					else
					{
						//child(model->getNode());
						manipulator_mode = BOX_MANIP;
						manipScaleGroup->visible(false);
					}


					applyParameters();
					/*
					toggleBool(manipulator_active);
					if(pBool(manipulator_active))
						model->setColor(pColor(manipulator_active_color));
					else
						 model->setColor(GsColor::red);
						 */
				}
				
				applyParameters();
			}	
		return res;
}
