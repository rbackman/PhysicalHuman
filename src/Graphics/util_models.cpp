#include "util_models.h"



Model::Model(GsVec Pt)
{
	node =0;
	model = 0;
	pos = Pt;
	tfm = new SnTransform; tfm->ref();
	grp = new SnGroup; grp->ref();
	grp->separator(true);
	grp->add(tfm);
	visible(true);
	model = new GsModel; model->ref();
}
void Model::init()
{
	if(model)
	{	
		node = new SnModel(model); node->ref();
	}
	else
	{
		//phout<<"model not initialized\n";
	
	}

	if(node)
		grp->add(node);
	else
		gsout<<"node not initialized\n";

}
OBJModel::OBJModel(GsString filename):Model(GsVec())
{
	if(!model->load_obj(filename))
	{
		gsout<<"error couldnt load model file "<<filename<<gsnl;
	}
	else
	{
		Model::init();
	}
}

Circle::Circle(GsVec p,float rad):Model(p)
{
	node=new SnLines; node->ref();
	model->unref();
	model = 0;
	makeCircle(p,rad);
	Model::init();
}
void Circle::setColor(GsColor col)
{
	SnLines* line = (SnLines*)node;
	line->color(col);
	Model::setColor(col);
}

void Circle::makeCircle( GsVec p, float rad )
{
	radius = rad;
	center = p;
	SnLines* line = (SnLines*)node;
	line->init();
	
	line->begin_polyline ();
	

	for	   ( unsigned i=0; i<=360; i+=10 ) 
	{
		line->push( GsVec( center.x+radius*cos(GS_TORAD(i)), center.y+radius*sin(GS_TORAD(i)),0 ));
	}
	
		line->end_polyline ();
}

Line::Line(GsVec p1,GsVec p2):Model(p1)
{
	node=new SnLines; node->ref();
	setPoints(p1,p2);
	model->unref();
	model = 0;
	Model::init();
}
void Line::setColor(GsColor col)
{
	SnLines* line = (SnLines*)node;
	line->color(col);
	Model::setColor(col);

}
void Line::setPoints(GsVec p1, GsVec p2)
{
	SnLines* line = (SnLines*)node;
	line->init();
	line->begin_polyline ();
	line->push(p1);
	line->push(p2);
	line->end_polyline ();
}
void Line::setVec(GsVec v)
{
	GsVec pos = ((SnLines*)node)->V[0];
	setPoints(pos,pos+v);
}
void Model::setPosition(GsVec p)
{
	pos = p;
	compose ( rot, pos, m );
	tfm->get().set(m);	
}
void Model::visible(bool t)
{
	grp->visible(t);
	if(node!=0)
	{
		if(node->type() == SnNode::TypeShape)
		{
			((SnShape*)node)->touch();
		}
	}

}
bool Model::visible()
{
return grp->visible();
}
void Model::setRotation(GsQuat q)
{
	rot = q;
	compose ( rot, pos, m );
	tfm->get().set(m);	
}
GsColor Model::getColor()
{

	return c;
}
void Model::setColor(GsColor col)
{
	
	c = col;
	if(model)
	{
		GsMaterial m;
		m.diffuse = c;
		model->set_one_material(m);
	}

	if(node!=0)
	{
		if(node->type() == SnNode::TypeShape)
		{
			((SnShape*)node)->touch();
		}
	}

}
void Model::rotate(float radx,float rady,float radz)
{
	GsMat m;
	GsQuat rot;
	gs_rot(gsXYZ,rot,radx,rady,radz);
	compose ( rot, pos, m );
	tfm->get().set(m);	
}
Model::~Model()
{
	grp->remove_all();
	tfm->unref();
	grp->unref();
    if(model)
		model->unref();
	if(node)
		node->unref();
}
Capsule::Capsule(GsVec p1, GsVec p2,float r1i,float r2i):Model(p1)
{
	r1 = r1i;
	r2 = r2i;
	model->make_capsule(p1,p2,r1,r2,20,true);
	Model::init();
}
void Capsule::setPoints(GsVec p1, GsVec p2)
{
	model->make_capsule(p1,p2,r1,r2,20,true);
}
Ball::Ball(GsVec Pt, float r,GsColor c) : Model(Pt)
{
	rad = r;

	model->make_sphere (GsVec(0,0,0),r,12,1);
	setColor(c);
	Model::init();
	setPosition(Pt);

}
	
Ball::Ball(GsVec Pt, float r) : Model(Pt)
{

	rad = r;

	model->make_sphere (GsVec(0,0,0),r,12,1);
	setColor(GsColor::blue);
	Model::init();
	setPosition(Pt);

}
void Ball::setRadius(float r)
{
	rad = r;
	model->make_sphere(GsVec(0,0,0),r,12,1);
}

Arrow::Arrow(GsVec Pt, GsVec dir):Model(Pt)
{
	model->load_obj("../data/models/arrow.obj");
	Model::init();
	setDirection(dir);
}

void Arrow::setDirection(GsVec d)
{
	direction = d;
	setRotation(GsQuat(GsVec(0,0,1),direction));
}

Box::Box(GsVec Pt, GsVec sze):Model(Pt)
{

	setSize(sze);
	GsMaterial newMat;
	newMat.diffuse.set(0,0,255);
	model->set_one_material(newMat);
	Model::init();
	setPosition(Pt);
}
void Box::setSize(int dof,float sze)
{
	_size[dof] = sze;
	setSize(_size);
}
void Box::setSize(GsVec sze)
{
	_size = sze;
GsVec a = GsVec(-sze.x/2.0f,-sze.y/2.0f,-sze.z/2.0f);
	GsVec b = GsVec(sze.x/2.0f,sze.y/2.0f,sze.z/2.0f);
	model->make_box(GsBox(a,b));
	
	if(node!=0)
	{
		if(node->type() == SnNode::TypeShape)
		{
			((SnShape*)node)->touch();
		}
	}
}



	
		