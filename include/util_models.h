#pragma once

#include <gsim/gs_vec.h>
#include <gsim/gs_array.h>
#include <gsim/gs_euler.h>
#include <gsim/gs_polygon.h>
#include <gsim/gs_model.h>
#include <gsim/gs_quat.h>
#include <gsim/sn_transform.h>
#include <gsim/sn_model.h>
#include <gsim/sn_lines.h>

#include <gsim/sn_group.h>

class Model
{
	private:
		GsColor c;
		SnGroup* grp;
		GsMat m;
		GsVec pos;
		GsQuat rot;
		SnTransform* tfm;
	protected:
		GsModel* model;
		void init();
		Model(GsVec Pt);
		SnNode* node;
	public:
		~Model();
		void setPosition(GsVec p);
		void setRotation(GsQuat q);
		void rotate(float radx,float rady,float radz);
		void setColor(GsColor col);
		bool visible();
		void visible(bool t);
		GsQuat getRotation(){return rot;}
		GsVec getTranslation(){return pos;}
		SnNode* getNode(){return node;}
		GsModel* getModel(){return model;}
		SnTransform* getTfm(){return tfm;}
		SnGroup* getGrp(){return grp;}
		GsColor getColor();
		GsString name;
};

class Line: public Model
{
public:
	Line(GsVec p1,GsVec p2);
	void setPoints(GsVec p1, GsVec p2);
	void setVec(GsVec v);
	void setColor(GsColor col);
};
class Circle: public Model
{
public:
	float radius;
	GsVec center;
	Circle(GsVec p,float rad);
	void setColor(GsColor col);
	void makeCircle( GsVec p, float rad );
};
class Capsule: public Model
{
	public:
		Capsule(GsVec P1, GsVec p2,float r1i,float r2i);
		void setPoints(GsVec p1, GsVec p2);
	private:
		float r1, r2;
	
		
};
class Arrow: public Model
{
	public:
		Arrow(GsVec Pt, GsVec dir);
		void setDirection(GsVec d);
	private:
		GsVec direction;
};

class Ball: public Model
{
	public:
		Ball(GsVec Pt, float rad,GsColor c);
		Ball(GsVec Pt, float rad);
		void setRadius(float r);
	private:
		float rad;
};
class Box: public Model
{
	GsVec _size;
	public:
		Box(GsVec Pt, GsVec sz);
		void setSize(GsVec sze);
		void setSize(int dof,float sze);
		GsVec getSize(){return _size;}

};

class OBJModel: public Model
{
	public:
		OBJModel(GsString filename);
		
};