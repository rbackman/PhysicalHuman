#pragma once

#include <gsim/sn_manipulator.h>
#include "util_serializable.h"
#include "util_models.h"



/*!a generic class for implementing a manipulator. if a manipulator has a parent(ie KnJoint or PhysicalJoint) then 
its Origin will be at the location and orientation when the manipulator is created*/

enum manipulator_parms
{
	manipulator_position,
	manipulator_orientation,
	manipulator_offset,
	manipulator_size,
	manipulator_visible,
	manipulator_active,
	manipulator_show_box,
	manipulator_active_color,
	manipulator_simple,
	manipulator_last,
	

};

enum magic_spring_parms
{
	magic_spring_k = manipulator_last,
	magic_spring_c,
	magic_spring_use_orientation,
	magic_spring_kr,
	magic_spring_cr,
	magic_spring_rest_length,
	magic_spring_break_length,
	magic_spring_p1,
	magic_spring_p2,
	magic_spring_force,
	magic_spring_torque,

};
enum virtual_spring_parms
{
	virtual_effector_joint = manipulator_last,
	virtual_base_joint,
	virtual_spring_k,
	virtual_spring_c,
	virtual_spring_use_orientation,
	virtual_spring_kr,
	virtual_spring_cr,
	virtual_spring_rest_length,
	virtual_spring_break_length,
	virtual_spring_p1,
	virtual_spring_p2,
	virtual_spring_force,
	virtual_spring_torque,
	virtual_spring_last_extent,
};
enum ik_manip_parms
{
	ik_manip_orbit = manipulator_last,
	ik_manip_root,
	ik_manip_show_lines
};
struct AxisMod
{
	enum{PX,PY,PZ};
	OBJModel* model;
	int dof;
};
struct AxisCircle
{
	enum{RX,RY,RZ};
	Circle* circle;
	int dof;
};

struct AxisHit 
{
	int dof;
	GsVec hitP;
	float dist;
};


class Manipulator : public SnManipulator, public Serializable
{
private:
	enum manip_modes {BOX_MANIP,ROT_TRANS_MANIP,SCALE_MANIP,CIRCLE_MANIP};
	manip_modes manipulator_mode;
	GsArray<bool> constraints;
	GsArray<AxisMod> manipMods;
	GsArray<AxisHit> manipHits;
	GsArray<AxisCircle> manipCircles;

	SnGroup* manipModelGroup;
	SnGroup* manipCircleGroup;
	SnGroup* manipScaleGroup;
	Manipulator* scaleManipX;
	Manipulator* scaleManipY;
	Manipulator* scaleManipZ;
	GsVec scaleValue;
	void init(const GsString& name,const GsString& typ, const GsString& file);
	void init();

	void setConstraint(int dof,bool val)
	{
		constraints[dof] = val;
	}
	void setConstraint(bool rx,bool ry,bool rz,bool px,bool py,bool pz)
	{
		constraints[0] = rx;
		constraints[1] = ry;
		constraints[2] = rz;
		constraints[3] = px;
		constraints[4] = py;
		constraints[5] = pz;
	}
	int currentDof;
	GsQuat initialRot;
	GsVec initialPos;
	bool _hit_active;
	GsVec2 mousep;
	int scaleManipSelected;
	
protected:
	Model* model;
	SnGroup* grp;
	
	bool _wantsMenu;
	GsVec2 _dialogPoint;
	GsVec start_position;
	GsQuat start_orientation;
	GsVec origin;
	GsVec hitPoint;
	GsQuat frame;


public:
	void setActive(const bool a){setP(manipulator_active,a);}
	bool isActive(){return pBool(manipulator_active);}
	rotation_type rotationOrder(){return getQuatParameter(manipulator_orientation)->rotationOrder; }
	Manipulator(const GsString& name,const GsString& typ, const GsString& file);
	Manipulator(const GsVec& pos,const GsVec& size,const bool simpleManip =false);
	~Manipulator(void);
	void loadStateFromFile(const GsString& file);
	GsString stateString();
	virtual bool evaluate();
	void translation(const GsVec& p);
	void rotation(const GsQuat& q );

	virtual void match();
	virtual void setInitial();

	SnGroup* getGroup(){return grp;}
	GsVec globalPosition();
	GsVec localPosition();
	GsQuat globalOrientation();
	GsQuat localOrientation();
	GsVec getStartPosition(){return origin + frame.apply(start_position);}
	GsQuat getStartOrientation(){return frame*start_orientation;}
	int handle_event ( const GsEvent &e , float t );
	int check_event ( const GsEvent& e, float& t );
	bool hitActive(){return _hit_active;}
	void applyParameters();
	bool wantsPopupMenu(GsVec2* p);
	void setOrigin(const GsVec& pos,const GsQuat& rot );
	void setOffset(const GsVec& v){setP(manipulator_offset,v);}
	void visible(bool v);
	void setColor(const GsColor& c );
	void setSize(const GsVec& sze );
	void setSize(int dof,float sze);
	GsVec size();
	float lineIntersectCircle(Circle* m,const GsLine& l,GsVec* v);
	void updateScale();
	void _transform ( const GsPnt& p, const GsVec& r );
	void updateScaleOffsets();
	int handlePush( const GsEvent & e, float t );
	int handleDrag( const GsEvent & e, float t );
	int handleKeyboard( const GsEvent & e, float t );
};
