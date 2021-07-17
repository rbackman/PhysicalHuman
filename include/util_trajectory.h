

#include "common.h"
#pragma once

#define MAX_X 1
#define MIN_X 0
#define MAX_Y 1
#define MIN_Y -1
// between every two points there is a bezier curve

enum trajectory_type
{
	TRAJ_BEZIER,
	TRAJ_LINEAR,
	TRAJ_STEP
};
enum curve_state
{
	CURVE_NOT_SELECTED,
	CURVE_POINT_SELECTED,
	CURVE_TANG_SELECTED,
	CURVE_TANG_PI,
	CURVE_TANG_PO,
	CURVE_ADD_POINT,
	CURVE_REMOVE_POINT,
	CURVE_MOVE_POINT,
	CURVE_FLATTEN_TANGENT,
	CURVE_STRAIGHTEN_TANGENT,
	CURVE_FREE_TANGENT,
	CURVE_EDIT_BOUNDS,
	CURVE_NEEDS_UPDATE
};

#define STRAIGHT_TNG -1.0f
#define FREE_TNG -2.0f
#define FLAT_TNG 0.5f


typedef struct _sample_data
{
	int index;
	GsVec2 rest;
	GsVec2 min;
	GsVec2 max;
}sample_data;
class Trajectory 
{
private:
	GsArray<GsVec2> p; /*!control points.*/
	GsArray<GsVec2> c; /*!curve points*/
	GsArray<float> t; /*!tangents*/
	GsArray<sample_data> _sample_limits;
	trajectory_type _curveType; 
public:
	GsColor curve_color;
	GsColor point_color;
	int curve_width;
	int point_size;
	bool show_tangents;
	bool show_control_points;

	~Trajectory();
	 Trajectory(trajectory_type type);
	
	 int numSamplePoints();
	 sample_data* sample( int i );
	 void addSample( sample_data p );
	 void removeSample( int sampleIdx );

	 int addPoint(GsVec2 pt,float pi);
	void init();

	bool loopContinuity; //the last control point should match with the first one
	bool flip;			 //if loopContinuity is true the last controlPoint will be the negative of the first
	
	float rest; //this is where the flipping happens
   float max,min; /*!max and min output*/

   float duration;
   float dt;

   //this is the alternate curve that is connected to this one ex(StanceUpLeg SwingUpLeg) set to zero if it doesnt exist
   Trajectory* complement;

	//for selection
	curve_state selectionState; //0:not selected 1:pt selected 2: tangent selected
	curve_state tangSelected; /*!0:no tang selected 1:pi selected 2:po selected*/
	int selection;  /*!the current selected control vertex*/
	
	
	/*this is the last curve point that was solved for in curve space*/
	GsVec2 lastP;
	/*! the last phase that created lastP from [0,1] */
	float lastPhase;

	/*!*/
	void removePoint(int i);
	/*!*/

   int addPoint(GsVec2 p);
   /*!*/
   void update();
   /*!*/
   void draw();
	/*!*/
	bool press(GsVec2 pt);
	/*!*/
	bool drag(GsVec2 pt);
	/*!*/
	GsVec2 curvePoint( float t);

	/*!midpoint*/
	float exactOutput(float x,float tol);
	/*!output based on 'c' array*/
	float getOutput(float x);

	float getClosestOutput(float x);

	GsVec2 getPiPos(int i);
    GsVec2 getPoPos(int i);
	void mergeControlPoints( float tolerance );
	float avgSlope( int i,int num);
	GsVec2 getPoint( int i );
	int numPoints();
	float getTangent(int i);
	void setPoint( int idx, GsVec2 np );
	trajectory_type getCurveType();
	void setCurveType( trajectory_type t );
	int numCurvePoints();
	GsVec2 getCurvePoint( int i );
	void setTangent( int j, float wt );
	void setRange( float min,float rest, float max );
	void randomize();
	

	void removeSamples();
	sample_data getSample( int i );
	void scale( float v,bool scaleY = false );
	GsArray<GsVec2>* getCurveArray();
	int numTangents();
};

	//this is a specal case where every bezier curve will have 4 points.. two tangents and two control points
	//for more generic curves use curve.h

	float BezStuff(int n, int i, float t);
	GsVec2 evalBez ( float t, GsVec2 p1, GsVec2 p2, GsVec2 p3, GsVec2 p4);
	float avgSlope(int i,int num, Trajectory* traj);