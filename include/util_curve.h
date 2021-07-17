//This is a 3D curve class


# pragma once

#include "common.h"
#include "util.h"

class Curve 
 { public :

	enum curve_types
	{
		LEGRANGE,
		BEZIER,
		BSPLINEQUAD,
		BSPLINECUBIC,
		LINEAR
	};

	~Curve();
	/*! Curve constructor n*/
	Curve();

   void update();
   int addPoint(GsVec pt);/*!add a point and return the index*/
   void clear();

   GsArray<GsVec> p; /*!control points*/
   GsArray<GsVec> c; /*!curve points*/
   
   GsColor cpCol;/*!color for control poly*/ 
    GsColor cCol;/*!color for curve*/ 

	bool vis;
	int first;  /*!for initialization*/
	int divPerPair;		 //measure of curve resolution
	int closed;

	//for selection
	int selectionState; //0:not selected 1:pt selected 2: tangent selected
	unsigned selection;  //the current selected control vertex
	
	int controlPoly;
	float curveTime;
	float T; //parametric curve length

	
	GsVec eval_bezier   ( float t);
	GsVec eval_lagrange ( float t);
	GsVec eval_bspline ( float u,int k);

	 SnLines* _line;
	 int curveMode;		 /*!LEGRANGE,BEZIER,BSPLINEQUAD,BSPLINECUBIC,BEZIERPIECES*/
  
 private:
	
 
 };

