
#pragma once

#include "common.h"

#include "ph_manip.h"

/* this manipulator started as an IKManipulator with IKBody from graphsim but I changed it to fit this project.
it uses graphsim KnIK <gsim/kn_ik.h>*/

class KnIk;

class IkManipulator : public HumanManipulator
 { public :
  
   protected :
    KnIk* _ik;
    SnLines* _lines;

   public :
	~IkManipulator ();

	/*Savable method*/
	void applyParameters();

    /*! Constructor */
    IkManipulator (IKModule* ikc, KnJoint* j, GsString file);

    /*! will turn on or off the drawing of the IK lines */
    void lines ( bool b );

    /*! Access to the used IK object */
    KnIk* ik() { return _ik; }
    
    /*! Update axis position and call iksolve() as if the goal was moved */
    bool evaluate ();

	void solve();


		GsString stateString();

 };

