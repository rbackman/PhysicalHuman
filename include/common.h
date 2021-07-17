#pragma once

#include <math.h>
#include <gsim/gs_vec.h>
#include <gsim/gs_vars.h>
#include <gsim/gs_array.h>
#include <gsim/gs_euler.h>
#include <gsim/gs_polygon.h>
#include <gsim/gs_model.h>
#include <gsim/gs_quat.h>
#include <gsim/gs_ogl.h>
#include <gsim/kn_posture.h>
#include <gsim/kn_skeleton.h>
#include <gsim/kn_scene.h>
#include <gsim/kn_motion.h>
#include <gsim/sn_manipulator.h>
#include <gsim/sn_transform.h>
#include <gsim/sn_model.h>
#include <gsim/sn_lines.h>
#include <gsim/sn_group.h>

#include <gsim/sn_manipulator.h>
#include <gsim/gs_geo2.h>
#include <gsim/gs_box.h>

#include <gsim/gs_vec.h>

#include <gsim/gs_quat.h>
#include <gsim/gs_mat.h>
#include <gsim/gs_quat.h>
#include <gsim/gs_euler.h>
#include <gsim/kn_joint.h>
#include <gsim/gs_image.h>


#define DEBUG_PRINT
#define VERIFY_PARAMETERS

//#define PRINT_CONSTRUCTORS 
#define phout gsout
//#define phout //

#ifdef WIN32
#define SLASH "\\"
#else
#define SLASH "\/"
#endif

enum rotation_type
{
	ROT_XYZ,
	ROT_XZY,
	ROT_ZYX,
	ROT_ZXY,
	ROT_YXZ,
	ROT_YZX
};


enum human_stance
{
	STANCE_RIGHT = 0,
	STANCE_LEFT
};



