#pragma once;

#include "common.h"
#include "util_models.h"
#include "util_serializable.h"

enum blob_parms
{
	blob_type,
	blob_position,
	blob_color,
};

class KinectMarker : public Serializable
{
	GsVec newP ;
	GsVec p;
public:
	GsVec position;
	Ball* model;
	GsVec dis;

	GsArray<GsVec> points;
	float sample;
	float max_v;
	KinectMarker(GsString n,GsColor col);
		KinectMarker(GsString filename,GsString mname);
	void check(GsColor color, GsVec pos,int var);
	void update();
	~KinectMarker();
	GsColor getColor();
};