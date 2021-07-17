# pragma once

#include "util_serializable.h"
#include "common.h"

enum point_cloud_parms
{
	point_cloud_value,
	point_cloud_points,
};
class PointCloud: public Serializable
{
	GsVec centroid;

public:
	GsArray<GsVec> points;

	PointCloud();
	void load(GsString filename);
	void applyParameters();
	void updateParameters();
	void setPoints(GsArray<GsVec>& p);
	void setValue(float v){
		setP(point_cloud_value,v);
	}
	void calculateCentroid();
	void centerPoints();
	float check( PointCloud* p );
};