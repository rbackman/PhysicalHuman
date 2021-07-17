
#include "point_cloud.h"

PointCloud::PointCloud():Serializable("PointCloud")
{
	MAKE_PARM(point_cloud_value,0.0f);
	MAKE_PARM(point_cloud_points,0.0f);
}

void PointCloud::load( GsString filename )
{
	GsString imagename = filename;
	remove_extension(imagename);
	imagename<<".bmp";

	loadParametersFromFile(filename);

	CHECK_FLOAT(point_cloud_value);
	CHECK_FLOAT(point_cloud_points);
	applyParameters();
}
void PointCloud::applyParameters()
{
	for (int i=0;i<sizeOfParameter(point_cloud_points);i+=3)
	{
		points.push(GsVec(pFloat(point_cloud_points,i),pFloat(point_cloud_points,i+1),pFloat(point_cloud_points,i+2)));
	}

}
void PointCloud::updateParameters()
{
	getFloatParameter(point_cloud_points)->val.size(0);
	for (int i=0;i<points.size();i++)
	{
		getFloatParameter(point_cloud_points)->val.push(points[i].x);
		getFloatParameter(point_cloud_points)->val.push(points[i].y);
		getFloatParameter(point_cloud_points)->val.push(points[i].z);
	}
}

void PointCloud::setPoints( GsArray<GsVec>& p )
{
	points.size(0);
	points.push(p);

	centerPoints();
	updateParameters();
}

void PointCloud::calculateCentroid()
{
	if(points.size()>0)
	{
		GsVec p;
		for (int i=0;i<points.size();i++)
		{
			p +=points[i];
		}
		p/=(float)points.size();
		centroid = p;
	}
}

void PointCloud::centerPoints()
{
	calculateCentroid();
	//phout<<"start centroid "<<centroid<<gsnl;
	for (int i=0;i<points.size();i++)
	{
		points[i] = points[i] - centroid;
	}
	calculateCentroid();
	//phout<<"end centroid "<<centroid<<gsnl;
}

float PointCloud::check( PointCloud* p )
{
	float d = 0;
	float md = 0;
	float newD = 0;
	for (int i=0;i<points.size();i++)
	{
		md = 20000;
		for(int j=0;j<p->points.size();j++)
		{
			newD = dist(points[i],p->points[j]);
			if(newD<md)
				md = newD;
		}
		d+=md;
	}
	d/= points.size();
	return d;
}

