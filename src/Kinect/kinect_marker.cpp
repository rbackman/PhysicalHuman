#include "kinect_marker.h"

#include "util.h"



KinectMarker::~KinectMarker()
{

	delete model;
	

}
KinectMarker::KinectMarker(GsString n,GsColor col):Serializable(n)
{
	MAKE_PARM(blob_type,"default_type");
	MAKE_PARM(blob_color,col);
	MAKE_PARM(blob_position,GsVec());
	
	model = new Ball(GsVec(0,0,0),0.02f,col);
}

KinectMarker::KinectMarker( GsString filename,GsString mname ):Serializable(mname)
{
	loadParametersFromFile(filename,mname);
	CHECK_STRING(blob_type);
	CHECK_COLOR(blob_color);
	CHECK_VEC(blob_position);
	model = new Ball(GsVec(0,0,0),0.02f,pColor(blob_color));
	model->setPosition(pVec(blob_position));
}

void KinectMarker::check(GsColor col, GsVec pos,int var)
{
	if(col_dist(pColor(blob_color),col)<var)
	{
		points.push(pos);
	}
}

void KinectMarker::update()
{
		
	if(points.size()==0)return;

	p.set(0,0,0);

	for(int i=0;i<points.size();i++)
	{
		p += points[i];
	}

	if(points.size()>1)
		p /= (float)points.size();

	/*
	GsVec pFiltered;
	int num = 0;
	for(int i=0;i<points.size();i++)
	{
		if(dist(points[i],p)>min_cut/100.0f)
		{
			
		}
		else
		{
			pFiltered+=points[i];
			num++;
		}
	}

	if(num=0)
	{
		phout<<"filtered them all\n";
			return;
	}
	p = pFiltered/num;
	phout<<"trimmed "<<points.size() - num<<" outliers "<<gsnl;
	*/
		
	newP = p - position;

	if(newP.len()>max_v)
		newP.len(max_v);

	newP = position+newP;
	
	position = position*(1.0f-sample) +newP*(sample);
	model->setPosition(position);
	

}

GsColor KinectMarker::getColor()
{
	return pColor(blob_color);
}
