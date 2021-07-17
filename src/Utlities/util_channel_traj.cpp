#include "util_channel_traj.h"


TrajectoryChannel::TrajectoryChannel(Serializable* objct,int parameter_id, channel_dof_types channelType,trajectory_type trajType,int array_indx):Channel(objct,parameter_id,channelType,channel_trajectory,array_indx)
{


	if(trajType == TRAJ_LINEAR)
		MAKE_PARM(trajectory_channel_type,"LINEAR");
	else if(trajType == TRAJ_STEP)
		MAKE_PARM(trajectory_channel_type,"STEP");
	else
		MAKE_PARM(trajectory_channel_type,"BEZIER");

	_channel_type = channelType;

	MAKE_PARM(trajectory_channel_p_t,0.0f);
	MAKE_PARM(trajectory_channel_p_y,0.0f);
	
		MAKE_PARM(trajectory_channel_tng,0.5f);

		if(trajType == TRAJ_BEZIER)
			getParameter(trajectory_channel_tng)->save = true;
		else
			getParameter(trajectory_channel_tng)->save = false;

	MAKE_PARM(trajectory_channel_sample,false);
	_trajectory = new Trajectory(trajType);
	channel_mode = channel_trajectory;
	_isTrajectory = true;
	pFloatArray(trajectory_channel_p_t)->size(0); 
	pFloatArray(trajectory_channel_p_t)->size(0);

	MAKE_PARM(trajectory_channel_loop_continuity,false);
	MAKE_PARM(trajectory_channel_loop_flip,false);
	MAKE_TEMP_PARM(trajectory_channel_sample_indexes,-1);
	MAKE_TEMP_PARM(trajectory_channel_sample_values,0.0f);
	MAKE_TEMP_PARM(trajectory_channel_curve_color,GsColor::blue);
	
	MAKE_TEMP_PARM(trajectory_channel_lock_reconfigure,false);
	MAKE_TEMP_PARM(trajectory_channel_curve_width,3);
	MAKE_TEMP_PARM(trajectory_channel_point_color,GsColor::red);
	MAKE_TEMP_PARM(trajectory_channel_point_size,7);
	MAKE_TEMP_PARM(trajectory_channel_show_tangents,true);
	MAKE_TEMP_PARM(trajectory_channel_show_control_points,true);

	initCurve();
}

TrajectoryChannel::TrajectoryChannel( const GsString& file,const GsString& nme ):Channel(file,nme)
{
	_trajectory = new Trajectory(TRAJ_BEZIER);
	init();
	_isTrajectory = true;
}
void TrajectoryChannel::init()
{
	Channel::init();
	CHECK_FLOAT(trajectory_channel_p_t);
	CHECK_FLOAT(trajectory_channel_p_y);
	CHECK_FLOAT(trajectory_channel_tng);
	CHECK_BOOL(trajectory_channel_sample);
	CHECK_STRING(trajectory_channel_type);
	CHECK_BOOL(trajectory_channel_loop_continuity);
	CHECK_BOOL(trajectory_channel_loop_flip);
	CHECK_INT(trajectory_channel_sample_indexes);
	CHECK_FLOAT(trajectory_channel_sample_values);
	CHECK_COLOR(trajectory_channel_curve_color);
	CHECK_INT(trajectory_channel_curve_width);
	CHECK_INT(trajectory_channel_point_size);
		CHECK_COLOR(trajectory_channel_point_color);
	CHECK_BOOL(trajectory_channel_show_tangents);
	CHECK_BOOL(trajectory_channel_show_control_points);
	CHECK_BOOL(trajectory_channel_lock_reconfigure);
	setP(trajectory_channel_curve_width,2);
	setP(trajectory_channel_point_size,8);

	verifyParameters();
	_isTrajectory = true;
	//channel_mode = channel_trajectory;
	_trajectory->init();
	if (pString(trajectory_channel_type)=="LINEAR")
	{
		_trajectory->setCurveType(TRAJ_LINEAR);
		getParameter(trajectory_channel_tng)->save = false;
// 		for(int i=0;i<pFloatArray(trajectory_channel_p_t)->size();i++)
// 		{
// 			_trajectory->addPoint(GsVec2(pFloat(trajectory_channel_p_t,i),pFloat(trajectory_channel_p_y,i)));
// 		}

	}
	else if (pString(trajectory_channel_type)=="STEP")
	{
		
		_trajectory->setCurveType(TRAJ_STEP); 
		getParameter(trajectory_channel_tng)->save = false;
		_channel_type = CH_BOOL;
// 		for(int i=0;i<pFloatArray(trajectory_channel_p_t)->size();i++)
// 		{
// 			_trajectory->addPoint(GsVec2(pFloat(trajectory_channel_p_t,i),pFloat(trajectory_channel_p_y,i)));
// 		}

	}
	else if(pString(trajectory_channel_type) == "BEZIER")
	{
		setP(trajectory_channel_show_tangents,true);
		setP(trajectory_channel_show_control_points,true);
		
		_trajectory->setCurveType(TRAJ_BEZIER); 
// 		for(int i=0;i<pFloatArray(trajectory_channel_p_t)->size();i++)
// 		{
// 			float tg = 0;
// 			if(i<pFloatArray(trajectory_channel_tng)->size())
// 				tg = pFloat(trajectory_channel_tng,i);
// 			_trajectory->addPoint(GsVec2(pFloat(trajectory_channel_p_t,i),pFloat(trajectory_channel_p_y,i)), tg);
// 		}
	}
	else
	{
		phout<<"_trajectory is null\n";
		_trajectory = 0;
	}
	applyParameters();
// 	if(getIntParameter(trajectory_channel_sample_indexes)->save)
// 	{
// 		if(getIntParameter(trajectory_channel_sample_indexes)->val.size()*4 == getFloatParameter(trajectory_channel_sample_values)->val.size())
// 		{
// 			for (int i=0;i<getIntParameter(trajectory_channel_sample_indexes)->val.size();i++)
// 			{
// 				sample_data d;
// 				d.index = pInt(trajectory_channel_sample_indexes,i);
// 				d.rest = _trajectory->getPoint(d.index);
// 			/*	d.rest.y +=crest();*/
// 				d.min.x = pFloat(trajectory_channel_sample_values,i*4);
// 				d.max.x = pFloat(trajectory_channel_sample_values,1+i*4);
// 				d.min.y = pFloat(trajectory_channel_sample_values,2+i*4);
// 				d.min.y = pFloat(trajectory_channel_sample_values,3+i*4);
// 				_trajectory->addSample(d);
// 			}
// 		}
// 		else
// 		{
// 			phout<<"limits don't match for channel "<< name() <<gsnl;
// 		}
// 	}
}
TrajectoryChannel::TrajectoryChannel( Serializable* sav ):Channel(sav)
{
	_trajectory = new Trajectory(TRAJ_BEZIER);
	init();
	_isTrajectory = true;
}
void TrajectoryChannel::addPoint(GsVec2 newP)
{
	int idx = 0;
	idx = _trajectory->addPoint(newP);

}
void TrajectoryChannel::makeStep( float t )
{
	if(samples())
	{
		sample_data d = _trajectory->getSample(0);
		_trajectory->removeSamples();
		int i = setKey(t);
		d.index = i;
		getCurve()->addSample(d);
	}
}

int TrajectoryChannel::setKey(float time)
{
	GsPnt2 p;
	if(getObject())
	{
		p = GsPnt2(time,currentChannelVal());
	}
	else
	{
		p = GsPnt2(time,getVal(time));
	}

	if(getCurve()->getCurveType() == TRAJ_BEZIER)
		return _trajectory->addPoint( p,FREE_TNG);
	else
		return _trajectory->addPoint( p);

}

void TrajectoryChannel::copyFromTrajectory()
{
	FloatParameter* fx = getFloatParameter(trajectory_channel_p_t);
	FloatParameter* fy = getFloatParameter(trajectory_channel_p_y);
	fx->val.size(0);
	fy->val.size(0);

	if(_trajectory)
	{
		_trajectory->update();
		for (int i=0;i<_trajectory->numPoints();i++)
		{
			GsVec2 pt = _trajectory->getPoint(i);
			fx->val.push(pt.x);
			fy->val.push(pt.y);
		}

		if(_trajectory->getCurveType()==TRAJ_BEZIER)
		{
			FloatParameter* ti = getFloatParameter(trajectory_channel_tng);
			if(ti)
			{
				ti->val.size(0);
				for (int i=0;i<_trajectory->numPoints();i++)
				{
					float tng = _trajectory->getTangent(i);
					ti->val.push(tng);
				}
			}
			else
				phout<<"this channel doesnt have tang parm even though its _trajectory does\n";
		}

	}

	getIntParameter(trajectory_channel_sample_indexes)->val.size(_trajectory->numSamplePoints());
	getFloatParameter(trajectory_channel_sample_values)->val.size(6*_trajectory->numSamplePoints());

	for (int i=0;i<_trajectory->numSamplePoints();i++)
	{
		sample_data* d = _trajectory->sample(i);
		setP(trajectory_channel_sample_indexes,d->index,i);
		
		setP(trajectory_channel_sample_values,d->rest.x,i*6);
		setP(trajectory_channel_sample_values,d->rest.y,i*6+1);
		setP(trajectory_channel_sample_values,d->min.x ,i*6+2);
		setP(trajectory_channel_sample_values,d->max.x ,i*6+3);
		setP(trajectory_channel_sample_values,d->min.y ,i*6+4);
		setP(trajectory_channel_sample_values,d->max.y ,i*6+5);
	}
	if(_trajectory->numSamplePoints()>0)
	{
		getIntParameter(trajectory_channel_sample_indexes)->save = true;
		getFloatParameter(trajectory_channel_sample_values)->save = true;
	}
	else
	{
		getIntParameter(trajectory_channel_sample_indexes)->save = false;
		getFloatParameter(trajectory_channel_sample_values)->save = false;
	}
}

void TrajectoryChannel::randomize()
{
	if(samples())
	{
		_trajectory->randomize();

		copyFromTrajectory();
	}
}




void TrajectoryChannel::movePt(int idx,float val)
{
	if(idx<0 || idx >=_trajectory->numPoints())
	{
		phout<<"trying to move a point that doesnt exist\n";
		return;
	}
	GsVec2 p = _trajectory->getPoint(idx);
	p.y = val;
	_trajectory->setPoint(idx,p);
	copyFromTrajectory();
}

void TrajectoryChannel::applyParameters()
{

	//phout<<"verify channel applyParm\n";
	FloatParameter* fx = getFloatParameter(trajectory_channel_p_t);
	FloatParameter* fy = getFloatParameter(trajectory_channel_p_y);
	FloatParameter* tng = 0;
	if(!_trajectory)
	{
		phout<<"applyParameters() TrajChanel: "<<name()<<"has no trajectory\n";
		return;
	}
	if(_trajectory->getCurveType()==TRAJ_BEZIER)
		tng = getFloatParameter(trajectory_channel_tng);

	_trajectory->init();

	_trajectory->setRange(cmin(),crest(),cmax());
	if(pBool(trajectory_channel_loop_continuity))
	{
		fy->val[fy->val.size()-1] = fy->val[0];
	}
	for (int i=0;i<fx->val.size();i++)
	{
		if(tng && i<tng->val.size())
		{
			_trajectory->addPoint(GsVec2(fx->val[i],fy->val[i]),tng->val[i]);
		}
		else
			_trajectory->addPoint(GsVec2(fx->val[i],fy->val[i]));
	}
	_trajectory->loopContinuity = pBool(trajectory_channel_loop_continuity);
	_trajectory->flip = pBool(trajectory_channel_loop_flip);
	_trajectory->rest = crest();
	_trajectory->curve_color = pColor(trajectory_channel_curve_color);
	_trajectory->curve_width = pInt(trajectory_channel_curve_width);
	_trajectory->point_color = pColor(trajectory_channel_point_color);
	_trajectory->show_control_points = pBool(trajectory_channel_show_control_points);
	_trajectory->show_tangents = pBool(trajectory_channel_show_tangents);
	_trajectory->point_size = pInt(trajectory_channel_point_size);

	_trajectory->update();

	//phout<<"traj applyPar "<<toString()<<gsnl;
	if(getIntParameter(trajectory_channel_sample_indexes)->save)
	{
		if(getIntParameter(trajectory_channel_sample_indexes)->val.size()*6 == getFloatParameter(trajectory_channel_sample_values)->val.size())
		{
			for (int i=0;i<getIntParameter(trajectory_channel_sample_indexes)->val.size();i++)
			{
				sample_data d;

				d.index  = pInt(trajectory_channel_sample_indexes,i);
				d.rest.x = pFloat(trajectory_channel_sample_values,i*6);
				d.rest.y = pFloat(trajectory_channel_sample_values,i*6+1);
				d.min.x  = pFloat(trajectory_channel_sample_values,i*6+2);
				d.max.x  = pFloat(trajectory_channel_sample_values,i*6+3);
				d.min.y  = pFloat(trajectory_channel_sample_values,i*6+4);
				d.max.y  = pFloat(trajectory_channel_sample_values,i*6+5);
				addSample(d);

			}
		}
		else
		{
			phout<<"limits don't match for channel "<<name()<<gsnl;
		}
	}
}

TrajectoryChannel::~TrajectoryChannel()
{
	if (_trajectory)
	{
		delete _trajectory;
	}
}
Trajectory* TrajectoryChannel::getCurve()
{
	return _trajectory;
}

float TrajectoryChannel::getStepPhase( float time ,bool reverse)
{
	
	for (int i=1;i<=_trajectory->numPoints();i++)
	{
		if(_trajectory->getPoint(i).x > time || i == _trajectory->numPoints())
		{
			if((_trajectory->getPoint(i-1).y>0 && !reverse) || (_trajectory->getPoint(i-1).y<=0 && reverse))
			{
				float firstV = _trajectory->getPoint(i-1).x;
				float secondV;
				if(i==_trajectory->numPoints())
					secondV = _trajectory->duration; //getCurvePoint(_trajectory->numCurvePoints()-1).x;
				else
					secondV = _trajectory->getPoint(i).x;

				float distV = ( secondV - firstV);
				if(distV>0)
				{
					float to = (time - firstV )/distV;
					return to;
				}
			}
		}
	}
	
	//phout<<"getStepPhase didn't work right" << _trajectory->numPoints()<< "\n";
	return 0;
}

float TrajectoryChannel::getVal( float time )
{
	float v =	getCurve()->getOutput(time);
	if(_flip && flips())
	{
		v= -v + 2*crest();
	}

	if(channel_mode == channel_trajectory)
	{
		return v;
	}
	else if(channel_mode == channel_additive)
	{
		v = v+ Channel::getVal(time);
	}
	else if(channel_mode == channel_multiply)
	{
		v = v*Channel::getVal(time);
	}
	else if(channel_mode == channel_inverse)
	{
		v = -v;
	}
	return v;
}



void TrajectoryChannel::addSample( sample_data p )
{
	_trajectory->addSample(p);
}

void TrajectoryChannel::removeSample( int sampleIdx )
{
	_trajectory->removeSample(sampleIdx);
	if(_trajectory->numSamplePoints()>0)
	{
		getIntParameter(trajectory_channel_sample_indexes)->save = true;
		getFloatParameter(trajectory_channel_sample_values)->save = true;
	}
	else
	{
		getIntParameter(trajectory_channel_sample_indexes)->save = false;
		getFloatParameter(trajectory_channel_sample_values)->save = false;
	}
}

void TrajectoryChannel::clearPoints()
{
	if(_trajectory)
		_trajectory->init();

	getIntParameter(trajectory_channel_sample_indexes)->save = false;
	getFloatParameter(trajectory_channel_sample_values)->save = false;
	getIntParameter(trajectory_channel_sample_indexes)->val.size(0);
	getFloatParameter(trajectory_channel_sample_values)->val.size(0);
	pFloatArray(trajectory_channel_p_t)->size(0);
	pFloatArray(trajectory_channel_p_y)->size(0);
	pFloatArray(trajectory_channel_tng)->size(0);
	applyParameters();
}



void TrajectoryChannel::setCurveType( trajectory_type t )
{
	if(!_trajectory)
		return;

	_trajectory->setCurveType(t);

	if(t==TRAJ_BEZIER)
	{
		getParameter(trajectory_channel_tng)->save = true;
		setP(trajectory_channel_type,"BEZIER");
	}
	else
	{
		getParameter(trajectory_channel_tng)->save = false;
	}


	if(t== TRAJ_LINEAR)
	{
		setP(trajectory_channel_type,"LINEAR");
	}
	else if(t==TRAJ_STEP)
	{
		setP(trajectory_channel_type,"STEP");
	}

}

void TrajectoryChannel::mergeControlPoints( float tolerance )
{
	if(!_trajectory)
		return;

	_trajectory->mergeControlPoints(tolerance);

	copyFromTrajectory();
}

void TrajectoryChannel::fitCurve( TrajectoryChannel* originalCurve, int numpoints, float tolerance, float mergeDist ,float concavity_tolerance,bool flat)
{


	clearPoints();


	Trajectory* traj = originalCurve->getCurve();
	setCurveType(TRAJ_BEZIER);
	_trajectory->loopContinuity = false;

	//this is a list of the control points that are needed to add
	GsArray<GsVec2> points;
	GsArray<float> tngts;
	float slope = traj->avgSlope(0,numpoints);
	float concavity = traj->avgSlope(1,numpoints) - slope;

	bool concaveUp;
	bool pos;
	if(concavity>0)
		concaveUp = true;
	else
		concaveUp = false;

	if(slope>0)
		pos = true;
	else
		pos = false;


	for (int j=0;j<traj->numCurvePoints();j++)
	{
		float nslope = traj->avgSlope(j,numpoints);
		if(j<traj->numCurvePoints()-2)
		{
			float nconcavity = traj->avgSlope(j+1,numpoints) - nslope;
			if(concaveUp==true && nconcavity < -concavity_tolerance)
			{
				//phout<<"pushed neg slope "<<j<<gsnl;
				points.push(traj->getCurvePoint(j));
				tngts.push(FREE_TNG);
				concaveUp = false;
			}
			else if(concaveUp==false && nconcavity > concavity_tolerance)
			{
				//phout<<"pushed pos slope "<<j<<gsnl;
				points.push(traj->getCurvePoint(j));
				tngts.push(FREE_TNG);
				concaveUp = true;
			}
		}
		if(pos==true && nslope < -tolerance)
		{
			//phout<<"pushed neg slope "<<j<<gsnl;
			points.push(traj->getCurvePoint(j));
			tngts.push(0.5f);
			pos = false;
		}
		else if(pos==false && nslope > tolerance)
		{
			//phout<<"pushed pos slope "<<j<<gsnl;
			points.push(traj->getCurvePoint(j));
			tngts.push(0.5f);
			pos = true;
		}
		else if( j == traj->numCurvePoints()-1 || j==0)
		{
			//phout<<"pushed beg/end slope "<<j<<gsnl;
			points.push(traj->getCurvePoint(j));
			tngts.push(0.5f);
		}
	}

	//phout<<"found "<<points.size()<<" max and mins for curve "<<originalCurve->name()<<gsnl;

	for (int j=0;j<points.size();j++)
	{
		if(flat)
		{
			_trajectory->addPoint(GsVec2(points[j].x,0.0f),FLAT_TNG);
			setP(channel_range,0.0f,1);
			originalCurve->setControlCurve(this);
		}
		else
		{
			_trajectory->addPoint(points[j],tngts[j]);
		}

	}

	mergeControlPoints(mergeDist);

	if(!flat)
	{
		for (int j=0;j<_trajectory->numPoints()-1;j++)
		{
			if(_trajectory->getTangent(j)>=0)
			{
				GsVec2 p1 = _trajectory->getPoint(j);
				GsVec2 p2 = _trajectory->getPoint(j+1);

				int startIndx = (int)(p1.x*traj->numCurvePoints())+2;
				int endIndx	  =	(int)(p2.x*traj->numCurvePoints())-2;

				GsVec2 p1t = GsVec2(p1.x,p2.y);
				GsVec p2t = GsVec2(p2.x,p1.y);
				float p12dist = dist(p1t,p2t);
				float p1tdist =0;
				float p2tdist =0;
				for(int k=startIndx;k<endIndx;k++)
				{
					GsVec2 pn = traj->getCurvePoint(k);
					p1tdist += dist(pn,p1t);
					p2tdist += dist(pn,p2t);
				}
				float v =0.5f;
				//	phout<<"p1dist "<<p1tdist<<" p2dist "<<p2tdist<< " p12dist "<<p12dist<<gsnl;
				if(p1tdist<p2tdist && p12dist>0)
				{
					v = 0.5f - 2.0f*(p2tdist - p1tdist)/(p2tdist+p1tdist);
				}
				else if(p1tdist>p2tdist && p12dist>0)
				{
					v = 0.5f + 2.0f*((p1tdist - p2tdist)/(p2tdist+p1tdist)); //p12dist);
				}
				//phout<<"v before limits "<<v<<gsnl;
				if(v<0)v=0;
				if(v>1)v=1;

				//phout<<" calculated weight "<<v<<gsnl<<gsnl;
				_trajectory->setTangent(j,v);
			}
		}
	}
	copyFromTrajectory();


}


float TrajectoryChannel::getCurveVal(int i)
{
	float val =crest();
	if(i>=0&& i < _trajectory->numCurvePoints())
	{
		val = _trajectory->getCurvePoint(i).y;
		if(flips()&&_flip)
			val*=-1;
	}
	return val;
}

void TrajectoryChannel::initCurve()
{
	clearPoints();
	addPoint(GsVec2(0.0f,crest() ));
	//addPoint(GsVec2(1.0f,crest()) );
	copyFromTrajectory();
}

int TrajectoryChannel::phaseToCurveIndex( float t )
{
	return (int)(t*(getCurve()->numCurvePoints()-1));
}



