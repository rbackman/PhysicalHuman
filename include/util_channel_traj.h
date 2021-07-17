
#include "util_channel.h"



class TrajectoryChannel : public Channel 
{

private:
	Trajectory* _trajectory;
	int phaseToCurveIndex( float t ); //given a phase value it will give the point to the left of the interpolated value
	float getCurveVal(int i);
	

public:
	TrajectoryChannel( Serializable* sav);
	TrajectoryChannel( const GsString& file,const GsString& name);
	TrajectoryChannel(Serializable* objct,int parameter_id, channel_dof_types channelType,trajectory_type trajType,int array_indx = -1);
	Trajectory* getCurve();
	float getVal( float time );
		
	void duration(float d){if(_trajectory)_trajectory->duration = d; _duration = d;}
	//curve can be linear step or spline	
	void setCurveType( trajectory_type t );

	void applyParameters();	   //sets _trajectory from savable data
	void copyFromTrajectory(); //updates savable information from _trajectory

	
	//sample bounds for trajectory randomization
	void randomize();
	void addSample( sample_data p );
	void removeSample( int sampleIdx );

	void makeStep( float phase ); //this is like setting a key but it coppies the sample data to the new point
	//if this curve can be randomized
	bool samples() {return	getCurve()->numSamplePoints()>0;}

		//curve editing
	int setKey(float t); // add a point to c from the currentChannelVal() at the curve.lastP time
	void addPoint(GsVec2 newP);
	void movePt(int idx,float p);
	void clearPoints();
	void fitCurve( TrajectoryChannel* originalCurve, int points, float tolerance, float mergeDist ,float concavity_tolerance,bool flat=false);
	void mergeControlPoints( float tolerance );
void initCurve();

	float getStepPhase( float time,bool reverse = false ); //this is a special function for step functions which will give the relative phase of a step

	~TrajectoryChannel();
	void init();
	
};

class Curve;
class Motion;

class VecTrajectory
{
private:

	Curve* _curve;
	int _parmId;
public:
	enum {TX,TY,TZ};
	GsArray<Channel*> chs;
	Curve* getCurve(){return _curve;}
	GsString objectName;

	GsVec origin;
	GsQuat frame;

	bool valid;

	VecTrajectory(const GsString& name,int id,GsVec orgin,GsQuat frm);
	VecTrajectory(Channel* x,Channel* y,Channel* z);
	~VecTrajectory();

	void setFromMotion(Motion* m);
	void update();

	GsArray<Channel*> getChannels(){return chs;}
	void setObject(const GsString& name, GsVec orgn,GsQuat frm);
	void setOrigin( GsVec og,GsQuat frm );
	void init();
};

