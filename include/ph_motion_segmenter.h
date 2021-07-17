#pragma once
#include "common.h"
#include "util_trajectory.h"
#include "util_serializable.h"

class HumanManager;
class HumanMotionManager;
class HumanMotion;
class Channel;
class VecTrajectory;
class Motion;
class PhysicalHuman;

struct kn_motion_segment
{
	GsVec startVel;
	GsVec endVel;
	GsVec stanceSwingStart;
	GsVec stanceSwingEnd;
	GsVec stanceComStart;
	GsVec stanceComEnd;
	human_stance stanceFoot;
	KnMotion* motion;
	GsColor color;

};
struct motion_fragment
{
	int start_frame;
	int end_frame;
	human_stance stancefoot;
};

enum motion_segment_parms
{
	segment_reduction_merge_distance,
	segment_motion_reduction_slope_tolerance,
	segment_motion_reduction_sample_points,
	segment_motion_reduction_conc_tolerance,
	segment_motion_com_vector,

};
class Box;

class HumanMotionSegmenter : public Serializable
{
private:
	HumanManager* manager;
	float phase;
	human_stance currentStance;

public:
	HumanMotionSegmenter(GsString file);
	~HumanMotionSegmenter();

	GsArray<HumanMotion*> simple_segments;
	GsArray<HumanMotion*> motion_segments;
	GsArray<kn_motion_segment*> kn_segments;

	void setMotion(GsVec pos,GsQuat rot);
	void processMotions(GsString file);
	void reduceMotion(bool flat);
	void selectMotion(int i);
	void segmentMotion(KnMotion* full_motion);
	void extractSteps(GsArray<motion_fragment> *steps, KnMotion* continuous_piece, human_stance stance);
	void extractContinuousPieces( GsArray<KnMotion*> *continuous_pieces, KnMotion* full_motion );
	void translateMotion( int mo );
	void orientMotion( int mo );

	/*convert the motion from joint angles to IK manipulator transformations*/
	void transformMotions(PhysicalHuman* h);
	/*arrange motions so they all start from the origin with the same heading and mirror if necessary so the left foot is the stance*/
	void sortMotions(GsVec skeletonStartPoint);
	/*mirror one of the segments*/
	void mirrorMotion(int i);
	void init(HumanManager* manager);
	void makeMorphCurves();
	void transformMotion(PhysicalHuman* h,KnMotion* m,HumanMotion* newM,GsArray<bool> vals);
};
