#include "ph_motion_manager.h"
#include "ph_motion_segmenter.h"

#include "util_channel.h"
#include "util_curve.h"
#include "ph_motion.h"
#include "util_models.h"
#include "ph_manager.h"
#include "ph_file_manager.h"
#include "ph_human.h"
#include "util_channel_traj.h"
#include "ph_mod_ref.h"

HumanMotionSegmenter::HumanMotionSegmenter(GsString file):Serializable("HumanMotionSegmenter")
{
	loadParametersFromFile(file);

	CHECK_FLOAT(segment_reduction_merge_distance);
	CHECK_FLOAT(segment_motion_reduction_slope_tolerance);
	CHECK_INT(segment_motion_reduction_sample_points);
	CHECK_FLOAT(segment_motion_reduction_conc_tolerance);
	MAKE_TEMP_PARM(segment_motion_com_vector,GsVec());

	verifyParameters();

	phase = 0;
}
void HumanMotionSegmenter::init(HumanManager* mgr)
{
	
	manager = mgr;

}


HumanMotionSegmenter::~HumanMotionSegmenter()
{
	for(int i=0;i<simple_segments.size();i++)
	{
		//delete these
// 		GsArray<HumanMotion*> simple_segments;
// 		GsArray<HumanMotion*> motion_segments;
// 		GsArray<kn_motion_segment*> kn_segments;
	}

}


void HumanMotionSegmenter::processMotions(GsString file)
{
	KnSkeleton* sk = manager->selectedCharacter()->skref();
	GsString motionFile = manager->getFiles()->getKinematicsFile(file);
	/*new KnSkeleton;

	if(!sk->load(motionFile))
	{
		phout<<"couldnt load skeleton "<<motionFile<<gsnl;
	}
	*/
	KnMotion* full_motion = new KnMotion;

	if(!full_motion->load(motionFile))
	{
		phout<<"continuous_piece load failed for "<<motionFile<<gsnl;
		return;
	}
	full_motion->connect(sk);

	scale_motion(full_motion,0.01f);

	segmentMotion(full_motion);
	sortMotions(manager->selectedCharacter()->reference_module()->pVec(reference_skeleton_position));
	transformMotions(manager->selectedCharacter());

	//reduceMotion(true);
	
}
//0.134 0.328 -0.096
//0.734 0.147 -0.047 -0.661
//#define SAVE_ORIGINAL_MOTION

void HumanMotionSegmenter::segmentMotion(KnMotion* full_motion)
{
	/* 
	this function will break a motion into a bunch of continuous_pieces each piece is defined by the portion where
	the heel of the swing foot first starts to raise(toe off) and ends when the toe once again reaches its 
	original height(toe down). the way I have it the clips will be overlapping if the walk is dynamic:
	meaning the toe off of one side comes before the toe down of the other side. this will need to be 
	accounted for somehow later
	*/
	
	
	for(int i=0;i<kn_segments.size();i++)
	{
		delete kn_segments.get(i)->motion;
		delete kn_segments.get(i);
	}
	kn_segments.size(0);


	GsArray<motion_fragment> steps;
	steps.push();
	steps.top().start_frame=0;
	steps.top().end_frame = 72;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=80;
	steps.top().end_frame = 143;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=160;
	steps.top().end_frame = 208;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=258;
	steps.top().end_frame = 327;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=340;
	steps.top().end_frame = 398;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=410;
	steps.top().end_frame = 483;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=550;
	steps.top().end_frame = 625;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=634;
	steps.top().end_frame = 699;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=715;
	steps.top().end_frame = 765;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=766;
	steps.top().end_frame = 836;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=850;
	steps.top().end_frame = 905;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=915;
	steps.top().end_frame = 984;
	steps.top().stancefoot = STANCE_LEFT;


	steps.push();
	steps.top().start_frame=985;
	steps.top().end_frame = 1064;
	steps.top().stancefoot = STANCE_LEFT;

	steps.push();
	steps.top().start_frame=1076;
	steps.top().end_frame = 1138;
	steps.top().stancefoot = STANCE_LEFT;



	for (int s = 0; s < steps.size(); s++ ) 
	{
			
			//now for each one of the steps bake it to its own KnMotion
			motion_fragment* fragment = &steps[s];
			kn_motion_segment* segment = new kn_motion_segment;
			segment->motion = new KnMotion ;
			segment->motion->connect(full_motion->skeleton());
			GsString name = "Motion_";
			name<<s;

			segment->motion->name(name);

			segment->color = GsColor::blue;
			segment->stanceFoot = fragment->stancefoot;
			int start = fragment->start_frame;
			int end = fragment->end_frame;
			int numFrames = end - start;

			if(numFrames > 20) //should be atleast 20 I guess
			{
				float lastTime = full_motion->keytime(end);
				float firstTime = full_motion->keytime(start);
				float duration = lastTime-firstTime;

				for(int j=start;j<end;j++)	
				{
					float t = duration*((float)j-(float)start)/((float)numFrames);
					//I have to make a copy of the posture because some segments may share
					//the same posture but be mirrored
					KnPosture* p = new KnPosture(*full_motion->posture(j));
					segment->motion->insert_frame(segment->motion->frames(),t,p);
				}
				kn_segments.push(segment);
			}
			else
			{
				//phout<<"step "<<s<<" of piece is too short\n";
			}
	}
// 
// 	//first take the motion and break it to pieces in case there are discontinuities
// 	GsArray<KnMotion*> continuous_pieces;
// 	extractContinuousPieces(&continuous_pieces, full_motion);
// 	int stepCount = 0;
// 	for(int m=0;m<continuous_pieces.size();m++)
// 	{
// 		//next take each piece and extract the steps from it
// 		//note the way I do this there is inherent overlap in the steps that are saved
// 		//in other words many of the postures are saved twice
// 		KnMotion* continuous_piece = continuous_pieces[m];	
// 		GsArray<motion_fragment> steps;
// 		
// 		if(m==0)
// 		{
// 			steps.push();
// 			steps.top().start_frame=0;
// 			steps.top().end_frame = 120;
// 			steps.top().stancefoot = STANCE_LEFT;
// 		}
// //first extract the steps where the left is the stance foot
// 		//extractSteps(&steps,continuous_piece,STANCE_LEFT);
// 		//then the right
// 		//extractSteps(&steps,continuous_piece,STANCE_RIGHT);
// 		
// 
// 		for (int s = 0; s < steps.size(); s++ ) 
// 		{
// 			
// 			//now for each one of the steps bake it to its own KnMotion
// 			motion_fragment* fragment = &steps[s];
// 			kn_motion_segment* segment = new kn_motion_segment;
// 			segment->motion = new KnMotion ;
// 			segment->motion->connect(full_motion->skeleton());
// 			GsString name = "Motion_";
// 			name<<stepCount;
// 			stepCount++;
// 			segment->motion->name(name);
// 
// 			segment->color = GsColor::blue;
// 			segment->stanceFoot = fragment->stancefoot;
// 			int start = fragment->start_frame;
// 			int end = fragment->end_frame;
// 			int numFrames = end - start;
// 
// 			if(numFrames > 20) //should be atleast 20 I guess
// 			{
// 				float lastTime = continuous_piece->keytime(end);
// 				float firstTime = continuous_piece->keytime(start);
// 				float duration = lastTime-firstTime;
// 
// 				for(int j=start;j<end;j++)	
// 				{
// 					float t = duration*((float)j-(float)start)/((float)numFrames);
// 					//I have to make a copy of the posture because some segments may share
// 					//the same posture but be mirrored
// 					KnPosture* p = new KnPosture(*continuous_piece->posture(j));
// 					segment->motion->insert_frame(segment->motion->frames(),t,p);
// 				}
// 				kn_segments.push(segment);
// 			}
// 			else
// 			{
// 				//phout<<"step "<<s<<" of piece is too short\n";
// 			}
// 		}
// 	}
// 	phout<<"found "<<kn_segments.size()<<" segments\n";

}


void HumanMotionSegmenter::sortMotions(GsVec skeletonStartPoint)
{
	KnJoint* stanceFoot;
	KnJoint* swingFoot;
	kn_motion_segment* seg;
	KnMotion* motion;
	KnJoint* root;
	KnSkeleton* sk;
	GsVec stanceFootStart;
	GsVec swingFootStart;
	GsVec hipsStart;
	GsVec stanceFootEnd;
	GsVec stanceFootMiddle;
	GsVec swingFootEnd;
	GsVec hipsEnd;

	for(int i=0;i<kn_segments.size();i++)
	{
		seg = kn_segments.get(i);
		sk = seg->motion->skeleton();
		motion = seg->motion;
		motion->connect(sk);
		root = sk->root();
		stanceFoot = sk->joint("LeftToeBase");
		swingFoot = sk->joint("RightToeBase");
	
		motion->apply_frame(0);
		sk->update_global_matrices();
		
		//we want all motions to be stance left for simplicity.. or is it to make my life more complicated?
		if (seg->stanceFoot==STANCE_RIGHT)
		{
			motion->mirror("Left","Right");
			seg->stanceFoot = STANCE_LEFT;
		}

		//rotate and translate the motions so that they start at the origin with a +z heading
		motion->apply_frame(0);
		sk->update_global_matrices();
		
		orientMotionToOrigin(motion);
		translateMotionToOrigin(motion,skeletonStartPoint);

		//set the motion to the beginning to capture inital posture
		motion->apply_frame(0);
		sk->update_global_matrices();
		swingFootStart = swingFoot->gcenter();
		hipsStart = root->gcenter();
		
		//set the motion to the middle
		//this is my hacky way of choosing a stance foot location that is planted.. could cause problems
		motion->apply_frame((int)(0.5f*(motion->frames()-1)));
		sk->update_global_matrices();
		stanceFootMiddle = stanceFoot->gcenter(); 

		//set the motion to the end to get final posture
		motion->apply_frame(motion->frames()-1);
		sk->update_global_matrices();
		swingFootEnd = swingFoot->gcenter();
		hipsEnd = root->gcenter();

		//the motion should now be set to have the left the stance foot
		seg->stanceSwingStart = swingFootStart - stanceFootMiddle;
		seg->stanceSwingEnd =	swingFootEnd - stanceFootMiddle;
		seg->stanceComEnd		=  hipsEnd - stanceFootMiddle;
		seg->stanceComStart		=  hipsStart - stanceFootMiddle;

	}
	
	

}
#include "util_manipulator.h"

GsArray<Channel*> makeManipChannels(Manipulator* sav,bool x,bool y,bool z,bool rx,bool ry,bool rz)
{
	GsArray<Channel*> chs;

	if(x)
	{
		chs.push(new TrajectoryChannel(sav,manipulator_position,CH_VEC_X,TRAJ_LINEAR));
		chs.top()->setP(channel_phase_flip,true);
	}
	if(y)chs.push(new TrajectoryChannel(sav,manipulator_position,CH_VEC_Y,TRAJ_LINEAR));
	if(z)chs.push(new TrajectoryChannel(sav,manipulator_position,CH_VEC_Z,TRAJ_LINEAR));

	
	if(rx)chs.push(new TrajectoryChannel(sav,manipulator_orientation,CH_ROT_X,TRAJ_LINEAR));
		
	if(ry)chs.push(new TrajectoryChannel(sav,manipulator_orientation,CH_ROT_Y,TRAJ_LINEAR));
		//chs.top()->setP(channel_phase_flip,true);
	 if(rz)chs.push(new TrajectoryChannel(sav,manipulator_orientation,CH_ROT_Z,TRAJ_LINEAR));
		

	return chs;
}
GsArray<Channel*> makeComChannels(Serializable* segmenter,bool x,bool y,bool z)
{
	GsArray<Channel*> chs;
	if(x){
		chs.push(new TrajectoryChannel(segmenter,segment_motion_com_vector,CH_VEC_X,TRAJ_LINEAR));
		chs.top()->setP(channel_phase_flip,true);
	}
	if(y){
		chs.push(new TrajectoryChannel(segmenter,segment_motion_com_vector,CH_VEC_Y,TRAJ_LINEAR));
		chs.top()->range(0.1f,0,0.1f);
	}
	if(z)
		chs.push(new TrajectoryChannel(segmenter,segment_motion_com_vector,CH_VEC_Z,TRAJ_LINEAR));

	return chs;
}
#include "ph_mod_com.h"
#include "ph_mod_ik.h"
void HumanMotionSegmenter::transformMotion(PhysicalHuman* h, KnMotion* motion,HumanMotion* newMotion,GsArray<bool> vals)
{
	if(vals.size()!=33)
	{
		phout<<"need 33 bools for transform motion\n";
		return;
	}
	IKModule* ik = h->ik_module();
	COMModule* com_controller = h->com_module();
	newMotion->initChannels();
	newMotion->setMotionName(motion->name());
	newMotion->setP(human_motion_manual_com,false);

	Manipulator* leftManip = (Manipulator*)ik->getLeftFootManip();
	Manipulator* rightManip = (Manipulator*) ik->getRightFootManip();
	Manipulator* rootManip =   (Manipulator*)ik->getRootManip();
	Manipulator* rightHandManip = (Manipulator*)ik->getRightHandManip();
	Manipulator* leftHandManip =  (Manipulator*)ik->getLeftHandManip();
	GsArray<bool> vs;
	
	newMotion->pushChannels(makeManipChannels(leftManip,vals[0],vals[1],vals[2],vals[3],vals[4],vals[5]));
	newMotion->pushChannels(makeManipChannels(rightManip,vals[6],vals[7],vals[8],vals[9],vals[10],vals[11]));
	newMotion->pushChannels(makeManipChannels(rootManip,vals[12],vals[13],vals[14],vals[15],vals[16],vals[17]));
	newMotion->pushChannels(makeManipChannels(leftHandManip,vals[18],vals[19],vals[20],vals[21],vals[22],vals[23]));
	newMotion->pushChannels(makeManipChannels(rightHandManip,vals[24],vals[25],vals[26],vals[27],vals[28],vals[29]));

	//#ifdef MAKE_COM_CHANNELS
	newMotion->pushChannels(makeComChannels(this,vals[30],vals[31],vals[32]));
	//#endif

	
	int firstFrame = 0;
	int	lastFrame = motion->frames()-1;
	

	int numFrames = lastFrame-firstFrame;
	float lastTime = motion->keytime(lastFrame);
	float firstTime = motion->keytime(firstFrame);
	float duration = lastTime - firstTime;

	newMotion->duration(duration);
	GsVec stancePosStart; 
	GsVec comVec;
	
	

	for( int f=firstFrame;f<lastFrame;f++)
	{
		motion->apply_frame(f);
		motion->skeleton()->update_global_matrices();
		ik->matchToSkeleton(motion->skeleton());
		if(f==0)
			stancePosStart = 	motion->skeleton()->joint("LeftFoot")->gcenter();

		comVec = 	motion->skeleton()->root()->gcenter() - stancePosStart;
		//phout<<"set ref pt "<<comVec<<gsnl;
		setP(segment_motion_com_vector,comVec);


		for(int j=0;j<newMotion->numChannels();j++)
		{
			float t = ((float)f/numFrames)*duration;
			Channel* ch = newMotion->getChannel(j);
			if(ch->isTrajectory())
				((TrajectoryChannel*)ch)->setKey(t);
		}

	}

	for(int j=0;j<newMotion->numChannels();j++)
	{
		Channel* ch = newMotion->getChannel(j);
		if(ch->isTrajectory())
			((TrajectoryChannel*)ch)->copyFromTrajectory();
	
	}

	for (int j=0;j<newMotion->numChannels();j++)
	{
		Channel* ch = newMotion->getChannel(j);
		if(ch->getObject()==this && ch->getParameterID()==segment_motion_com_vector)
		{
			ch->setParameter((Serializable*)com_controller,com_desired_support_vector,ch->getDofType());

			ch->setP(channel_general_name,"empty");
			ch->getParameter(channel_general_name)->save = false;
			// phout<<"switching com parm "<<ch->toString()<<gsnl;
		}
	}
	//newMotion->generalizeNames(STANCE_LEFT);
	newMotion->updateChannelList();
}
void HumanMotionSegmenter::transformMotions(PhysicalHuman* h)
{

	for(int i=0;i<kn_segments.size();i++)
	{
		kn_motion_segment* seg = kn_segments.get(i);
		KnMotion* motion = seg->motion;

		HumanMotion* newMotion = new HumanMotion();
		newMotion->setP(human_motion_stance_swing_vec_start,seg->stanceSwingStart);
		newMotion->setP(human_motion_stance_swing_vec_end,seg->stanceSwingEnd);
		newMotion->setP(human_motion_stance_com_vec_start,seg->stanceComStart);
		newMotion->setP(human_motion_stance_com_vec_end,seg->stanceComStart);

		motion_segments.push(newMotion);
		GsArray<bool> vals;
		for(int i=0;i<33;i++)
		{
			vals.push(true);
		}

		transformMotion(h,motion,newMotion,vals);
	

	}
	
	kn_segments.get(0)->motion->apply_frame(0);
	
	phout<<"transformed "<<motion_segments.size()<<" into ik motions\n";

}


void HumanMotionSegmenter::reduceMotion(bool flat)
{
	GsArray<HumanMotion*> newMotions;
	for (int i=0;i<motion_segments.size();i++)
	{
		
		HumanMotion* m_original =  motion_segments[i]; 
		HumanMotion* m = new HumanMotion();
		m->setMotionName(m_original->getMotionName());
		m->reduceMotion(m_original ,pInt(segment_motion_reduction_sample_points),pFloat(segment_motion_reduction_slope_tolerance),pFloat(segment_reduction_merge_distance),pFloat(segment_motion_reduction_conc_tolerance),flat);
		m->generalizeNames(STANCE_LEFT);
		m->setP(human_motion_stance_swing_vec_start,m_original->pVec(human_motion_stance_swing_vec_start));
		m->setP(human_motion_stance_swing_vec_end,m_original->pVec(human_motion_stance_swing_vec_end));
		m->setP(human_motion_stance_com_vec_start,m_original->pVec(human_motion_stance_com_vec_start));
		m->setP(human_motion_stance_com_vec_end,m_original->pVec(human_motion_stance_com_vec_end));
		m_original->generalizeNames(STANCE_LEFT);
		m_original->setStance(STANCE_LEFT);
		m->setStance(STANCE_LEFT);
		simple_segments.push(m);
		
	}
	phout<<"reduced "<<simple_segments.size()<<"segments\n";
}

void HumanMotionSegmenter::extractContinuousPieces( GsArray<KnMotion*>* continuous_pieces, KnMotion* full_motion )
{
	KnSkeleton* sk = full_motion->skeleton();
	KnJoint *hips = sk->root();
	GsVec hipsStart;
	GsVec hipsCurrent;
	GsVec hipsLast;
	continuous_pieces->push() = new KnMotion;
	continuous_pieces->top()->connect(sk);

	full_motion->apply_frame(0);
	sk->update_global_matrices();
	hipsLast =hips->gcenter();

	for (unsigned int i = 1; i < full_motion->frames(); i++ ) 
	{
		/* apply the frame and update_global_matrices */
		full_motion->apply_frame(i);
		sk->update_global_matrices();

		if(dist(hips->gcenter(),hipsLast)>0.5f)
		{
			//phout<<"found jump at frame "<<i<<gsnl;
			continuous_pieces->push() = new KnMotion;
			continuous_pieces->top()->connect(sk);
		}
		else
		{
			continuous_pieces->top()->insert_frame(continuous_pieces->top()->frames(),full_motion->keytime(i),full_motion->posture(i));
		}
		hipsLast = hips->gcenter();
	}
}

void HumanMotionSegmenter::extractSteps(GsArray<motion_fragment> *steps, KnMotion* motion,human_stance stance)
{
	KnSkeleton* sk = motion->skeleton();
	KnJoint *swing_toe;
	KnJoint* swing_hip;
	KnJoint *stance_toe;
	KnJoint* stance_hip;

	if(stance ==STANCE_LEFT)
	{
		swing_toe = sk->joint("RightToeBase");
		swing_hip = sk->joint("RightUpLeg");
		stance_toe = sk->joint("LeftToeBase");
		stance_hip = sk->joint("LeftUpLeg");
	}
	else
	{
		swing_toe = sk->joint("LeftToeBase");
		swing_hip = sk->joint("LeftUpLeg");
		stance_toe = sk->joint("RightToeBase");
		stance_hip = sk->joint("RightUpLeg");
	}

	KnJoint* swing_ankle = swing_toe->parent();
	KnJoint* stance_ankle = stance_toe->parent();

	float epsilon = 0.002f; //the error margin that signals a motion is moving. 
	float cutoff = 0.01f;	//if the change is greater than this it has started moving
	int frameCounter = 0;
	//some temp variables to keep track of when segmenting each clip

	GsVec swingFootStart;
	float swingToeStartHeight;
	float swingToeCurrentHeight;
	GsVec swingFootCurrent;
	int swingFootStartIndx =0; 

	for ( int i = 0; i < (int)motion->frames(); i++ ) 
	{
		motion->apply_frame(i);
		sk->update_global_matrices();

		frameCounter++;

		swingFootCurrent = swing_ankle->gcenter();
		swingToeCurrentHeight = swing_toe->gcenter().y;

		if(i==0)
		{
			swingFootStart = swingFootCurrent;	
			swingToeStartHeight = swingToeCurrentHeight;
			swingFootStartIndx =0;
		}

		if(i>swingFootStartIndx)
		{
			//check if the heel has started moving which means the toe is lifting
			float swingFootDistance = dist(swingFootCurrent,swingFootStart);
			if(swingFootDistance>cutoff)
			{
			
				GsVec stanceFootStart = stance_toe->gcenter();
				bool stanceFootMoving = false;
				steps->push().stancefoot = stance;
				steps->top().start_frame = swingFootStartIndx;
				bool lookingForEnd = true;
				unsigned idx = i;  //to keep track of the frame relative to the beginning
				int numFrames = 0;
	
				float swingHeight;
				while(lookingForEnd)
				{
					idx++;
					numFrames++;
					if(idx>=motion->frames())
					{
						lookingForEnd = false;

#ifdef TRASH_LAST_PIECE
						steps->pop(); //if we dont want to keep the end piece
#else
						steps->top().end_frame = idx-1;
#endif
						swingFootStartIndx = idx;
						continue;
					}

					//update the motion
					motion->apply_frame(idx);
					sk->update_global_matrices();

					if(numFrames>20)
					{
						if(stanceFootMoving)
						{
							
							if(abs(swingHeight-swing_ankle->gcenter().y)<0.05f)
							{
								steps->top().end_frame = idx;
								lookingForEnd = false;
								swingFootStart = swing_ankle->gcenter();
								swingFootStartIndx = idx;
							}
						}
						else
						{
							if(dist(stanceFootStart,stance_toe->gcenter())>epsilon)
							{
								stanceFootMoving = true;
								swingHeight = swing_ankle->gcenter().y;
							}
						}
						

					}
				}
			}
		}

	


	}

	
	
}


