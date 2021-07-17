#include "ph_motion_manager.h"
#include "ph_mod_ik.h"
#include "ph_motion.h"
#include "util_trajectory.h"
#include "util_channel_traj.h"
#include "util_channel.h"
#include "ph_manager.h"
#include "ph_manip.h"
#include "ph_mod_ik.h"
#include "ph_manip_ik.h"
#include "util_curve.h"
#include "ph_file_manager.h"
void HumanMotionManager::mergeControlPoints( float tolerance )
{

	for (int i=0;i<scvs.size();i++)
	{
		if(scvs.get(i)->isTrajectory())
		{
			TrajectoryChannel* tch = (TrajectoryChannel*)scvs.get(i);

			tch->mergeControlPoints(tolerance);
		}
	}
}

void HumanMotionManager::fitCurve( int points ,float tolerance,float mergeDist,float concavity_tolerance)
{
	// 	if(!currentMotion())
	// 		return;
	// 
	// 	GsArray<Channel*> newChns;
	// 	for (int i=0;i<scvs.size();i++)
	// 	{
	// 		Channel* originalCurve = scvs[i];
	// 		if(scvs.get(i)->getChannelMode() == channel_trajectory)
	// 		{
	// 			TrajectoryChannel* tch = (TrajectoryChannel*)scvs.get(i);
	// 
	// 			Channel* newCurve = new Channel(originalCurve);
	// 		
	// 
	// 			GsString oldName = originalCurve->name();
	// 			oldName<<"_old";
	// 			originalCurve->setName(oldName);
	// 			originalCurve->deactivate();
	// 
	// 			newCurve->fitCurve(originalCurve,points,tolerance,mergeDist,concavity_tolerance);
	// 			currentMotion()->pushChannel(newCurve);
	// 			newChns.push(newCurve);
	// 		}
	// 	}
	// 	scvs.push(newChns);
	// 	connectMotion(currentMotion());
	// 	updateMotionTrajectories();
	phout<<"fix fit curve\n";
}

void HumanMotionManager::deleteChannels()
{
	if(!currentMotion())
		return;

	for(int i=0;i<scvs.size();i++)
	{
		for(int j=0;j<currentMotion()->numChannels();j++)
		{
			if(scvs[i] == currentMotion()->getChannel(j))
			{
				currentMotion()->removeChannel(j);
			}
		}
	}

	scvs.size(0);
	updateCurves();
}
void HumanMotionManager::deleteSelectedCurves()
{
	if(!currentMotion())
		return;
	for(int it=0;it<currentMotion()->numChannels();it++)
	{
		for(int i=0;i<scvs.size();i++)
		{
			if(scvs[i] == currentMotion()->getChannel(it))
			{
				currentMotion()->removeChannel(it);
			}
		}
	}
	for(int i=0;i<scvs.size();i++)
	{
		delete scvs[i];
	}
	scvs.size(0);
}
void HumanMotionManager::setKey()
{
	for (int i=0;i<scvs.size();i++)
	{
		if(scvs.get(i)->isTrajectory())
		{
			TrajectoryChannel* tch = (TrajectoryChannel*)scvs.get(i);
			tch->setKey(getTime());
			tch->copyFromTrajectory();
		}
	}
}

TrajectoryChannel* HumanMotionManager::push( GsVec2 mouseP, curve_state s )
{

	for(int j=0;j<scvs.size();j++)
	{ 
		if(scvs.get(j)->isTrajectory())
		{
			TrajectoryChannel* tch = (TrajectoryChannel*)scvs.get(j);

			Trajectory* cv = tch->getCurve();
			if(cv==0)continue;
			cv->selectionState = s;
			if(cv->press( mouseP ))
			{
				return tch;
			}
		}



	}
	return 0;
}
TrajectoryChannel* HumanMotionManager::drag( GsVec2 mouseP )
{
	for (int i=0;i<scvs.size();i++)
	{
		if(scvs.get(i)->isTrajectory())
		{
			TrajectoryChannel* tch = (TrajectoryChannel*)scvs.get(i);
			if (tch->getCurve()->drag(mouseP))
			{
				return tch;
			}
		}

	}
	return 0;
}


void HumanMotionManager::updateMotionTrajectories()
{
	if(!currentMotion())
	{
		if(trajGrp)
			trajGrp->visible(false);
		return;
	}
	IKModule* ik = human->ik_module();
	GsQuat ik_frame = ik->getFrameManip()->globalOrientation();
	GsQuat human_frame = human->getHeading();




	if(stanceHandCurve)
	{
		stanceHandCurve->setFromMotion(currentMotion());
		stanceHandCurve->setOrigin(ik->getStanceHandManip()->getStartPosition(),ik_frame);
		stanceHandCurve->update();  
	}
	if(swingHandCurve)
	{
		swingHandCurve->setFromMotion(currentMotion());
		swingHandCurve->setOrigin(ik->getSwingHandManip()->getStartPosition(),ik_frame);
		swingHandCurve->update(); 
	}
	if(rootCurve)
	{
		rootCurve->init();
		rootCurve->setFromMotion(currentMotion());
		rootCurve->setOrigin(ik->getRootManip()->getStartPosition(),ik_frame);
		rootCurve->update();
	}
	if(leftFootCurve)
	{
		leftFootCurve->init();
		leftFootCurve->setFromMotion(currentMotion());
		leftFootCurve->update();   
	}
	if(rightFootCurve)
	{	
		rightFootCurve->init();
		rightFootCurve->setFromMotion(currentMotion());
		rightFootCurve->update(); 
	}
	if(comCurve)
	{
		comCurve->setFromMotion(currentMotion());
		comCurve->setOrigin(human->getStancePosition(),GsQuat());
		comCurve->update(); 
	}

}

void HumanMotionManager::drawMotion()
{

	IKModule* ik = human->ik_module();
	GsQuat ik_frame = ik->getFrameManip()->globalOrientation();
	GsQuat human_frame = human->getHeading();
	
	if(!trajGrp)
	{
		trajGrp = new SnGroup;
		manager->getRoot()->add(trajGrp);
	}
	
	if(!stanceHandCurve) {
		stanceHandCurve = new VecTrajectory("LeftHand",manipulator_position,ik->getLeftHandManip()->getStartPosition(),ik_frame);
		trajGrp->add(stanceHandCurve->getCurve()->_line);
	}
	if(!swingHandCurve){
		swingHandCurve = new VecTrajectory("RightHand",manipulator_position,ik->getRightHandManip()->getStartPosition(),ik_frame);
		trajGrp->add(swingHandCurve->getCurve()->_line);
	}
	if(!rootCurve)	   {
		rootCurve = new VecTrajectory("Hips",manipulator_position,ik->getRootManip()->getStartPosition(),ik_frame);
		trajGrp->add(rootCurve->getCurve()->_line);
	}
	if(!leftFootCurve) {
		leftFootCurve = new VecTrajectory("LeftFoot",manipulator_position,ik->getLeftFootManip()->getStartPosition(),ik_frame);
		trajGrp->add(leftFootCurve->getCurve()->_line);
	}
	if(!rightFootCurve){
		rightFootCurve = new VecTrajectory("RightFoot",manipulator_position,ik->getRightFootManip()->getStartPosition(),ik_frame);
		trajGrp->add(rightFootCurve->getCurve()->_line);
	}
	if(!comCurve){
		/*
		we keep the com trajectory and the root trajectory. the root is still useful for the ik but the
		com should really just be the support vector... but maybe we can simplify 
		*/
		comCurve = new VecTrajectory("COMModule",com_desired_support_vector,human->getStancePosition(),human_frame);
		comCurve->getCurve()->cCol = GsColor::green;
		trajGrp->add(comCurve->getCurve()->_line);
	}

	if(currentMotion()->isHumanMotion())
	{
		_swingStanceOffset = currentMotion()->pVec(human_motion_stance_swing_vec_start);
	}
	updateMotionTrajectories();
	if(trajGrp)
	{
		trajGrp->visible(true);
	}
}


#include "ph_motion_segmenter.h"
#include "ph_mod_ref.h"

void HumanMotionManager::makeComposite(const GsString& contName,const GsString& mname,const GsArray<bool>& vals,bool allFrames,int firstFrame,int lastFrame)
{
	
	if(currentKnMotion())
	{
		GsString contDir = manager->getFiles()->getMotionDirectory(human->characterName());
		contDir<<contName<<SLASH;
		KnMotion* knm = currentKnMotion();
		HumanMotion* newm = openMotion(mname);

		if(newm==0)
		{
			manager->message("control motion was invalid:",mname);
			delete newm;
			return;
		}
		if(!allFrames)
		{
			for( int i=0;i<firstFrame;i++)
			{
				knm->remove_frame(0);
			}
			int numFrames = lastFrame-firstFrame;
			while((int)knm->frames()>numFrames)
			{
				knm->remove_frame(knm->frames()-1);
			}
		}
		translateMotionToOrigin(knm,human->reference_module()->pVec(reference_skeleton_position));
		orientMotionToOrigin(knm);

		manager->getMotionSegmenter()->transformMotion(human,knm,newm,vals);


	if(!manager->getFiles()->directoryExists(contDir))
	{
		manager->getFiles()->makeDirectory(contDir);
		phout<<"making directory "<<contDir<<gsnl;
	}
		newm->setMotionName(mname);
		GsString fileName = contDir;
		fileName<<contName<<".motion";
		GsString baseName = contDir;
		baseName<<contName<<".base";
		
		phout<<"saving motion "<<fileName<<gsnl;

		manager->getFiles()->saveMotionTrajectories(fileName,newm);
		manager->getFiles()->saveBaseMotion(baseName,newm);
	}
	else
		manager->message("must select kn motion");
		
}
void HumanMotionManager::moveChannelUp()
{
	if(!currentMotion())
		return;
	GsArray<Channel*>* chs =  getSelectedChannels();
	for(int i=0;i<chs->size();i++)
	{
		for(int j=1;j<currentMotion()->numChannels();j++)
		{
			if(currentMotion()->getChannel(j)==chs->get(i))
			{
				currentMotion()->swapChannel(j,j-1);
				break;
			}
		}
	}
	currentMotion()->updateChannelList();
}

void HumanMotionManager::moveChannelDown()
{
	if(!currentMotion())
		return;
	GsArray<Channel*>* chs =  getSelectedChannels();
	for(int i=chs->size()-1;i>=0;i--)
	{
		for(int j=currentMotion()->numChannels()-2;j>=0;j--)
		{
			if(currentMotion()->getChannel(j)==chs->get(i))
			{
				currentMotion()->swapChannel(j+1,j);
				break;
			}
		}
	}
	currentMotion()->updateChannelList();
}

void HumanMotionManager::scale_skeleton( KnSkeleton* sk,float sc )
{
	sk->root()->pos()->value(sk->root()->pos()->value()*sc);
	for (int i=0 ; i < sk->joints().size(); i++)
	{
		sk->joints()[i]->offset(sk->joints()[i]->offset()*sc);		
	}
}

void HumanMotionManager::scaleMotion( float v )
{
	if(currentMotion())
	{
		currentMotion()->setP(motion_duration,currentMotion()->duration()*v);
		for(int i=0;i<currentMotion()->numChannels();i++)
		{
			if(currentMotion()->getChannel(i)->isTrajectory())
			{
				TrajectoryChannel* trajC = (TrajectoryChannel*)currentMotion()->getChannel(i);
				trajC->getCurve()->scale(v);

			}
		}

	}
	else
	{
		manager->message("must be editing motion to scale it");
	}
	updateCurves();
}