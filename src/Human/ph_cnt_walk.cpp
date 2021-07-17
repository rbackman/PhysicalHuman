#include "ph_cnt_walk.h"
#include "ph_mod_ik.h"
#include "ph_mod_contact.h"
#include "ph_mod_ref.h"
#include "util_channel.h"
#include "common.h"
#include "ph_mod_com.h"
#include "ph_cnt_virtual.h"
#include "util_curve.h"

WalkController::WalkController(PhysicalHuman* human,const char* file):Module(human,"WalkController",file)
{
	ik=0;

	stanceFoot = human->getStanceState();
	graph = 0; 
	stepPhase = 0;
}


WalkController::~WalkController(void)
{
	
}

static void virtualManipCallback ( SnManipulator* mnp,const GsEvent& ev, void* udata )
{
	WalkController* wc = (WalkController*)udata;

	GsVec p1 = wc->h->getCOMProjection();
	GsVec p2 = wc->h->stanceFoot()->getCOMPosition(); 
	GsVec p3 = wc->stepManip->translation();
	GsVec stepTop = wc->h->swingFoot()->getCOMPosition()+GsVec(0,0.2f,0);


	wc->com_line->clear();
	wc->com_line->addPoint(p1);
	wc->com_line->addPoint(p2);
	wc->com_line->addPoint(p3);
	wc->com_line->addPoint(p3);
	wc->com_line->update();


	wc->swing_line->clear();
	wc->swing_line->addPoint(wc->h->swingFoot()->getCOMPosition());
	wc->swing_line->addPoint(stepTop);
	wc->swing_line->addPoint(p3);
	wc->swing_line->addPoint(p3);
	wc->swing_line->update();


}

void WalkController::stepTo(GsVec p3)
{
	GsVec p1 = h->getCOMProjection();
	GsVec p2 = h->stanceFoot()->getCOMPosition(); 
	stepManip->translation(p3);
	GsVec stepTop = h->swingFoot()->getCOMPosition()+GsVec(0,0.2f,0);

	swing_line->clear();
	swing_line->addPoint(h->swingFoot()->getCOMPosition());
	swing_line->addPoint(stepTop);
	swing_line->addPoint(p3);
	swing_line->addPoint(p3);
	swing_line->update();

	com_line->clear();
	com_line->addPoint(p1);
	com_line->addPoint(p2);
	com_line->addPoint(p3);
	com_line->addPoint(p3);
	com_line->update();
}

void WalkController::init()
{
	ik = h->ik_controller();
	
	stepManip = new SnManipulator;
	
	stepManip->child(new SnModel(h->swingFoot()->box->model->getModel()));

	
	stepManip->callback(virtualManipCallback,this);
	stepManip->translation(h->swingFoot()->getCOMPosition()+GsVec(0,0,0.5f));
	

	grp->add(stepManip);

	com_line  = new Curve;
	com_line->cCol.set(255,0,0);
	grp->add(com_line->_line);

	swing_line = new Curve;
	grp->add(swing_line->_line);

	stepTo(stepManip->translation());


}
void WalkController::activate()
{
	//if(graph)
	//	graph->loadMotion("step_basic");
	Module::activate();
	stepTo(h->swingFoot()->getCOMPosition());

	//stepPhase = 0;
	//graph->setStance(channels,h,STANCE_LEFT);
	//setState(WALK_START);
}
bool WalkController::evaluate()
{
	//bool valid = Controller::evaluate();
	
	

	if(isActive() && pFloat(controller_phase)<=1.0f )
	{
		setP(controller_phase,pFloat(controller_phase)+0.0001f);
		h->com_controller()->setDesiredCOM( com_line->eval_bezier(getPhase()) );
		//h->virtual_controller()->getSpring(h->swingFoot()->name())->manip->translation(swing_line->eval_bezier(getPhase()));
		
		if(graph==0)
		{
			gsout<<"need to set graph pointer for walk controller\n";
			//graph->setPhase(controller_phase);
			return true;
		}
		
		/*
		switch(getState())
		{
			case WALK_START:
				//do some initialization
				//h->stanceState = STANCE_LEFT;
				if(channels.size() ==0)
				{
					channels.push( App->files->loadMotionNamed(h,"step_basic") );

				}
				setState(WALK_START_WEIGHT_STANCE);
				h->com_controller()->setP(com_velocity_desired_coronal,walkSpeed);
				getChannel("SwingIKFoot_PY")->movePt(1,stepHeight);

			break;
			case WALK_START_WEIGHT_STANCE:
			//set stance to swing ratio
				h->contact_controller()->setStanceSwingRatio(1.0f);
				setState(WALK_WEIGHT_STANCE);
			break;

			case WALK_WEIGHT_STANCE:
			//wait for the stance foot to contain the com
				{	float off = h->contact_controller()->stanceComOffset().len();
				//	gsout<<off<<gsnl;
				if(off<stanceOff) //h->contact_controller()->stanceFootContains(h->getCOM()))
					setState(WALK_START_STEP);
			
				}
				break;
			case WALK_START_STEP:
				//make trajectories for the swing foot considering the IP model
				stepPhase = 0;
				swingLifted = false;
				h->freeSwingLeg();
				h->swingFoot()->setP(joint_virtual_force,false);
				h->swingHip()->setP(joint_virtual_force,false);

				setState(WALK_STEP);
				
				//graph->moveSwingFootTo(App->human->contact_controller()->getIPStepPos());


			break;
			case WALK_STEP:
				//keep moving swing foot till it is planted again
				
				stepPhase += h->world->getSimStep();
				if(stepPhase<0.5f)
				{
					for(int i=0;i<channels.size();i++)
					{
						channels.get(i)->setPhase(stepPhase);
					}
					ik->evaluate();
				}
				else
				{ //if it is at the end of the phase 
					if(h->contact_controller()->swingFootPlanted())
						setState(WALK_STEP_LANDED);

				}

				if(!swingLifted)
				{
					if(!h->contact_controller()->swingFootPlanted())
						swingLifted = true;
				}
				else
				{
					if(h->contact_controller()->swingFootPlanted())
					setState(WALK_STEP_LANDED);
				}

			break;
			case WALK_STEP_LANDED:
			//switch the stance and swing foot
				h->lockSwingLeg();

				setState(WALK_START_WEIGHT_STANCE);
				stepPhase = 0;
				
				stanceFoot = !stanceFoot;

				graph->setStance(channels,App->human,stanceFoot);

			break;
			case WALK_STOP:
				h->contact_controller()->setStanceSwingRatio(0.5f);
				break;
			case WALK_STOPPING:
				//if the stanceSwing goal position is close enough to the com position you are done
			break;
		}


		if(stepPhase >= 0.5 )
		{
			
			
		}
		
		graph->setPhase(stepPhase);
	}
	*/

	}
	return true;
}


