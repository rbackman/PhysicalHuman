#pragma once

#include "common.h"
#include "util_serializable.h"
#include "util_channel.h"

enum motion_parms
{
	motion_name,
	motion_start_state,
	motion_trajectory_list,
	motion_node_list,
	motion_duration,
	motion_loops,		//if true than the actual duration is 2xmotion_duration
	motion_time,
	motion_descriptor_avg,
	motion_descriptor_min,
	motion_descriptor_max,
	motion_descriptor_env,
    motion_success,
	motion_env_expand_len,
	motion_last
};


class Motion : public Serializable
{
private:
	
	Motion* _control_motion;
	GsArray<Channel*> _channels;

	bool _needsRefresh;
	bool _active;
	protected:
		Channel* _timewarp;
		bool human_motion;
		bool flip;

		void	refreshChannels()
		{
			_needsRefresh = true;
		}
		float _playTime;
		
	public:
		GsArray<Channel*> getChannels(){return _channels;}
		float playTime() {return _playTime;}
		bool isHumanMotion(){return human_motion;}
		bool needRefresh()
		{
			if(_needsRefresh)
			{
				_needsRefresh = false;
				return true;
			}
			return false;
		}
	
	public:
		bool loadDataFromFile(GsString motion_files);
		bool active(){return _active;}
		//Motion(GsString file,float dt);
		Motion();
		Motion(Motion* m);
		~Motion();
		
		Channel* getChannel(int i){if(i<_channels.size())return _channels[i];else return 0;}
		Channel* getChannel(GsString name);
		Channel* getChannel(Channel* otherCh);;

		void setMotionName(GsString n);
		GsString getMotionName();
		void pushChannel(Channel* ch);
		void insertChannel( Channel* ch, int idx );


		void pushChannels(GsArray<Channel*> chs);

		int numChannels(){return _channels.size();}
		float duration();

		float phase()
		{
			return pFloat(motion_time);
		}
		void duration(float dur)
		{
			setP(motion_duration,dur);
			applyParameters();
		}

			GsString baseString();
		GsString toString(bool printAll =false);
		void reduceMotion( Motion* m_original  ,int numSamples,float slope_tolerance,float merge_distance,float concavity_tolerance,bool flat);
		void setControlMotion(Motion* m,bool overrideConnections = false);
		void setTimeWarp(Channel* ch);

		bool load( GsString file );
		void applyParameters();

		bool loops();
		void loops(bool lp);
		/*set time takes as input 0-duration*/
		void setTime( float t );
		/*set phase takes from zero to one*/
		void setPhase( float p )
		{
			setTime(p*duration());
		}

		void update();
		void removeChannel( int it );
		void activate();
		void initChannels();
		void updateChannelList();
		void swapChannel( int prevI ,int newI);
		void scale( float v );
		GsString trajectoryString();
		GsString boundsString();
		bool haseTrajectories();
		bool hasNodes();
		
};