#pragma once

#include <ole2.h>
#include <Windows.h>
#include <NuiApi.h>
#include "common.h"

#define MAX_NUM_KINECTS 1

#define USE_PLAYER_INDEX

#define DEPTH_RESOLUTION NUI_IMAGE_RESOLUTION_320x240
#define VIDEO_RESOLUTION NUI_IMAGE_RESOLUTION_640x480

#define DEPTH_SIZE_X 320
#define DEPTH_SIZE_Y 240
#define NUM_POINTS (DEPTH_SIZE_X*DEPTH_SIZE_Y)

#define VIDEO_SIZE_X 640
#define VIDEO_SIZE_Y 480

typedef struct point3D
{
	GsVec pos;
	GsColor col;
	bool active;
}_point3D;


inline float limitFloat(float f,float min,float max)
{
	if(f<min)f=min;
	if(f>max)f=max;
	return f;
}
inline GsVec limitVec(GsVec v,float min,float max)
{
	return GsVec(limitFloat(v.x,min,max),limitFloat(v.y,min,max),limitFloat(v.z,min,max));
}
inline int element(int x,int y)
{
	return y*DEPTH_SIZE_X + x;
}


class Kinect
{
	INuiSensor*         kinect;        
	HANDLE              videoEvent;     
	HANDLE              depthEvent;  
	HANDLE				depthStreamHandle;
	HANDLE				videoStreamHandle;
	HRESULT Nui_Init(int id = 0);
	void                    Nui_UnInit( );
	void                    Nui_GotDepthAlert();
	void                    Nui_GotVideoAlert();
	void                    Nui_Zero();
	
	static LONG CALLBACK    WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	static DWORD WINAPI     Nui_ProcessThread(LPVOID pParam);

	// thread handling
	HANDLE        m_hThNuiProcess;
	HANDLE        m_hEvNuiProcessStop;
	int           m_LastFPStime;
	int           m_FramesTotal;
	int           m_LastFramesTotal;
	bool locked;

	point3D points[NUM_POINTS];
	
	bool _new_data;
public:
	bool use_depth;
	bool use_color;
	Kinect();
	void lock(){locked = true;}
	void unlock(){locked=false;}

	point3D* getPoints(){return &points[0];}
	void setAngle(int a);
	int numKinects();
	void setPointPos(int x,int y,float px,float py,float pz);
	void setPointColor(int x,int y,GsColor c);
	GsVec getPoint(int x,int y);
	GsVec getPoint( int i);
	int getFPS(){return  m_LastFPStime;}

	void close();
	void init();
	bool hasNewData();
};