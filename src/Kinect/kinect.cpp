#include "kinect.h"

#include <mmsystem.h>

#include "util.h"

void Kinect::Nui_Zero()
{

	m_hThNuiProcess=NULL;
	m_hEvNuiProcessStop=NULL;


	m_FramesTotal = 0;
	m_LastFPStime = -1;
	m_LastFramesTotal = 0;
}


int Kinect::numKinects()
{
	int numKinects;
	NuiGetSensorCount(&numKinects);
	return numKinects;
}
HRESULT Kinect::Nui_Init(int id)
{
	locked = false;

	HRESULT                hr;
	int numKinects;
	hr = NuiGetSensorCount(&numKinects);
	if (FAILED(hr) ) { return hr; }

	if(numKinects>MAX_NUM_KINECTS)
		numKinects = MAX_NUM_KINECTS;


	depthEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	videoEvent =  CreateEvent( NULL, TRUE, FALSE, NULL );

	hr = NuiCreateSensorByIndex(id,&kinect );

#ifdef USE_PLAYER_INDEX
	hr = 	kinect->NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |   NUI_INITIALIZE_FLAG_USES_COLOR );

#else
	hr = 	kinect->NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH |   NUI_INITIALIZE_FLAG_USES_COLOR );
#endif

	if( FAILED( hr ) )
	{
		phout<<"error initializing kinect "<<id<<" error code "<< hr<<gsnl;
		return hr;
	}

	hr = kinect->NuiImageStreamOpen 
		(
		NUI_IMAGE_TYPE_COLOR,
		VIDEO_RESOLUTION,
		0,
		2,
		videoEvent,
		&videoStreamHandle
		);
	if( FAILED( hr ) )
	{
		phout<<"error creating the video stream\n";
		return hr;
	}


	hr = kinect->NuiImageStreamOpen
		(
#ifdef USE_PLAYER_INDEX
		NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
#else
		NUI_IMAGE_TYPE_DEPTH,
#endif
		DEPTH_RESOLUTION,
		0,
		2,
		depthEvent,
		&depthStreamHandle );

	if( FAILED( hr ) )
	{
		phout<<"error loading depth stream\n";
		return hr;
	}

	kinect->NuiImageStreamSetImageFrameFlags(depthStreamHandle,  NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);



	if(numKinects>0)
	{
		// Start the Nui processing thread
		m_hEvNuiProcessStop=CreateEvent(NULL,FALSE,FALSE,NULL);
		m_hThNuiProcess=CreateThread(NULL,0,Nui_ProcessThread,this,0,NULL);
	}


	return hr;
}

void Kinect::setAngle( int a)
{
	phout<<"set angle "<<a<<gsnl;
	kinect->NuiCameraElevationSetAngle(a);

}


void Kinect::Nui_UnInit( )
{


	// Stop the Nui processing thread
	if(m_hEvNuiProcessStop!=NULL)
	{
		// Signal the thread
		SetEvent(m_hEvNuiProcessStop);

		// Wait for thread to stop
		if(m_hThNuiProcess!=NULL)
		{
			WaitForSingleObject(m_hThNuiProcess,INFINITE);
			CloseHandle(m_hThNuiProcess);
		}
		CloseHandle(m_hEvNuiProcessStop);
	}






	if( depthEvent  && ( depthEvent != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle( depthEvent );
		depthEvent = NULL;
	}
	if( videoEvent && ( videoEvent != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle(videoEvent );
		videoEvent = NULL;
	}

	kinect->NuiShutdown();
}



DWORD WINAPI Kinect::Nui_ProcessThread(LPVOID pParam)
{
	Kinect *pthis= (Kinect *) pParam;
	HANDLE                hEvents[3];
	int                    nEventIdx,t,dt;

	// Configure events to be listened on
	hEvents[0]=pthis->m_hEvNuiProcessStop;
	hEvents[1]=pthis->depthEvent;
	hEvents[2]=pthis->videoEvent;


	// Main thread loop
	while(1)
	{
		// Wait for an event to be signaled
		nEventIdx=WaitForMultipleObjects(3,hEvents,FALSE,100);

		// If the stop event, stop looping and exit
		if(nEventIdx==0)
			break;            

		// Perform FPS processing
		t = 0;//timeGetTime( );
		if( pthis->m_LastFPStime == -1 )
		{
			pthis->m_LastFPStime = t;
			pthis->m_LastFramesTotal = pthis->m_FramesTotal;
		}
		dt = t - pthis->m_LastFPStime;
		if( dt > 1000 )
		{
			pthis->m_LastFPStime = t;
			int FrameDelta = pthis->m_FramesTotal - pthis->m_LastFramesTotal;
			pthis->m_LastFramesTotal = pthis->m_FramesTotal;

		}



		// Process signal events
		switch(nEventIdx)
		{
		case 1:
			pthis->Nui_GotDepthAlert();
			pthis->m_FramesTotal++;
			break;

		case 2:
			pthis->Nui_GotVideoAlert();
			break;
		}
	}

	return (0);
}

void Kinect::Nui_GotVideoAlert()
{
	//	phout<<"got video alert from cam "<<i<<gsnl;
	if(locked)
	{
		//phout<<"locked video alert\n";
		return;
	}
	const NUI_IMAGE_FRAME * pImageFrame = NULL;

	HRESULT hr = NuiImageStreamGetNextFrame
		(
		videoStreamHandle,
		0,
		&pImageFrame 
		);
	if( FAILED( hr ) )
	{
		phout<<"image stream failed error "<<hr<<gsnl;
		return;
	}

	INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 )
	{
		//phout<<"video pitch "<<LockedRect.Pitch<<gsnl;
		BYTE * pBuffer = (BYTE*) LockedRect.pBits;

		long colorx,colory;
		long colIndx = 0;
		for( int y = 0 ; y < DEPTH_SIZE_Y-1 ; y++ )
		{
			for( int x = 0 ; x < DEPTH_SIZE_X-1 ; x++ )
			{
				if(use_color)
				{
					if(use_depth)
					{

						USHORT depth = (USHORT)getPoint(x,y).z; // <<3;
						//find the color value in the buffer that corresponds to this point in the point cloud
						NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480,0,x,y,depth,&colorx,&colory);

						//make sure the color coordinates are within the view area
						if(colorx >= VIDEO_SIZE_X)colorx = VIDEO_SIZE_X-1;
						if(colory >= VIDEO_SIZE_Y)colory = VIDEO_SIZE_Y-1;
						if(colory < 0) colory = 0;
						if(colorx < 0) colorx = 0;


#ifdef USE_PLAYER_INDEX

#else
						colorx = VIDEO_SIZE_X - colorx + 16; //not sure why 16 but it seems to work
#endif

						colIndx = (colorx + colory*VIDEO_SIZE_X)*4;

						if(colIndx > VIDEO_SIZE_X * VIDEO_SIZE_Y * 4)
						{
							colIndx =  VIDEO_SIZE_X * VIDEO_SIZE_Y*4 -1;
							phout<<"colIndx out of bounds"<<colIndx<< " ont index x:" << x <<" y: "<<y<<gsnl;
						}
					}
					else
						colIndx = (x*2 + y*2*VIDEO_SIZE_X)*4;
					//colors are stored as b g r
					setPointColor(x ,y,	GsColor(pBuffer[colIndx+2],pBuffer[colIndx+1],pBuffer[colIndx]) );   	
				}
				else 
				{
					float depth = getPoint(x,y).z/3.0f;
					setPointColor( DEPTH_SIZE_X- x ,y,	GsColor(0.0f,1.0f-depth,depth));

				}
			} 		
		}
	}
	else
	{
		phout<< "Buffer length of received texture is bogus\r\n" ;
	}

	NuiImageStreamReleaseFrame( videoStreamHandle, pImageFrame );
}

void Kinect::Nui_GotDepthAlert()
{
	if(locked)
	{
		//phout<<"locked depth alert\n";
		return;
	}
	_new_data = true;

	const NUI_IMAGE_FRAME * pImageFrame = NULL;

	HRESULT hr = NuiImageStreamGetNextFrame(
		depthStreamHandle,
		0,
		&pImageFrame );

	if( FAILED( hr ) )
	{
		return;
	}

	INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 )
	{
		BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		USHORT * pBufferRun = (USHORT*) pBuffer;


		for( int y = 0 ; y < DEPTH_SIZE_Y ; y++ )
		{
			//phout<<gsnl;
			for( int x = 0 ; x < DEPTH_SIZE_X ; x++ )
			{
				int id = element(x,y);
				points[id].active = true;
				if(use_depth)
				{
					Vector4 v = NuiTransformDepthImageToSkeleton(x,y,*pBufferRun); 
					setPointPos(x,y,v.x,v.y,v.z);
					pBufferRun++;

					if(v.z==0)
						points[id].active = false;
				}
				else
				{
					setPointPos( x,y,(float)x/DEPTH_SIZE_X,-(float)y/DEPTH_SIZE_X,0 );

				}

			}
		}
	}
	else
	{
		phout<<"Buffer length of received texture is bogus\r\n" ;
	}

	NuiImageStreamReleaseFrame(depthStreamHandle, pImageFrame );
}


GsVec Kinect::getPoint( int i)
{
	return points[i].pos;
}
GsVec Kinect::getPoint( int x,int y)
{
	return points[element(x,y)].pos;
}
void Kinect::setPointPos(int x,int y,float px,float py,float pz)
{
	points[element(x,y)].pos.set(px,py,pz);
}
void Kinect::setPointColor( int x,int y,GsColor c)
{
	if( x>=0 && x < DEPTH_SIZE_X-1 && y >=0 && y <DEPTH_SIZE_Y-1 )
	{
		points[element(x,y)].col = c;
	}
}

Kinect::Kinect()
{
	use_color = true;
	use_depth = true;
}

void Kinect::close()
{
	Nui_UnInit();
}

void Kinect::init()
{
	Nui_Init();
}

bool Kinect::hasNewData()
{
	if(_new_data)
	{
		_new_data = false;
		return true;
	}
	return false;
}
