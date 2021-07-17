
# include "m3dconnexion.h"

# define DIRECTINPUT_VERSION 0x0800
# include <dinput.h>
# include <windows.h>

#include <stdio.h>
#include <string.h>

# define MAX_DEVICES 8  // No particularly reason for 8.  It just seems like plenty.  You may want to use a more flexible data structure.
# define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }
# define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }

//# define FAILED(Status) ((HRESULT)(Status)!=DI_OK)

# define MAXTR   2000
# define MAXROT  2000

//===== Types =====

enum VectorType {
	BothVectors = 0,
	BothZeroVectors,
	TranslationVector, 
	RotationVector,
};

class DIDeviceInfo
{ public:
	DIDEVICEINSTANCE     DIDevInstance;
	LPDIRECTINPUTDEVICE8 pUsbHidDevice;
	DIDEVCAPS            diDevCaps;
	HWND				 hDlg;
	HANDLE				 hUsbHidDeviceEvent;
	DIJOYSTATE			 lastJoyState;
	VectorType			 lastVectorType;
    float                maxtr, maxrot;
};

//===== Global Variables =====
static LPDIRECTINPUT8  g_pDI = NULL;
static int		       g_nDevices = 0;
static DIDeviceInfo   *g_pDevices[MAX_DEVICES];

//===== Static Functions =====

static BOOL CALLBACK EnumAxesCB( const DIDEVICEOBJECTINSTANCE* pdidoi, VOID* pContext );
static BOOL CALLBACK EnumDevicesCB( const DIDEVICEINSTANCE* pdidInstance, VOID* pContext );
static bool UpdateDI8State( float values[6], char buttons[8], DIDeviceInfo *pDIDeviceInfo );
static void FreeDI8Device( DIDeviceInfo *pdinfo );

//===== Interface Functions =====

//===========================================================================
//  Returns num of devices
//===========================================================================
int m3d_devices ()
 {
   return g_nDevices;
 }

//===========================================================================
//  Initialize the DirectInput variables.
//===========================================================================
int m3d_init ()
{
    if ( g_nDevices>0 ) return g_nDevices; // init only once

    HRESULT hr;

    // Register with the DirectInput subsystem and get a pointer
    // to a IDirectInput interface we can use.
    // Create a DInput object
    hr = DirectInput8Create( GetModuleHandle(NULL), DIRECTINPUT_VERSION, 
                             IID_IDirectInput8, (VOID**)&g_pDI, NULL );
    if( FAILED(hr) )
	{
       /* hr can be: DI_OK, DIERR_BETADIRECTINPUTVERSION, DIERR_INVALIDPARAM,
          DIERR_OLDDIRECTINPUTVERSION, or DIERR_OUTOFMEMORY. */
        return 0;
	}

    // Look for an HID device.
    hr = g_pDI->EnumDevices( DIDEVTYPE_HID, EnumDevicesCB, NULL, DIEDFL_ATTACHEDONLY );
    if ( FAILED(hr) ) return 0;

    // Make sure we got an HID device
    if( g_nDevices == 0 )
    {
        MessageBox( NULL, "3Dx device not found.", "M3D Init", 
                    MB_ICONERROR | MB_OK );
        //PostQuitMessage( 0 );
        return 0;
    }

	/* Setup each device */
	int i=0;
	for(i=0; i<g_nDevices; i++)
	{

		/* Set the data format to "simple joystick" - a predefined data format 
		 *
		 * A data format specifies which controls on a device we are interested in,
		 * and how they should be reported. This tells DInput that we will be
		 * passing a DIJOYSTATE structure to IDirectInputDevice::GetDeviceState().
		 * This is a fine format to use for the 3Dx Devoce as it has 6 DOF and upto
		 * 32 buttons.
		 */
        hr = g_pDevices[i]->pUsbHidDevice->SetDataFormat( &c_dfDIJoystick );
		if( FAILED(hr) ) 
		{
			MessageBox( NULL, "Data Format failed.", "M3D Init", 
                    MB_ICONERROR | MB_OK );
		}

		/* Set the cooperative level to let DInput know how this device should
		 * interact with the system and with other DInput applications.
		 * You probably don't want the dialog box to be the target of the events.
		 */
/*      
		hr = g_pDevices[i]->pUsbHidDevice->SetCooperativeLevel ( g_pDevices[i]->hDlg, 
				// DISCL_EXCLUSIVE | DISCL_FOREGROUND    // normal use:  get events when have focus
				 DISCL_NONEXCLUSIVE | DISCL_BACKGROUND ); // get events all the time 
		if( FAILED(hr) )
		{
			MessageBox( NULL, "Cooperative Level Failed.", "M3D Init", 
                    MB_ICONERROR | MB_OK );
		}
*/
		/* Determine how many axis the UsbHidDevice has (so we don't error out setting
		 * properties for unavailable axis)
		 */
		g_pDevices[i]->diDevCaps.dwSize = sizeof(DIDEVCAPS);
        hr = g_pDevices[i]->pUsbHidDevice->GetCapabilities(&(g_pDevices[i]->diDevCaps));
		if( FAILED(hr) ) 
		{
			MessageBox( NULL, "Get Capabilities Failed.", "M3D Init", 
                    MB_ICONERROR | MB_OK );
		}

		/* Enumerate the axes of the UsbHidDevice and set the range of each axis and
		 * make sure the axes are set to ABSOLUTE mode.
		 */
        hr = g_pDevices[i]->pUsbHidDevice->EnumObjects( EnumAxesCB, g_pDevices[i], DIDFT_AXIS );
		if( FAILED(hr) )
		{
			MessageBox( NULL, "Set Axis Failed.", "M3D Init", 
                    MB_ICONERROR | MB_OK );
		}

	}

    return i;
}

/*---------------------------------------------------------------------------*/
static BOOL CALLBACK EnumDevicesCB( const DIDEVICEINSTANCE* pdidInstance,
                             VOID* pContext )
/*
  Called once for each enumerated UsbHidSpaceball. If we find one, create a
  device interface on it so we can play with it.
-----------------------------------------------------------------------------*/
{
	int newIndex;
    HRESULT hr;
 	GUID guid = pdidInstance->guidProduct;
#   define LOGITECH_3DX_VID    0x046d  // Vendor ID for Logitech/3Dx

    // Check to see if this is a 3Dx device.  Look at the VendorID before stopping the enumeration.
	if ( (guid.Data1 & 0x0000ffff) != LOGITECH_3DX_VID)
		return DIENUM_CONTINUE;

	// Filter out LOGI mice and such
	else if (pdidInstance->wUsagePage != 0x0001 || pdidInstance->wUsage != 0x0008)
		return DIENUM_CONTINUE;

	// Make sure haven't found too many 3Dx devices for our array
	if (g_nDevices >= MAX_DEVICES)
		return DIENUM_CONTINUE;


	newIndex = g_nDevices;
    g_pDevices[g_nDevices++] = (DIDeviceInfo*)malloc( sizeof(DIDeviceInfo) );

    g_pDevices[newIndex]->maxtr = MAXTR;
    g_pDevices[newIndex]->maxrot = MAXROT;

	/* 
	 * Save the DIDEVICEINSTANCE struct for later display.
	 * This isn't necessary.  The demo just does it to display the info in the dlg box.
	 */
	g_pDevices[newIndex]->DIDevInstance = *pdidInstance;

	// Obtain an interface to the enumerated device.
    hr = g_pDI->CreateDevice( pdidInstance->guidInstance, &g_pDevices[newIndex]->pUsbHidDevice, NULL );

    /* If it failed, then we can't use this UsbHidDevice. (Maybe the user unplugged
     * it while we were in the middle of enumerating it.)
	 */
    if( FAILED(hr) ) 
	{
		free (g_pDevices[newIndex]);
		g_nDevices--;
        return DIENUM_CONTINUE;
	}

	/* Create a info dialog box for this device */
	//fix: g_pDevices[newIndex]->hDlg = CreateDialogParam( hInst, MAKEINTRESOURCE(IDD_DIALOG1), NULL, DlgProc, (LPARAM)g_pDevices[newIndex] );

    return DIENUM_CONTINUE;
}

/*---------------------------------------------------------------------------*/
BOOL CALLBACK EnumAxesCB( const DIDEVICEOBJECTINSTANCE* pdidoi,
                          VOID* pContext )
/*
   Callback function for enumerating the axes on a UsbHidDevice
-----------------------------------------------------------------------------*/
{
	DIDeviceInfo *pDIDeviceInfo = (DIDeviceInfo *)pContext;

#if 0 // this has no effect
	// Set the range for the axis
    DIPROPRANGE diprg; 
    diprg.diph.dwSize       = sizeof(DIPROPRANGE); 
    diprg.diph.dwHeaderSize = sizeof(DIPROPHEADER); 
    diprg.diph.dwHow        = DIPH_BYID; 
    diprg.diph.dwObj        = pdidoi->dwType; // Specify the enumerated axis
    diprg.lMin              = -512; // +/- 512 matches the hardware 1:1 
    diprg.lMax              =  511; 

	if( FAILED( pDIDeviceInfo->pUsbHidDevice->SetProperty( DIPROP_RANGE, &diprg.diph ) ) )
		return DIENUM_STOP;
#endif

	/*
	 * Recent 3Dx USB device descriptors indicate that the device axes return relative data
	 * even though the device is really an absolute device.
	 * HID ignores this, but apparently DI uses it.
	 * Changing the axis handling to return RELATIVE values gives us back what we want (displacement
	 * values rather than accumulated values).  If older USB devices were to be supported, this handling
	 * would have to be changed to DIPROPAXISMODE_ABS.  
	 *
	 * In this mode, DI sets the notification event even though the values haven't changed.
	 */
    DIPROPDWORD dipdw;
    dipdw.diph.dwSize       = sizeof(DIPROPDWORD); 
    dipdw.diph.dwHeaderSize = sizeof(DIPROPHEADER); 
    dipdw.diph.dwHow        = DIPH_DEVICE; 
    dipdw.diph.dwObj        = 0; // set for whole device not an axis (really only needs to be done once)
    dipdw.dwData            = DIPROPAXISMODE_REL;

	if( FAILED( pDIDeviceInfo->pUsbHidDevice->SetProperty( DIPROP_AXISMODE, &dipdw.diph ) ) )
		return DIENUM_STOP;
	
    return DIENUM_CONTINUE;
}

//===========================================================================
//   Free up a device.
//===========================================================================
void m3d_stop ()
 {
   int i;
   for ( i=0; i<g_nDevices; i++ ) FreeDI8Device(g_pDevices[i]);
 }

void FreeDI8Device( DIDeviceInfo *pdinfo )
{
	// Release the event
	if ( pdinfo->pUsbHidDevice )
		pdinfo->pUsbHidDevice->SetEventNotification( NULL );

    // Unacquire the device one last time just in case 
    // the app tried to exit while the device is still acquired.
    if( pdinfo->pUsbHidDevice ) 
        pdinfo->pUsbHidDevice->Unacquire();
    
    // Release any DirectInput objects.
    SAFE_RELEASE( pdinfo->pUsbHidDevice );

	free (pdinfo);
	pdinfo = NULL;
}

//===========================================================================
//   Get the input device's state and display it.
//===========================================================================
void m3d_sensitivity ( int maxtr, int maxrot, int deviceid )
 {
   if ( deviceid >=g_nDevices ) return;
   g_pDevices[deviceid]->maxtr = (float)maxtr;
   g_pDevices[deviceid]->maxrot = (float)maxrot;
 }

//===========================================================================
//   Get the input device's state and display it.
//===========================================================================
bool m3d_read ( float values[6], char buttons[8], int deviceid )
 {
   return UpdateDI8State(values,buttons,g_pDevices[deviceid]);
 }

static bool UpdateDI8State( float values[6], char buttons[8], DIDeviceInfo *pDIDeviceInfo )
{
    HRESULT     hr;
    DIJOYSTATE  js;           // DInput UsbHidSpaceball state 
	DIDeviceInfo dinfo = *pDIDeviceInfo;

    if( NULL == dinfo.pUsbHidDevice ) return false;

    /* 
	 * Poll the device to read the current state.
     */
    hr = dinfo.pUsbHidDevice->Poll(); 
    if( FAILED(hr) )  
    {
		//OutputDebugString("Poll failed\n");

        // DInput is telling us that the input stream has been
        // interrupted. We aren't tracking any state between polls, so
        // we don't have any special reset that needs to be done. We
        // just re-acquire and try again.
        hr = dinfo.pUsbHidDevice->Acquire();
        while( hr == DIERR_INPUTLOST ) 
            hr = dinfo.pUsbHidDevice->Acquire();

        // hr may be DIERR_OTHERAPPHASPRIO or other errors.  This
        // may occur when the app is minimized or in the process of 
        // switching, so just try again later 
        return false; 
    }

    // Get the input's device state
    if( FAILED( hr = dinfo.pUsbHidDevice->GetDeviceState( sizeof(DIJOYSTATE), &js ) ) )
	{
		//OutputDebugString("GetDeviceState failed\n");
        return false; // The device should have been acquired during the Poll()
	}

    // Get state
    # define SCALE(v,m)  float(v>=m? 1.0f : v<=-m? -1.0f : float(v)/m)
    float maxtr = pDIDeviceInfo->maxtr;
    float maxrot = pDIDeviceInfo->maxrot;
    values[0] = SCALE(js.lX,maxtr);
    values[1] = SCALE(js.lY,maxtr);
    values[2] = SCALE(js.lZ,maxtr);
    values[3] = SCALE(js.lRx,maxrot);
    values[4] = SCALE(js.lRy,maxrot);
    values[5] = SCALE(js.lRz,maxrot);

    unsigned int max = dinfo.diDevCaps.dwButtons;
    # define BUT(i) buttons[i] = i<max? (js.rgbButtons[i]?1:0) : 0
    BUT(0); BUT(1); BUT(2);
    BUT(3); BUT(4); BUT(5);
    BUT(6); BUT(7);

    /*
	// Launch Control Panel on button 1:
    if(js.rgbButtons[0])g_pDI->RunControlPanel( NULL, 0 );

	// Launch Joystick Control Panel on button 2
	typedef void (WINAPI* LPFNSHOWJOYCPL)( HWND hWnd );
	HMODULE hCPL;
	LPFNSHOWJOYCPL pShowJoyCPL;
	hCPL = LoadLibrary(TEXT("joy.cpl")); 
	pShowJoyCPL  = (LPFNSHOWJOYCPL)GetProcAddress(hCPL, TEXT("ShowJoyCPL"));
	if (js.rgbButtons[1]) if (pShowJoyCPL)	pShowJoyCPL(NULL);  //Invoke Joystick Control Panel
    */

    return true;
}
