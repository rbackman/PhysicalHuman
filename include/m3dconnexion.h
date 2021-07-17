# ifndef M3DCONNEXION_H
# define M3DCONNEXION_H

//========================================================================
// Simple interface to read values from a 3d mouse device via direct input
// Marcelo Kallmann 2008
//========================================================================

/*! Initialize the DirectInput variables and return the number of 3d mice found.
    No effect if already initilized. If zero is returned nothing was found. */
int m3d_init ();

/*! Free the input devices connected with m3d_init() */
void m3d_stop ();

/*! Returns num of devices; valid ids will be in {0,...,num-1} */
int m3d_devices ();

/*! Read the input device's state */
bool m3d_read ( float values[6], char buttons[21], int deviceid=0 );

/*! Set the maximum input values considered for translation and rotation axes */
void m3d_sensitivity ( int maxtr=2000, int maxrot=2000, int deviceid=0 );

# endif // M3DCONNEXION_H
