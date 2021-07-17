
# include "util_image_viewer.h"
#include <gsim/gs_ogl.h>
# include <gsim/fl.h>
#include <gsim/gs_event.h>
#include "common.h"
ImageViewer::ImageViewer ( int x, int y, int w, int h, const char *l ) : FlViewer ( x, y, w, h, l )
 {

	 scale = false;
  // FlViewer::root ( _root );
	 _image = new GsImage;
	 _image->init(w,h);
	if(!_image->load("yosemite.bmp"))
	{
		phout<<"image load failed \n";
	}
	else
	 {
		 _image->vertical_mirror();
		 phout<<"loaded image\n";
		 redraw();
	}
 }

ImageViewer::~ImageViewer ()
 {

 }




int ImageViewer::handle_scene_event ( const GsEvent &e )
 {
   // window events can be custom-processed here:
   if ( e.button1 )
    {
    }
  
   if(e.type == GsEvent::Push)
   {
	
   }
   // now let the viewer process the remaining events:
   return FlViewer::handle_scene_event ( e );
 }

void ImageViewer::draw()
{
	glClearColor (GsColor::black);
	glClear ( GL_COLOR_BUFFER_BIT );
	
		glDrawPixels(_image->w(),_image->h(),GL_RGBA,GL_UNSIGNED_BYTE, &_image->r(0,0) );
}
