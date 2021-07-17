# pragma once


# include <gsim/fl_viewer.h>
#include <gsim/gs_image.h>

class Gs;
class Manipulator;


class ImageViewer : public FlViewer
 { private :
	
	GsImage* _image;
	bool scale;
   public:
    ImageViewer ( int x, int y, int w, int h, const char *l=0 );
   ~ImageViewer ();
   void setScale(bool b){scale = b;}
	void setImage(GsImage* image)
	{
		if(scale)
		{
			if(image->h()>0 && image->w()>0)
			{
				_image->init(w(),h());
				float scaleFactor = ((float)image->h())/((float)h());
				if(h()/image->h()>w()/image->w())
					scaleFactor = ((float)image->w()/((float)w()));

				for (int i=0;i<w();i++)
				{
					for(int j=0;j<h();j++)
					{
						float x = (i*scaleFactor);
						float y = (j*scaleFactor);
						if(x<image->w() && y<image->h())
						{
							*_image->ptpixel(j,i) = *image->ptpixel((int)y,(int)x);
						}
						else
							_image->ptpixel(j,i)->set(0,0,0);
					}
				}
			}
		}
		else
		{
			_image->init(image->w(),image->h());
			_image->buffer() = image->buffer(); 
		}

		redraw();
	}
	void draw();

	virtual int handle_scene_event ( const GsEvent &e );
};

