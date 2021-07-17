# pragma once

# include <gsim/gs_vars.h>
# include <gsim/fl_vars_win.h>
#include <vector>

# include "data_glove_win_fl.h"
class DataGlove;
class FileManager;

class DataGloveWin : public DataGloveFluid
 { 
 private :
	 DataGlove* _glove;
	 FileManager* files;
   public :

    DataGloveWin (DataGlove* glove);
   ~DataGloveWin ();
    void show ();
	void update();

    void event ( GloveEvent e );
	bool gloveActive();
	void setData( std::vector<float>* nout );
	
};

