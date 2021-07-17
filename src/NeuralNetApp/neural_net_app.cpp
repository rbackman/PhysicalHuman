
# include <gsim/fl.h>

#include "data_glove.h"
#include "data_glove_win.h"
#include "neural_net.h"
#include "util_file_manager.h"
#include "neuralnet_win.h"


int main ( int argc, char** argv )
{
	
	//DataGlove* glove = new DataGlove(0);
	//glove->openConnection();
	//DataGloveWin* gloveWin = new DataGloveWin(glove);
	//gloveWin->show();
	
	NeuralNetWin* net_win = new NeuralNetWin;
	net_win->show();

	while(true)
	{
		fltk::check();
		net_win->update();
	}
	
	return 0;
}


