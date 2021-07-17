# pragma once

#include <windows.h>
#include <ole2.h>
#include <vector>

# include "neural_net_fluid.h"



class NeuralNet;
class FileManager;
class GsImage;


class NeuralNetWin : public NeuralNetFluid
 { 
 private :
	  std::vector<int> pattern_indexes;
	 std::vector<float> error;
	  std::vector<float> test_error;
	 int IMG_W;
	 int IMG_H;
	 int D_IN;
	int D_HID;
	 int D_OUT;
	 GsString dirName;
	NeuralNet* net;
	GsImage* _hidden_image;
	GsImage* _output_image;
	FileManager* files;
	bool training;
	bool batch_update;
	int t_total ;
	int t_report;
	float sse;
	float test_sse;
	std::vector<float> input;
	std::vector<float> output;
	GsImage* input_image;
	GsStrings file_list;
	GsStrings test_file_list;
	int img_num;
   public :
	
    NeuralNetWin ();
   ~NeuralNetWin ();
  
    void show ();

   public :
	   virtual void event ( NeuralNetEvent e  );

	   void loadNet(const char* file);

	   void convertDatabase();
	   void update();

	   void saveFiles();

	   void refreshImageView();
	   void updateUI();
	   void updateImages();
	   void propogateImage( GsString file );
	   void loadFiles( const char* dir );
	   void makeNet(int dx,int dy,int dh,int dop);
	   void randomizePatterns();
	   void loadTestFiles( const char* dirname );
};

