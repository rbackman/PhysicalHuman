
# include <gsim/fl.h>


# include "neuralnet_win.h"
#include "neural_net.h"
#include "util_file_manager.h"
#include <fstream>

# define CHKVAR(s) if(_vars->search(s)<0) { fltk::alert("Missing parameter [%s]!",s); exit(0); }

//#define NET_CONFIG_FILE "net\\net2.cfg"
//#define NET_IMG_DIR "test2\\8x8"
#define STORE_IMAGES

NeuralNetWin::NeuralNetWin ()
{
	
	_hidden_image = new GsImage;
	_output_image = new GsImage;
#ifdef STORE_IMAGES
	input_image = 0;
#else
	input_image = new GsImage;
#endif
	

	net = 0;
	
	files = new FileManager("..\\data");


	 batch_update = false;
	t_total = 0;
	t_report = 10;
	sse =0;
	test_sse =0;

	IMG_W = 8;
	IMG_H = 8;

	ui_net_data_dir->value("test2\\8x8");
	ui_net_cfg->value("net\\net2.cfg");

	net = 0;



	
	

}

NeuralNetWin::~NeuralNetWin ()
{
}

void NeuralNetWin::show ()
{
	//message ( "Ready" );
	ui_window->show();
	
}

void NeuralNetWin::event ( NeuralNetEvent e )
{

	switch(e)
	{
	case evLoadNet:
		{
			loadNet(ui_net_cfg->value());
		}break;
	case evMakeNet:
		{
			error.clear();
			makeNet((int)ui_make[0]->value(),(int)ui_make[1]->value(),(int)ui_make[2]->value(),(int)ui_make[3]->value());
			net->init_weights();
			GsString input;
			if(fl_string_input("neural net config file","enter a name of the new neural net",input))
			{
				GsString filename = "net\\";
				filename<<input<<".cfg";
				net->saveToFile(files->getFile(filename));
			}

		}
		break;
	}

	if(!net)
	{
		gsout<<"Net not loaded \n";
		return;

	}


	switch ( e )
	{
	case evSaveData:
		saveFiles();
		break;
	case 	evRandomizeWeights:
		net->init_weights();
		updateImages();
		break;
	case evNoiseWeights:
		net->init_weights_noise((int)ui_noise_seeds->value());
		updateImages();
			break;
	case evLoadImages:
		{
			loadFiles(ui_net_data_dir->value());
		}break;
	
	case evLoadTestImages:
		loadTestFiles(ui_net_test_data_dir->value());
		break;
	case evStopTrain:
		t_total = 0;
		training = false;
		break;
	case evNetQuit:
		gs_exit();
		break;
	case evUpdateImages:
		updateImages();
		break;
	case 	evTrainImages:
		{
			img_num = 0;
			loadFiles(ui_net_data_dir->value());
			saveFiles();
			t_total = (int)ui_num_epoch->value();
			training = true;
		}
	break;
	case	evTestImages:
		loadFiles(ui_net_data_dir->value());
		t_total = 1;
		training = false;
	break;
	case evUpdateNetParms:
		{
			net->learning_rate = (float)ui_learning_rate->value();
			net->momentum =	(float)ui_momentum->value();
			net->weight_decay = (float)ui_weight_decay->value();
		}
		break;
	}
}


void NeuralNetWin::updateUI()
{
	ui_dim_in->value(net->dimIn());
	ui_dim_hid->value(net->dimHidden());
	ui_dim_out->value(net->dimOut());
	ui_learning_rate->value(net->learning_rate);
	ui_momentum->value(net->momentum);
	ui_weight_decay->value(net->weight_decay);
	ui_dim_in_w->value(net->dimInW());
	ui_dim_in_h->value(net->dimInH());
	ui_n_train->value(net->getNumTrain());
}



void NeuralNetWin::refreshImageView()
{

// 	ui_hand_viewer->getHandImage(_synth_hand_image);
 	ui_hidden_weights->setScale(ui_scale_images->value());
	ui_output_weights->setScale(ui_scale_images->value());
// 	ui_synth_depth_viewer->setImage(_synth_hand_image);

}

void NeuralNetWin::updateImages()
{
	float max = -100;
	float min = 100;
	int choice = ui_display_choice->value();
	float num;

	for (int c=0;c<net->dimHidden();c++)
	{
		for (int l=0;l<net->dimIn();l++)
		{
			if(choice == 0)
				num = net->hidden_unit->neurons[c].weights[l];
			else
				num = net->hidden_unit->neurons[c].dw_old[l];
			
			if(num<min)
				min = num;
			if(num>max)
				max = num;
		}
	}
	gsout<<"max :"<<max<<"  min :"<<min<<gsnl;
	for (int c=0;c<net->dimHidden();c++)
	{
		for (int l=0;l<net->dimIn();l++)
		{
			
			if(choice == 0)
				num = net->hidden_unit->neurons[c].weights[l];
			else
				num = net->hidden_unit->neurons[c].dw_old[l];

			num = ((num - min)/(max-min));
			_hidden_image->pixel(l,c) = GsColor::interphue(num);
		}
	}
	for (int c=0;c<net->dimOut();c++)
	{
		for (int l=0;l<net->dimHidden();l++)
		{
			if(choice == 0)
				num = net->output_unit->neurons[c].weights[l];
			else
				num = net->output_unit->neurons[c].dw[l];

			num  = ((num - min)/(max-min));
			_output_image->pixel(l,c) = GsColor::interphue(num);
		}
	}

	ui_hidden_weights->setScale(ui_scale_images->value());
	ui_output_weights->setScale(ui_scale_images->value());
	ui_hidden_weights->setImage(_hidden_image);
	ui_output_weights->setImage(_output_image);
}

void NeuralNetWin::update()
{
		if(t_total>0)
		{
			t_total--;
			gsout<<".";
			sse=0;
			test_sse=0;
			randomizePatterns();
			for (int i=0;i<file_list.size();i++)
			{
				propogateImage(files->getFile(file_list[pattern_indexes[i]]));
				if(training)
				{
					net->backPropogate(&input,&output);
					if (!batch_update)
					{
						net->updateWeights();
					}
				}
				sse+=net->getSSError(&output);
			}

			if(batch_update && training)
				net->updateWeights();
			
			sse = sse/file_list.size();

			error.push_back(sse);

			if(test_file_list.size()>0)
			{
				test_sse = 0;
				for (int i=0;i<test_file_list.size();i++)
				{
					propogateImage(files->getFile(test_file_list[i]));
					test_sse+=net->getSSError(&output);
				}
				test_sse/=test_file_list.size();
				test_error.push_back(test_sse);
			}

			if(t_total%t_report==0 || !training)
			{
			

				if(training)
				{
					saveFiles();

					printf("\nEPOCH %d: SSE = %g  TEST SSE = %g; \n",t_total,sse,test_sse);
				}
				else
				{
					printf("\nTEST: SSE = %g; \n",sse);
				}
				//logfile<<"EPOCH "<< e <<": TRAIN SSE = "<< sse <<"; TEST SSE = " <<test_sse << "  \n";
			}
		}

}

void NeuralNetWin::propogateImage( GsString file )
{
#ifdef STORE_IMAGES
	input_image = GsImage::sharedload(file);
#else
	input_image->load(file);
#endif
	
	int cnt = 0;
	for (int k=0;k<IMG_W;k++)
	{
		for (int l=1;l<=IMG_H;l++)
		{
			if(cnt<D_IN)
			{
				input[cnt] = input_image->ptpixel(l,k)->r / 255.0f;
				cnt++;
			}
			else
			{
				gsout<<"too many inputs\n";
			}
		}
	}
	for (int k=0;k<D_OUT;k++)
	{
		output[k] = input_image->ptpixel(0,k)->r / 255.0f;
	}
	net->propogateForward(&input);
}
void NeuralNetWin::loadTestFiles( const char* dirname )
{
	test_file_list.size(0);
	GsString d = "images\\";
	d<<dirname;
	files->getFiles(d,test_file_list,"png");
	gsout<<"loaded "<< test_file_list.size() <<" test files\n";
}


void NeuralNetWin::loadFiles( const char* dir )
{
	if(dirName!=dir)
	{
		file_list.size(0);
		dirName = dir;
		GsString d = "images\\";
		d<<dir;
		files->getFiles(d,file_list,"png");
		gsout<<"loaded "<< file_list.size() <<" files\n";
		randomizePatterns();
	}
}

void NeuralNetWin::makeNet(int dx,int dy,int dh,int dop)
{
	D_IN = dx*dy;
	D_HID = dh;
	IMG_W = dx;
	IMG_H = dy;
	D_OUT = dop;


	if(net)
		delete net;

	net = new NeuralNet(D_IN,D_OUT,D_HID,0,1,0.1f,0.03f,0,0);
	net->setImgDim(dx,dy);

	input.resize(D_IN);
	output.resize(D_OUT);

	_hidden_image->init(D_HID,D_IN);
	_output_image->init(D_OUT,D_HID);

#ifndef STORE_IMAGES
	input_image->init(IMG_W,IMG_H+1);
#endif // !STORE_IMAGES

	
}

void NeuralNetWin::loadNet( const char* file )
{
	if(net)
	{
		delete net;
	}
	net = new NeuralNet(files->getFile(file));

	GsString errorFile = files->getFile(file);
	remove_extension(errorFile);
	errorFile<<".error";
	std::ifstream eFile(errorFile);
	error.clear();
	if(eFile.is_open())
	{
		
		int num;
		eFile>>num;
		for(int i=0;i<num;i++)
		{
			float val;
			eFile >> val  ;
			error.push_back(val);
		}
		eFile.close();
	}

	D_IN = net->dimIn();
	D_HID = net->dimHidden();
	IMG_W = net->dimInW();
	IMG_H = net->dimInH();
	D_OUT = net->dimOut();

	input.resize(D_IN);
	output.resize(D_OUT);

	_hidden_image->init(D_HID,D_IN);
	_output_image->init(D_OUT,D_HID);

	updateUI();
	updateImages();
}

void NeuralNetWin::saveFiles()
{
	net->saveToFile(files->getFile(ui_net_cfg->value()));
	
	updateImages();

	GsString errorFile = files->getFile(ui_net_cfg->value());
	remove_extension(errorFile);
	errorFile<< "_images" ;
	files->makeDirectory(errorFile);
	errorFile<<"\\"<< img_num << ".png";
	img_num++;
	gsout<<"saving image "<<errorFile<<gsnl;
	_hidden_image->save(errorFile);

	if(error.size()>0)
	{
		GsString errorFile = files->getFile(ui_net_cfg->value());
		remove_extension(errorFile);
		errorFile<<".error";
		std::ofstream eFile(errorFile);
		eFile<<error.size()<<gsnl;
		for(unsigned int i=0;i<error.size();i++)
		{
			eFile<<error[i]<<"\n";
		}
		eFile.close();
	}
	if(test_error.size()>0)
	{
		GsString errorFile = files->getFile(ui_net_cfg->value());
		remove_extension(errorFile);
		errorFile<<".test_error";
		std::ofstream eFile(errorFile);
		eFile<<error.size()<<gsnl;
		for(unsigned int i=0;i<test_error.size();i++)
		{
			eFile<<test_error[i]<<"\n";
		}
		eFile.close();
	}
}

void NeuralNetWin::randomizePatterns()
{
	pattern_indexes.resize(file_list.size());
	for (unsigned int i=0;i<pattern_indexes.size();i++)
	{
		pattern_indexes[i] = i;
	}
	for (unsigned int i=0;i<pattern_indexes.size();i++)
	{
		int swapIndex = gs_random(0,pattern_indexes.size()-1);
		int temp = pattern_indexes[swapIndex];
		pattern_indexes[swapIndex] = pattern_indexes[i];
		pattern_indexes[i] = temp;
	}
}

