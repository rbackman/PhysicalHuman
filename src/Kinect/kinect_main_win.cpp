
# include <gsim/fl.h>
# include <gsim/fl_vars_win.h>

#include <gsim/dv_netclient.h>
#include <gsim/dv_netmdglove.h>
#include <gsim/dv_interfacedefines.h>
#include <gsim/dv_interdata.h>

//#include "app_defines.h"
//#include "app_dgloveprofile.h"


# include "kinect_main_win.h"
#include "kinect_manager.h"
#include "util_manipulator.h"
#include "kinect_marker.h"
#include "util_serializable_editor.h"
#include "point_cloud.h"
#include "util_file_manager.h"
#include "neural_net.h"
#include "data_glove.h"
#include "data_glove_win.h"
#include <fstream>

# define CHKVAR(s) if(_vars->search(s)<0) { fltk::alert("Missing parameter [%s]!",s); exit(0); }

#define D_IN  IMG_W*IMG_H

KinectMainWin::KinectMainWin ()
{
	
	files = new FileManager("..\\data");
	skelWin =0;
	ui_viewer->init ( this );
	ui_hand_viewer->init(this);
	_real_hand_image = new GsImage;
	_synth_hand_image = new GsImage;
	_diff_hand_image = new GsImage;
	_diff_hand_image->init(ui_diff_viewer->w(),ui_diff_viewer->h());
	_real_hand_image->init(ui_real_depth_viewer->w(),ui_real_depth_viewer->h());
	_synth_hand_image->init(ui_synth_depth_viewer->w(),ui_synth_depth_viewer->h());
	ui_image_set_name->value("default");
	_neural_net = 0;
	kinect = 0;

	handImage = 0;
	reducedImage = 0;

	glove = new DataGlove(0);
	
	if(glove->openConnection())
	{
		glove->loadConfig(files->getFile("glove\\glove.cfg"));
		gloveWin = new DataGloveWin(glove);
	}
	else
	{
		delete glove;
		glove = 0;
	}
	

	loadConfigFiles();
}

KinectMainWin::~KinectMainWin ()
{
}

void KinectMainWin::show ()
{
	updateUI();
	ui_neural_net_cfg->value("net\\net.cfg");
	//message ( "Ready" );
	ui_window->show();
	//ui_hand_window->show();
	

	
}

int KinectMainWin::selectedKinect()
{
	return ui_kinect_selected->value();
}
void KinectMainWin::event ( KinectEvent e )
{

	switch ( e )
	{
	case evConvertToText:
		convertToText();
		break;
	case evShowGloveWin:
		gloveWin->show();
		break;
		case evKinectActive:
			{
				makeKinect();
			}
			break;
		
	case evConvertDatabase:
		{
			convertDatabase();

		}
		break;
	case evLoadNet:
		{
			if(_neural_net)
				delete _neural_net;

			_neural_net = new NeuralNet(files->getFile(ui_neural_net_cfg->value()));
			IMG_W = _neural_net->dimInW();
			IMG_H = _neural_net->dimInH();
			DIM_OUT = _neural_net->dimOut();

			ui_net_parms[0]->value(IMG_W);
			ui_net_parms[1]->value(IMG_H);
			ui_net_parms[2]->value(_neural_net->dimIn());
			ui_net_parms[3]->value(DIM_OUT);
			ui_net_parms[4]->value(_neural_net->getNumTrain());
			ui_net_output->minimum(0);
			ui_net_output->maximum(DIM_OUT-1);
		}
		break;
	case evShowHandWin:
		ui_hand_window->show();
		break;

	case evLoadSkelWin:
		{
			if( skelWin==0)
			{
				skelWin = new FlSkeletonWin;
				skelWin->add(ui_hand_viewer->getHandSkel());
			}
			skelWin->show();
		}
		break;

	case evKinectQuit: 
		
		ui_window->hide();
		break;
	}
	///kinect related events
	if(kinect)
	{

	Kinect* k = kinect->currentKinect();
	switch ( e )
	{

	case evKinectQuit: 
		k->close();

		break;
	case evFilterImage:
		

		bilateral_filter(kinect->currentImage(),ui_sigd->value(),ui_sigr->value(),(int)ui_hh->value(),(int)ui_hw->value());
		if(kinect->getCloudImage(_synth_hand_image))
		{
			ui_synth_depth_viewer->setScale(ui_synth_image_scale->value());
			ui_synth_depth_viewer->setImage(_synth_hand_image);
		}
		break;

	case evStartRecording:
		

		_record_count = 0;
		if(!ui_rec_clouds->value())
		{
			phout<<"saving files\n";

			for (int i=0;i<kinect->numClouds();i++)
			{
				kinect->saveCloud(i);
			}
		}
		break;
	case evToggleDepthMode:
		ui_hand_viewer->depthMode = !ui_hand_viewer->depthMode;
		ui_hand_viewer->redraw();
		break;
	case  evShowImageWin:
		ui_depth_window->show();
		break;
	case evRefreshImageView:
		refreshImageView();
			break;
	case  evConfigListSelected:

		{
		

			GsString name =  ui_config_list->child(ui_config_list->value())->label();
			GsString lab = "../data/kinect/";
			lab<<name<<".cfg";

			phout<<"loading from "<<lab<<gsnl;
			kinect->load(lab);
			loadPointCloudFiles(name);
			setUIFromKinect();
		}
		break;
	case evSaveConfig:
		{
			

			GsString filename = kinect->getFileName();
			GsString name = kinect->getFileName();
			remove_filename(filename);

			remove_path(name);
			remove_extension(name);

			if(fl_string_input("config file name ","enter name of config file",name))
			{
				GsString cmd = "mkdir ";
				cmd << "pointclouds"<<name;
			
				files->makeDirectory(cmd);

				filename << name<<".cfg";
				phout<<"saving new file "<<filename;

				GsOutput op;
				op.open(filename);
				op<<kinect->toString();

				for (int i=0;i<kinect->numMarkers();i++)
				{
					op<<kinect->getMarker(i)->toString();
				}
				op.close();
			}
			loadConfigFiles();
		}
		break;

	case evEditKinectManager:
		{
		

			SerializableEditor* editor = new SerializableEditor(kinect);
			editor->ui_serializable_editor->label("KinectManager");
			editor->show();
		}
		break;
	
	case  evCheckCloud:
		{
			
			kinect->findClosestCloud();
		}
		break;
	case evChangeCloud:
		{
			
			kinect->selectCloud(ui_cloud_list->value());
			kinect->getCloudImage(_synth_hand_image);
			ui_synth_depth_viewer->setScale(ui_synth_image_scale->value());
			ui_synth_depth_viewer->setImage(_synth_hand_image);
			ui_viewer->redraw();
		}
		break;

	case evSaveHandCloud:
		{
			

			GsString cloudname = randomString();

			if(kinect->currentCloud())
				cloudname = kinect->currentCloud()->getShortFileName();
			
			GsString configName = kinect->getShortFileName();
			GsString filename = "../data/pointclouds/";
			filename<< configName <<"/" <<cloudname;
			kinect->saveHandCloud(filename);
			
		}

		break;
	case evSaveImage:
		{
			saveImage();

			

		}
		break;
	case evCaptureHand:
		{
			
			if(kinect->captureHandCloud((float)ui_cloud_val->value()))
			{
				ui_hand_viewer->addRealHandPoints(kinect->getHandPoints());
				GsString name = randomString();
				if(fl_string_input("enter a name for the point cloud","cloud name",name))
				{
					kinect->currentCloud()->setShortFilename(name);
					ui_cloud_list->add(name);
				}

			}

		}
		break;

	case    evDeletePoints:
		ui_hand_viewer->deletePoints();
		break;

	case evMakePoints:
		ui_hand_viewer->makePointCloud();
		break;
	case  evChangeMarker:
		{
			

			if(ui_marker_list->children()>0)
			{
				GsString selected = ui_marker_list->child(ui_marker_list->value())->label();
				phout<<"selected "<<selected<<gsnl;
				kinect->selectMarker(selected);
				updateUI();
			}
		}
		break;
	case evChangeType:
		{
			

			if(ui_marker_list->children()>0)
			{
				GsString mtype = ui_marker_type->child(ui_marker_type->value())->label();
				if(kinect->getCurrentMarker())
					kinect->getCurrentMarker()->setType(mtype);
				phout<<"setting to type "<<mtype<<gsnl;
			}
		}
		break;

	case evFingerRadiusAdjust:
		

		for (int i=0;i<kinect->numMarkers();i++)
		{
			kinect->getMarker(i)->model->getModel()->make_sphere(GsVec(0,0,0),(float)ui_finger_radius->value(),12,1);
			kinect->getMarker(i)->model->setColor(kinect->getMarker(i)->getColor());
			kinect->getMarker(i)->update();
		}
		break;
	case evFingerAdjust:
		{
			

			for (int i=0;i<kinect->numMarkers();i++)
			{
				kinect->getMarker(i)->sample = (float)ui_sample->value();

				kinect->getMarker(i)->max_v = (float)ui_max_v->value();

			}
		}
		break;
	case  evDeleteLastFinger: kinect->deleteLastMarker(); break;
	case  evClearFinger: kinect->clearMarkers(); break;
	case  evCalibrate: kinect->calibrateFingers(); break;

	case evTrans:
		{

			ui_viewer->setTrans(selectedKinect(),(float)ui_kinect[0]->value(),(float)ui_kinect[1]->value(),(float)ui_kinect[2]->value(),(float)ui_kinect[3]->value(),(float)ui_kinect[4]->value(),(float)ui_kinect[5]->value());
		}
		break;
	case evAngle:
		k->setAngle((int)ui_angle->value());
		break;
	
	case evRangeAdjust:

		setKinectFromUI();
		break;


	}
	}
}

void KinectMainWin::setKinectFromUI()
{
	Kinect* k = kinect->currentKinect();

	kinect->setP(kinect_manager_range,(float)ui_range[0]->value(),0);
	kinect->setP(kinect_manager_range,(float)ui_range[1]->value(),1);
	kinect->setP(kinect_manager_range,(float)ui_range[2]->value(),2);
	kinect->setP(kinect_manager_clip_box_size,(float)ui_range[3]->value());
	kinect->setP(kinect_manager_hand_dist,(float)ui_hand_var->value());
	kinect->setP(kinect_manager_hand_proximity,ui_hand_proximity->value());
	kinect->setP(kinect_manager_use_color,ui_use_color->value());
	kinect->setP(kinect_manager_use_depth,ui_use_depth->value());
	kinect->setP(kinect_manager_use_all_points,ui_all_points->value());
	kinect->setP(kinect_manager_track_colors,ui_track_colors->value());
	kinect->setP(kinect_manager_draw_markers,ui_draw_markers->value());
	kinect->setP(kinect_manager_draw_mesh,ui_draw_mesh->value());
	kinect->setP(kinect_manager_draw_output,ui_draw_output->value());
	kinect->setP(kinect_manager_use_clip_box,ui_use_clip_box->value());
	kinect->setP(kinect_manager_hand_color_variance,(int)ui_hand_color_var->value());
	kinect->setP(kinect_manager_hand_color_proximity,ui_hand_color_proximity->value());
	kinect->setP(kinect_manager_clip_box_visible,ui_clip_box_vis->value());


	ui_viewer->_surfGroup->visible(ui_draw_mesh->value());

	kinect->applyParameters();
}
void KinectMainWin::setUIFromKinect()
{
	Kinect* k = kinect->currentKinect();
	ui_range[0]->value( kinect->pFloat(kinect_manager_range,0) );
	ui_range[1]->value(kinect->pFloat(kinect_manager_range,1));
	ui_range[2]->value(kinect->pFloat(kinect_manager_range,2));
	ui_range[3]->value(kinect->pFloat(kinect_manager_clip_box_size));
	ui_hand_var->value(kinect->pFloat(kinect_manager_hand_dist)); 

	ui_draw_markers->value(kinect->pBool(kinect_manager_draw_markers));
	ui_use_color->value(kinect->pBool(kinect_manager_use_color));
	ui_use_depth->value(kinect->pBool(kinect_manager_use_depth));
	ui_all_points->value(kinect->pBool(kinect_manager_use_all_points));
	ui_track_colors->value(kinect->pBool(kinect_manager_track_colors));
	ui_draw_mesh->value(kinect->pBool(kinect_manager_draw_mesh));
	ui_draw_output->value(kinect->pBool(kinect_manager_draw_output));
	ui_use_clip_box->value(kinect->pBool(kinect_manager_use_clip_box));

	ui_hand_color_var->value(kinect->pInt(kinect_manager_hand_color_variance));
	ui_hand_color_proximity->value(kinect->pBool(kinect_manager_hand_color_proximity));
	ui_clip_box_vis->value(kinect->pBool(kinect_manager_clip_box_visible));

	ui_marker_list->remove_all();
	for (int i=0;i<kinect->sizeOfParameter(kinect_manager_marker_list);i++)
	{
		ui_marker_list->add(kinect->pString(kinect_manager_marker_list,i));
	}

}
void KinectMainWin::makeKinect( )
{
	

	if(kinect)
	{
		delete kinect;
		kinect = 0;
		if(!ui_kinect_active->value())
			return;
	}

	kinect = new KinectManager;
	kinect->init();

	handImage = new GsImage;
	reducedImage = new GsImage;

	ui_viewer->setKinectManager(kinect);

	ui_viewer->getRoot()->add(kinect->getGroup());
	for (int i=0;i<kinect->sizeOfParameter(kinect_manager_marker_types);i++)
	{
		ui_marker_type->add(kinect->pString(kinect_manager_marker_types,i));
	}

	for(int i=0;i<kinect->getNumKinects();i++)
	{
		GsString name = "Kinect ";
		name<<i;
		ui_kinect_selected->add(name);
	}

	setUIFromKinect();

	loadPointCloudFiles("default");
}
void KinectMainWin::update()
{
	if(glove && gloveWin->gloveActive())
	{	
		glove->readDeviceData();
		//glove->printData();
		gloveWin->update();
	}

	
	//kinect->currentKinect()->lock();
	if(kinectActive())
	{
		kinect->update();
	

	if(kinect->hasNewData())
	{
		kinect->getHandImage(_real_hand_image);

		if(ui_stream_net_output->value() && _neural_net)
			propogateImage();

		ui_viewer->updatePoints(0, kinect->getPoints());
		ui_hand_viewer->update();
		//ui_hand_viewer->redraw();
		ui_viewer->redraw();
	
		if(ui_stream_images->value())
		{
			saveImage();
		}

		if(ui_kinect_stream->value())
		{
			kinect->getHandImage(_real_hand_image);
		
			if(ui_kinect_image_filter->value())
				bilateral_filter(_real_hand_image,ui_sigd->value(),ui_sigr->value(),(int)ui_hh->value(),(int)ui_hw->value());

			ui_real_depth_viewer->setScale(ui_kinect_image_scale->value());
			ui_real_depth_viewer->setImage(_real_hand_image);
		}
		if(ui_check_stream->value())
		{
			kinect->findClosestCloud();
			if(kinect->getCloudImage(_synth_hand_image))
			{
			ui_synth_depth_viewer->setScale(ui_synth_image_scale->value());
			ui_synth_depth_viewer->setImage(_synth_hand_image);
			}

		}
		if(ui_synth_stream->value())

		{
			ui_hand_viewer->getHandImage(_synth_hand_image);
			ui_synth_depth_viewer->setScale(ui_synth_image_scale->value());
			ui_synth_depth_viewer->setImage(_synth_hand_image);
		}
		if(ui_diff_stream->value())
		{
			unsigned int diff = calculateDiff(_real_hand_image,_synth_hand_image,_diff_hand_image);
			ui_diff_val->value(diff);
			ui_diff_viewer->setScale(ui_diff_image_scale->value());
			ui_diff_viewer->setImage(_diff_hand_image);
		}
		if(ui_rec_clouds->value())
		{
			_record_count++;

			if(kinect->captureHandCloud((float)_record_count))
			{
				if(ui_kinect_image_filter->value())
				{
					bilateral_filter(kinect->currentImage(),ui_sigd->value(),ui_sigr->value(),(int)ui_hh->value(),(int)ui_hw->value());
				}
				GsString name = "cap_";
				name<<_record_count;
				kinect->currentCloud()->setShortFilename(name);
				ui_cloud_list->add(name);
			}
		}
	}
	}
}
void KinectMainWin::updateUI()
{
	if(!kinect)
		return;

	ui_marker_list->remove_all();
	for (int i=0;i<kinect->numMarkers();i++)
	{
		ui_marker_list->add(kinect->getMarker(i)->name());
	}

	if(kinect->getCurrentMarker())
	{
		GsString tname = kinect->getCurrentMarker()->pString(blob_type);

		for (int i=0;i<ui_marker_type->size();i++)
		{
			if(ui_marker_type->child(i)->label() == tname)
			{
				ui_marker_type->value(i);
			}
		}

	}
}

#include <gsim/gs_scandir.h>
void KinectMainWin::loadPointCloudFiles(GsString configname)
{
	ui_cloud_list->remove_all();
	kinect->initClouds();

	GsStrings files;
	GsStrings dir;
	GsStrings ext;
	ext.push("cld");
	GsString scene_dir = "../data/pointclouds/";
	scene_dir<<configname<<"/";

	gs_scandir(scene_dir,dir,files,ext);

	for (int j=0;j<files.size();j++)
	{
		GsString m = files[j];
		remove_extension(m);
		remove_path(m);
		ui_cloud_list->add(m,0,0);
		kinect->loadCloudFromFile(files[j]);
	}

}
void KinectMainWin::loadConfigFiles()
{
	ui_config_list->remove_all();

	GsStrings files;
	GsStrings dir;
	GsStrings ext;
	ext.push("cfg");
	GsString scene_dir = "../data/kinect/";
	gs_scandir(scene_dir,dir,files,ext);

	for (int j=0;j<files.size();j++)
	{
		GsString m = files[j];
		remove_extension(m);
		remove_path(m);
		ui_config_list->add(m,0,0);
	}

}

void KinectMainWin::refreshImageView()
{
	//should already have image
	//kinect->getHandImage(_real_hand_image);
	ui_real_depth_viewer->setScale(ui_kinect_image_scale->value());
	ui_real_depth_viewer->setImage(_real_hand_image);

	ui_hand_viewer->getHandImage(_synth_hand_image);
	ui_synth_depth_viewer->setScale(ui_synth_image_scale->value());
	ui_synth_depth_viewer->setImage(_synth_hand_image);

	unsigned int diff = calculateDiff(_real_hand_image,_synth_hand_image,_diff_hand_image);
	ui_diff_val->value(diff);
	ui_diff_viewer->setScale(ui_diff_image_scale->value());
	ui_diff_viewer->setImage(_diff_hand_image);
}

unsigned int KinectMainWin::calculateDiff( GsImage* i1, GsImage* i2, GsImage* output )
{
	int c1;
	int c2;
	int dif;
	unsigned int res = 0;
	
// 	if(i1->w()!=i2->w() || i1->h()!= i2->h())
// 	{
// 		phout<<"image dimensions don't match ";
// 		output->init(1,1);
// 		return 0;
// 	}

	output->init(max(i1->w(),i2->w()),max(i1->h(),i2->h()));

	for (int i=0;i<output->w();i++)
	{
		for(int j=0;j<output->h();j++)
		{
			c1 = i1->ptpixel(j*i1->h()/output->h(),i*i1->w()/output->w())->r;
			c2 = i2->ptpixel(j*i2->h()/output->h(),i*i2->w()/output->w())->r;
			if(c1>0 && c2>0)
			{
				dif =  abs(c1-c2);
				output->ptpixel(j,i)->set(dif,dif,dif);
				res += dif;
			}
			else
				output->ptpixel(j,i)->set(0,0,0);
		}
	}
	return dif;
}

void KinectMainWin::saveImage()
{
	int num = (int)ui_image_number->value();
	ui_image_number->value(num+1);
	
	//should already have image
	//kinect->getHandImage(_real_hand_image);
	gsbyte val;
	if(ui_embed_jnt_angles->value() && glove)
	{
		gsout<<"data: ";
		for (int i=0;i<DIM_OUT;i++)
		{
			val = 	(gsbyte) (glove->getData()[i]*255);
			gsout<<val<<" ";
			_real_hand_image->ptpixel(0,i)->r =	(gsbyte) (glove->getData()[i]*255);
			_real_hand_image->ptpixel(0,i)->g=0;
			_real_hand_image->ptpixel(0,i)->b=0;
			_real_hand_image->ptpixel(0,i)->a=1;
		}
		gsout<<gsnl;
	}
	else if(ui_embed_pose_interp->value() && glove)
	{
		val = (gsbyte) (glove->getPoseInterp()*255);
		gsout<<"interp value: "<<val<<gsnl;;
		for (int i=0;i<_real_hand_image->w();i++)
		{
			
			_real_hand_image->ptpixel(0,i)->r = val;
			_real_hand_image->ptpixel(0,i)->g=0;
			_real_hand_image->ptpixel(0,i)->b=0;
			_real_hand_image->ptpixel(0,i)->a=1;
		}
		
	}
	else if(ui_embed_manual_val->value())
	{
		int val = ui_pose_number->value()-1;

		for (int i=0;i<_real_hand_image->w();i++)
		{
			_real_hand_image->ptpixel(0,i)->r=0;
			_real_hand_image->ptpixel(0,i)->g=0;
			_real_hand_image->ptpixel(0,i)->b=0;
			_real_hand_image->ptpixel(0,i)->a=255;
		}
		_real_hand_image->ptpixel(0,val)->r=255;
	}

	GsString filename = "../data/images/";
	filename<< ui_image_set_name->value();

	GsString dir =  "images\\";
	dir <<ui_image_set_name->value();
	files->makeDirectory(dir);


	filename<<"/image_" <<num<<".png";
	gsout<<"save file "<<filename<<gsnl;
	_real_hand_image->save(filename);
}
void KinectMainWin::convertToText()
{
	int w = (int)ui_image_w->value();
	int h = (int)ui_image_h->value();

	GsString dir = "images\\";
	dir<< ui_image_set_name->value()<<"\\";

	gsout<<"looking for files in "<<dir<<gsnl;
	GsStrings file_list;
	files->getFiles(dir, file_list,"png");
	GsImage* img = new GsImage;
	GsColor* col;
	GsString inputFilename = dir;
	inputFilename<<"input.txt";
	GsString outputFilename = dir;
	outputFilename<<"output.txt";

	gsout<<"loading "<<file_list.size()<<" \n"<<inputFilename<<gsnl<<outputFilename<<gsnl;;
	std::ofstream inputFile(files->getFile(inputFilename));
	std::ofstream outputFile(files->getFile(outputFilename));

	for (int i=0;i<file_list.size();i++)
	{
		img->load(files->getFile(file_list[i]));

		for (int k=1;k<img->h();k++) //first column is output
		{
			for (int j=0;j<img->w();j++)
			{//add the last row of pixels which is the joint angles
				col = img->ptpixel(k,j);
				inputFile<<(int)col->r<<" ";
			}
		}
		inputFile<<"\n";
		outputFile<<(int)img->ptpixel(0,0)->r<<"\n";
	}
	inputFile.close();
	outputFile.close();
	gsout<<"done converting\n";
}

void KinectMainWin::convertDatabase()
{
	int w = (int)ui_image_w->value();
	int h = (int)ui_image_h->value();

	GsString dir = "images\\";
	dir<< ui_image_set_name->value()<<"\\";
	
	

	gsout<<"looking for files in "<<dir<<gsnl;
	GsStrings file_list;
	files->getFiles(dir, file_list,"png");
	GsImage* img = new GsImage;
	GsImage* old_image = new GsImage;
	GsColor* col;
	img->init(w,h+1);

	float scaleImageX = 1;
	float scaleImageY = 1;

	img->init(w,h+1);

	GsString sub_set_name;
	sub_set_name<<w<<"x"<<h;

	GsString sub_d_name = "images\\";
	sub_d_name<<ui_image_set_name->value()<<SLASH<<sub_set_name<<"\\";

	files->makeDirectory(sub_d_name);

	for (int i=0;i<file_list.size();i++)
	{

		old_image->load(files->getFile(file_list[i]));
		
		bilateral_filter(old_image,20,20,4,4);

		if(ui_preserve_aspect_ratio->value())
		{
			if (old_image->h()>old_image->w())
			{
				scaleImageX = (float)old_image->h()/h;
				scaleImageY = scaleImageX;
			}
			else
			{
				scaleImageX = (float)old_image->w()/w;
				scaleImageY = scaleImageX;
			}
		}
		else
		{
			scaleImageX = (float)old_image->w()/w;
			scaleImageY = (float)old_image->h()/h;
		}

		//scaleImage(img,old_image,w,h);

		for (int j=0;j<w;j++)
		{//add the last row of pixels which is the joint angles
			col = old_image->ptpixel(0,j);
			img->ptpixel(0,j)->set(col->r,col->g,col->b);
		}
		int x,y;


		for (int j=0;j<w;j++)
		{
			for (int k=0;k<h;k++)
			{
				
				y = (int)(scaleImageY*k);
				if(y==0)//make sure I dont use the output for the image
					y=1;

				x =  (int)(scaleImageX*j);
				
				if(x<old_image->w() && y<old_image->h())
				{
					col = old_image->ptpixel(y,x);
					img->ptpixel(k+1,j)->set(col->r,col->g,col->b);
				}
				else
				{
					img->ptpixel(k+1,j)->set(0,0,0);
				}
				
			}
		}
		GsString new_file_name = file_list[i];
		remove_path(new_file_name);
		//gsout<<"new file name "<<new_file_name<<gsnl;

		GsString new_file_path = sub_d_name;
		new_file_path << new_file_name;
		gsout<<"new file made "<<files->getFile(new_file_path)<<gsnl;
		img->save(files->getFile(new_file_path));
	}
	delete old_image;
	
}

bool KinectMainWin::kinectActive()
{
	return ui_kinect_active->value() && kinect;
}

void KinectMainWin::propogateImage()
{
	if(!kinect)
	{
		gsout<<"Must activate kinect to propogate image\n";
		return;
	}
	if(_real_hand_image->w()==0 || _real_hand_image->h() == 0)
	{
		//need an image to work
		return;
	}
	int w = IMG_W;
	int h = IMG_H;
	reducedImage->init(w,h);
	GsColor* col;
	int cnt = 0;
	std::vector<float> vin;
	std::vector<float> vout;
	
	for (int j=0;j<w;j++)
	{
		for (int k=0;k<h;k++)
		{
			int x;
			int y;
			float scaleX =1;
			float scaleY = 1;
			if (ui_preserve_aspect_ratio->value())
			{
				if(_real_hand_image->h()>_real_hand_image->w())
				{
					scaleX = (float)_real_hand_image->h()/h;
					scaleY = scaleX;
				
				}
				else
				{
					scaleX = (float)_real_hand_image->w() /w;
					scaleY = scaleX;
				
				}
			}
			else
			{
				scaleX = (float)_real_hand_image->w() /w;
				scaleY = (float)_real_hand_image->h() /h;
			}

			y   =(int)(scaleY*k);
			x   =(int)(scaleX*j);

			col = _real_hand_image->ptpixel(x,y);
			reducedImage->ptpixel(k,j)->set(col->r,col->g,col->b);
			vin.push_back(col->r/255.0f);
		}
	}
	if(gloveWin->gloveActive() && glove)
	{
		for (int j=0;j<DIM_OUT;j++)
		{
			vout.push_back(glove->getData()[j]);
		}
	}
	else
	{
		for (int j=0;j<DIM_OUT;j++)
		{
			vout.push_back(0);
		}
	}
	_neural_net->propogateForward(&vin);

	std::vector<float> nout = _neural_net->getOutput();
	gloveWin->setData(&nout);
	int maxIdx = 0;
	float maxV = 0;
	for (int i=0;i<nout.size();i++)
	{
		if(nout[i]>maxV)
		{
			maxV = nout[i];
			maxIdx = i;
		}
	}
	ui_net_output->position(maxIdx);
	/*
	
	gsout<<"output: ";
	for (unsigned int j=0;j<nout.size();j++)
	{
		gsout<<nout[j]<<" ";
	}
	gsout<<gsnl;
	*/

}

