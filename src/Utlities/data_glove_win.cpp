
# include <gsim/fl.h>


#include "data_glove_win.h"

#include "data_glove.h"
#include "util_file_manager.h"

DataGloveWin::DataGloveWin(DataGlove* glove)
{
	_glove = glove;
	files = new FileManager("..\\data");
	for (unsigned int i=0;i<_glove->getSensor()->getNumSensors();i++)
	{
		std::string s = DataGlove::Sensor::PrintSensorType(i);
		
		gsout <<s.c_str()<<gsnl;
		const char* c = s.c_str();
		data[i]->label(c);
		
	}
	for (int i=_glove->getSensor()->getNumSensors();i<18;i++)
	{
		data[i]->hide();
	}
}

void DataGloveWin::event ( GloveEvent e )
{
	switch ( e )
	{
	case 	evSetPose1:
		_glove->setPose(0);

		break;
		case evSetPose2:
			_glove->setPose(1);
			for (int i=0;i<_glove->numSensors();i++)
			{
				data_min[i]->value(_glove->getPose(1)->at(i) -_glove->getPose(0)->at(i));
			}
			break;
		case 	evLoadGloveCfg:
			_glove->loadConfig(files->getFile(ui_glove_config->value()));
			for (int i=0;i<_glove->numSensors();i++)
			{
				data_min[i]->value(_glove->getMaxCalibration(i) -_glove->getMinCalibration(i));
			}
		break;
		case evSaveGloveCfg:
			_glove->saveConfig(files->getFile(ui_glove_config->value()));
		break;
		case evSetMax:
			_glove->getSensor()->setCalibRequest(DataGlove::Sensor::MAX);
		break;
		case evSetMin:
			_glove->getSensor()->setCalibRequest(DataGlove::Sensor::MIN);
			break;

		case evSetMaxThumb:
			_glove->getSensor()->setCalibRequest(DataGlove::Sensor::MAX_THUMB);
			break;
		case evQuit:
			gs_exit();
			break;
		default:
			phout<<"Serializable editor doesn't know what to do with that event\n";
			break;
	}
 }
DataGloveWin::~DataGloveWin ()
 {
	

 }
void DataGloveWin::update()
{
	for (unsigned int i=0;i<_glove->getSensor()->getNumSensors();i++)
	{
		float v;
		_glove->getSensor()->getAngleSensorData(i,v);
		data[i]->value(v);
	}
	if(_glove->numPoses()==2)
	{
		ui_interp_output->value(_glove->getPoseInterp());
	}
}
void DataGloveWin::show ()
 {
	 ui_glove_config->value("glove\\glove.cfg");
	ui_window->show();
	
 }

bool DataGloveWin::gloveActive()
{
	return ui_glove_active->value();
}

void DataGloveWin::setData( std::vector<float>* nout )
{
	for (int i=0;i<nout->size() && i < 18 ;i++)
	{
		data[i]->value( nout->at(i) );
	}
}


