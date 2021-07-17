/*=======================================================================
   Copyright 2007 Marcelo Kallmann. All Rights Reserved.
   This software is distributed for noncommercial use only, without
   any warranties, and provided that all copies contain the full copyright
   notice licence.txt located at the base folder of the distribution. 
  =======================================================================*/

/** \file dv_interdglove.cpp 
 * --------------------- */


#include "data_glove.h"

#include <sstream>
#include <assert.h>
#include <fstream>


DataGlove::Sensor::Sensor(int numsensors) : 
    _glove_isrhand(true),
    _num_sensors(numsensors), 
    _gesture(0),
    _cal_req(NONE)
{ 
    assert(numsensors > 1);
    _rawdata = new unsigned short[numsensors];
    _normdata = new float[numsensors];
    _angledata = new float[numsensors];
}

DataGlove::Sensor::Sensor(const Sensor& rhs):
    _glove_isrhand(rhs._glove_isrhand),
    _num_sensors(rhs._num_sensors), 
    _gesture(rhs._gesture),
    _cal_req(rhs._cal_req)
{
    assert(_num_sensors > 1);
    _rawdata = new unsigned short[_num_sensors];
    _normdata = new float[_num_sensors];
    _angledata = new float[_num_sensors];

    memcpy(_rawdata, rhs._rawdata, sizeof(unsigned short) * _num_sensors);
    memcpy(_normdata, rhs._normdata, sizeof(float) * _num_sensors);
    memcpy(_angledata, rhs._angledata, sizeof(float) * _num_sensors);
}

DataGlove::Sensor::~Sensor()
{
    if(_rawdata)
        delete[] _rawdata;
    if(_normdata)
        delete[] _normdata;
    if(_angledata)
        delete[] _angledata;

    _angle_ranges.clear();
}

bool DataGlove::Sensor::setAngleRangeValues(const std::vector<AngleRange>& ranges)
{
    if(ranges.size() != (unsigned int) _num_sensors)
        return false;
    
    _angle_ranges.clear();
    for(unsigned int i = 0; i < ranges.size(); i++)
        _angle_ranges.push_back(ranges.at(i));

    return true;
}

float DataGlove::Sensor::computeAngle(unsigned int i)
{
    return (_angle_ranges.at(i)._range.y - _angle_ranges.at(i)._range.x) * _normdata[i] + _angle_ranges.at(i)._range.x;
}

bool DataGlove::Sensor::setSensorData(unsigned short* values, float* nvalues, int gesture, int sensorsno)
{
    if(sensorsno != (int) _num_sensors)
        return false;

    for(int i = 0; i < sensorsno; i++) {
        _rawdata[i] = values[i];
        _normdata[i] = nvalues[i];
        _angledata[i] = computeAngle(i);
    }

    _gesture = gesture;

    return true;
}

std::string DataGlove::Sensor::PrintSensorType(unsigned int id)
{
    std::string stype;
    switch(id) {
        case FD_THUMBNEAR:
            stype = "FD_THUMBNEAR";
            break;
	    case FD_THUMBFAR:
            stype = "FD_THUMBFAR";
            break;
	    case FD_THUMBINDEX:
            stype = "FD_THUMBINDEX";
            break;
	    case FD_INDEXNEAR:
            stype = "FD_INDEXNEAR";
            break;
	    case FD_INDEXFAR:
            stype = "FD_INDEXFAR";
            break;
	    case FD_INDEXMIDDLE:
            stype = "FD_INDEXMIDDLE";
            break;
	    case FD_MIDDLENEAR:
            stype = "FD_MIDDLENEAR";
            break;
	    case FD_MIDDLEFAR:
            stype = "FD_MIDDLEFAR";
            break;
	    case FD_MIDDLERING:
            stype = "FD_MIDDLERING";
            break;
	    case FD_RINGNEAR:
            stype = "FD_RINGNEAR";
            break;
	    case FD_RINGFAR:
            stype = "FD_RINGFAR";
            break;
	    case FD_RINGLITTLE:
            stype = "FD_RINGLITTLE";
            break;
	    case FD_LITTLENEAR:
            stype = "FD_LITTLENEAR";
            break;
	    case FD_LITTLEFAR:
            stype = "FD_LITTLEFAR";
            break;
	    case FD_THUMBPALM:
            stype = "FD_THUMBPALM";
            break;
	    case FD_WRISTBEND:
            stype = "FD_WRISTBEND";
            break;
	    case FD_PITCH:
            stype = "FD_PITCH";
            break;
	    case FD_ROLL:
            stype = "FD_ROLL";
            break;
        default:
            stype = "FD_NOTRECOG";
            break;
    }
    return stype;
}

bool DataGlove::Sensor::printSensorData(std::string& ret, bool raw)
{
    if(_num_sensors < 0)
        return false;

    std::ostringstream strs;
    for(unsigned int i = 0; i < (unsigned int) _num_sensors; i++) {
        strs << DataGlove::Sensor::PrintSensorType(i) << ": ";
        if(raw)
            strs << _rawdata[i];
        else
            strs << _normdata[i];
        strs << "\n";
    }
    strs << "Gesture: " << _gesture;
    ret = strs.str();
    return true;
}

bool DataGlove::Sensor::printAngleData(std::string& ret)
{
    if(_num_sensors < 0)
        return false;

    std::ostringstream strs;
    for(unsigned int i = 0; i < (unsigned int) _num_sensors; i++) {
        strs << DataGlove::Sensor::PrintSensorType(i) << 
            " Rmin: " << _angle_ranges.at(i)._range.x << 
            " Rmax: " << _angle_ranges.at(i)._range.y << 
            " A: ";
        
        switch(_angle_ranges.at(i)._dof){
            case X:
                strs << "X";
                break;
            case Y:
                strs << "Y";
                break;
            default:
                strs << "Z";
                break;
        }
            
        strs << " -> "<< _angledata[i] << "\n";
    }
    strs << "Gesture: " << _gesture;
    ret = strs.str();
    return true;
}

bool DataGlove::Sensor::getRawSensorData(unsigned int index, unsigned short& value) const 
{
    if((int) index > _num_sensors)
        return false;

    value = _rawdata[index];
    return true;
}

bool DataGlove::Sensor::getNormSensorData(unsigned int index, float& value) const 
{
    if((int) index > _num_sensors)
        return false;

    value = _normdata[index];
    return true;
}

bool DataGlove::Sensor::getAngleSensorData(unsigned int index, float& value) const 
{
    if((int) index > _num_sensors)
        return false;

    value = _angledata[index];
    return true;
}

bool DataGlove::Sensor::getOrientation(unsigned int index, GsQuat& rot) const 
{
    if((int) index > _num_sensors)
        return false;

    GsVec axisrot(0.0, 1.0, 0.0);
    if(index < _angle_ranges.size())
        switch(_angle_ranges.at(index)._dof){
            case X:
                axisrot.set(1.0, 0.0, 0.0);
                break;
            case Y:
                axisrot.set(0.0, 1.0, 0.0);
                break;
            default:
                axisrot.set(0.0, 0.0, 1.0);
                break;
        }
    rot =  GsQuat(axisrot, GS_TORAD(_angledata[index]));
    return true;
}

void DataGlove::Sensor::recalibrate( bool param1 )
{
	_need_recalibration = param1;
}

float DataGlove::Sensor::getRangeValue()
{
	float val = 0;
	for(int i = 0; i < _num_sensors; i++) 
	{
		 val+= _normdata[i];
	}
	val/=_num_sensors;
	return val;
}


DataGlove::DataGlove( unsigned int usb_port_id) :
    _glove_interface(NULL),
    _glove_type(FD_GLOVENONE),
    _glove_hand(FD_HAND_RIGHT),
    _glove_num_sensors(0),
    _data_raw(NULL),
    _data_norm(NULL),
    _min_calibration(NULL),
    _max_calibration(NULL),
    _usb_port_id(usb_port_id),
	_connected(false),
	_sensor(NULL)
{
    //setDeviceReadLimit(true, INTERDGLOVE_READ_MS);
}

DataGlove::~DataGlove()
{
    if(this->isConnected())
		this->closeConnection();
    _glove_interface = NULL;
}

bool DataGlove::openConnection()
{
    if(this->isConnected()) {
		gsout << " Data Glove module already connected..." << gsnl;
		return false;
	}

	

    std::string portname = "USB";
    std::ostringstream strs;
    strs << _usb_port_id;
    portname += strs.str();
    gsout << "openConnection - Connecting..."<<portname.c_str();
    _glove_interface = fdOpen((char*) portname.c_str());
    if(!_glove_interface) {
        gsout << "    - No Data Glove found! Check teh USB connection." << gsnl;
        return false;
    }

    gsout << "    - Data Glove connected!" << gsnl;

    _glove_type = fdGetGloveType(_glove_interface);
    _glove_hand = fdGetGloveHand(_glove_interface);
    _glove_num_sensors = fdGetNumSensors(_glove_interface);

    _min_calibration = new unsigned short[_glove_num_sensors];
    _max_calibration = new unsigned short[_glove_num_sensors];

    _data_raw = new unsigned short[_glove_num_sensors];
    _data_norm = new float[_glove_num_sensors];

    resetCalibration();

    if(_sensor == 0) { // First time creation
		_sensor = new Sensor(_glove_num_sensors);
        _sensor->setDeviceProperties(printGloveTypeFormatted(), (_glove_hand == FD_HAND_RIGHT ? true : false));
	}

	Sensor::AngleRange def;
	def._dof = Sensor::Y;
	def._range.set(0.0, 90.0);
	std::vector<Sensor::AngleRange> ranges;
	for(unsigned int i = 0; i < INTERDGLOVE_SENSORS_NUMBER; i++)
		ranges.push_back(def);

	_sensor->setAngleRangeValues(ranges);

   //	this->throwBeep(DV_INTERDGLOVE_CONN_ENSTABLISHED_BEEP);
    this->setIsConnected(true);

    gsout << printGloveTypeFormatted().c_str() << gsnl;
    gsout << "    - Data Glove on port " << portname.c_str() << " connected!" << gsnl;

    return true;
}

void DataGlove::closeConnection()
{
    if(this->isConnected()) {
		fdClose( _glove_interface );

        _glove_interface = NULL;
        _glove_type = FD_GLOVENONE;
        _glove_hand = FD_HAND_RIGHT;
        _glove_num_sensors = 0;

        if(_min_calibration)
            delete[] _min_calibration;
        _min_calibration = NULL;

        if(_max_calibration)
            delete[] _max_calibration;
        _max_calibration = NULL;

        if(_data_raw)
            delete[] _data_raw;
        _data_raw = NULL;

        if(_data_norm)
            delete[] _data_norm;
        _data_norm = NULL;

       delete _sensor;
    	
	    this->setIsConnected(false);
	   // this->throwBeep(DV_INTERDGLOVE_CONN_ENSTABLISHED_BEEP);
    	
	    gsout << "DvInterDGlove::closeConnection - Disconnected." << gsnl;
    }
}

bool DataGlove::readDeviceData()
{
   // if(!checkDeviceReadAvailability()) // wait till next cycle
  //      return true;

    if(!_glove_interface) {
		gsout << "    - Data glove connection lost! The device must be restarted" << gsnl;
		return false; 
	}

  

    
    fdGetSensorRawAll(_glove_interface, _data_raw);
    int gesture = fdGetGesture(_glove_interface);

    if(_sensor->needToRecalibrate()) {
        calibrate(_sensor->getCalibrationRequest(), _data_raw);
        _sensor->recalibrate(false);
    }

    normalize(_data_raw, _data_norm);
    _sensor->setSensorData(_data_raw, _data_norm, gesture, _glove_num_sensors);

    return true;
}

void DataGlove::normalize(unsigned short* indata, float* outdata)
{
    unsigned short val;
    for(int i = 0; i < _glove_num_sensors; i++) {
        val = indata[i];
        if(val < _min_calibration[i])
            val = _min_calibration[i];
        if(val > _max_calibration[i])
            val = _max_calibration[i];

        if(_max_calibration[i] > _min_calibration[i]) {
            float div = float(_max_calibration[i]) - float(_min_calibration[i]);
            val -= _min_calibration[i];
            outdata[i] = float(val) / div;
        } else 
            outdata[i] = 1.0f; // Out of range
    }
}

void DataGlove::resetCalibration()
{
    for(int i = 0; i < _glove_num_sensors; i++){
        _min_calibration[i] = 0;
        _max_calibration[i] = 4095;
    }

	


}

bool DataGlove::calibrate(Sensor::CalibRequest req, unsigned short* data)
{
    switch(req) {
        case Sensor::MIN :
            for(int i = 0; i < _glove_num_sensors; i++)
                _min_calibration[i] = data[i];
            break;
        case Sensor::MAX :
            for(int i = 0; i < _glove_num_sensors; i++)
                _max_calibration[i] = data[i];
            break;
        case Sensor::MAX_THUMB :
            if (_glove_type==FD_GLOVE7 || _glove_type==FD_GLOVE7W)
                _max_calibration[FD_THUMBNEAR] = data[FD_THUMBNEAR];
            else 
			{
                _max_calibration[FD_THUMBNEAR] = data[FD_THUMBNEAR];
                _max_calibration[FD_THUMBFAR] = data[FD_THUMBFAR];
                _max_calibration[FD_THUMBINDEX] = data[FD_THUMBINDEX];
                _max_calibration[FD_THUMBPALM] = data[FD_THUMBPALM];
            }
            break;
        default:
            return false;
            break;
    };

    /*
    gsout << "MIN: ";
    for(int i = 0; i < _glove_num_sensors; i++)
        gsout << _min_calibration[i] << " ";
    gsout << gsnl;

    gsout << "MAX: ";
    for(int i = 0; i < _glove_num_sensors; i++)
        gsout << _max_calibration[i] << " ";
    gsout << gsnl << gsnl;
    */

    return true;
}

std::string DataGlove::printGloveTypeFormatted()
{
    std::string ret("  - Type: ");

    switch(_glove_type) {
	    case FD_GLOVE7:
            ret += "Glove7"; 
            break;
	    case FD_GLOVE7W:
            ret += "Glove7W";
            break;
	    case FD_GLOVE16:
            ret += "Glove16";
            break;
	    case FD_GLOVE16W:
            ret += "Glove16W";
            break;
	    case FD_GLOVE5U:
            ret += "DG5 Ultra serial";
            break;
	    case FD_GLOVE5UW:
            ret += "DG5 Ultra serial, wireless";
            break;
	    case FD_GLOVE5U_USB:
            ret += "DG5 Ultra USB"; 
            break;
	    case FD_GLOVE14U: 
            ret += "DG14 Ultra serial"; 
            break;
	    case FD_GLOVE14UW: 
            ret += "DG14 Ultra serial, wireless"; 
            break;
	    case FD_GLOVE14U_USB: 
            ret += "DG14 Ultra USB"; 
            break;
        default:
            return ret + "None";
	}
    
    ret += " Handness: ";
    ret += (_glove_hand == FD_HAND_RIGHT ? "Right" : "Left");
    ret += " Sensors: ";

    std::ostringstream strs;
    strs << _glove_num_sensors;
    ret += strs.str();

    return ret;
}

void DataGlove::ScanUSBPort(std::vector<std::string>& names)
{
    unsigned short aPID[INTERDGLOVE_MAXIMUM_GLOVES_CONNECTIONS];
	int nNumFound = INTERDGLOVE_MAXIMUM_GLOVES_CONNECTIONS;
    gsout << "Searching Data Gloves from USB ..." << gsnl;
	fdScanUSB(aPID,nNumFound);
    gsout << "  - Devices found: " << nNumFound << gsnl;
	for(int c = 0; c < nNumFound; c++) {
        std::ostringstream strs;
        strs << c;
        std::string devstr = strs.str() + " - ";
        switch(aPID[c]) {
		    case DG14U_R:
			    devstr += "Data Glove 14 Ultra Right";
			    break;
		    case DG14U_L:
			    devstr += "Data Glove 14 Ultra Left";
			    break;
		    case DG5U_R:
			    devstr += "Data Glove 5 Ultra Right";
			    break;
		    case DG5U_L:
			    devstr += "Data Glove 5 Ultra Left";
			    break;
		    default:
			    devstr += "Unknown";
		}
        names.push_back(devstr);
	}
}

void DataGlove::printData()
{
	std::string s;
	_sensor->printSensorData(s);
	gsout<<s.c_str()<<gsnl;
}

bool DataGlove::loadConfig( GsString filename )
{
	FILE* file = fopen(filename,"r");
	if(!file)
	{
		return false;
	}
	int num_sensors;
	
	fscanf(file,"%d",&num_sensors);

	if(num_sensors != _sensor->getNumSensors())
	{
		gsout<<"sensor count doesnt match\n";
		return false;
	}

	for (int i = 0; i <  _sensor->getNumSensors(); i++)
	{
		fscanf(file,"%d",&_sensor->getRange(i)->_dof);
		fscanf(file,"%g",&_sensor->getRange(i)->_range.x);
		fscanf(file,"%g",&_sensor->getRange(i)->_range.y);
		fscanf(file,"%d",&_min_calibration[i]);
		fscanf(file,"%d",&_max_calibration[i]);

	}

	gsout<<"done loading "<<filename<<gsnl;
	return true;
}

void DataGlove::saveConfig( GsString file )
{

	std::ofstream output(file);
	output<< _sensor->getNumSensors()<<"\n";

	for (int i = 0; i <  _sensor->getNumSensors(); i++)
	{
		 output<<  _sensor->getRange(i)->_dof<<" "<<_sensor->getRange(i)->_range.x <<" "<<_sensor->getRange(i)->_range.y<<" "<< _min_calibration[i]<<" "<< _max_calibration[i]<<"\n";

	}

	
	output.close();
	gsout<<"saved glove cfg to "<<file<<gsnl;
}

void DataGlove::setPose( int num )
{
	if(poses.size()<=num)
	{
		poses.resize(num+1);
	}
	poses[num].resize(_glove_num_sensors);
	for (int i=0;i<_glove_num_sensors;i++)
	{
		_sensor->getRawSensorData(i,poses[num][i]);

	}
}

float DataGlove::getPoseInterp()
{
	float val = 0;
	float total =0;
	if(poses.size()==2)
	{
		for (int i=0;i<_glove_num_sensors;i++)
		{
			unsigned short p1 = poses[0][i];
			unsigned short p2 = poses[1][i];
			if(abs(p2-p1) > 100)
			{ //only use sensors with large delta
			unsigned short p;
			_sensor->getRawSensorData(i,p);
			val +=  ((float)(p - p1)) ;
			total += (p2-p1);
			}
		}
	}
	else
	{
		gsout<<"need to poses to interpolate\n";
	}
	return abs(val/total);
}



