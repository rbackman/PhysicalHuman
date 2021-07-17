/*=======================================================================
   Copyright 2007 Marcelo Kallmann. All Rights Reserved.
   This software is distributed for noncommercial use only, without
   any warranties, and provided that all copies contain the full copyright
   notice licence.txt located at the base folder of the distribution. 
  =======================================================================*/

/** \file dv_interdglove.h 
 * --------------------- */

#ifndef _DV_INTERDGLOVE_H
#define	_DV_INTERDGLOVE_H


#include "util_serializable.h"

#include <vector>
#include <string>


#include <windows.h> // FGlove.h requires this to be complete
#include <dglove/fglove.h>

/******************************************************
 *
 *    class DvInterDGlove
 *
 */

#define INTERDGLOVE_READ_MS                                  16
#define INTERDGLOVE_MAXIMUM_GLOVES_CONNECTIONS               10
#define INTERDGLOVE_CONN_ENSTABLISHED_BEEP                   1000, 200
#define INTERDGLOVE_SENSORS_NUMBER                           18

class DataGlove 
{
public:
	// Data Glove data class
	class Sensor
	{
	public:

        enum CalibRequest { NONE = 0, MIN, MAX, MAX_THUMB };
		typedef enum glove_dof
		{
			X,
			Y,
			Z
		}glove_dof;

        struct AngleRange {
            GsPnt2 _range;
            glove_dof _dof;    
        };

        Sensor(int numsensors);
        Sensor(const Sensor& rhs);
        ~Sensor();
		inline bool needToRecalibrate() const { return _need_recalibration; }
        void setCalibRequest(CalibRequest request) { _cal_req = request; _need_recalibration=true;  }
        void setDeviceProperties(const std::string& type, bool right = true) { _glove_type = type; _glove_isrhand = right; }
        bool setAngleRangeValues(const std::vector<AngleRange>& ranges);
        bool setSensorData(unsigned short* values, float* nvalues, int gesture, int sensorsno);

        inline bool isRightHandness() const { return _glove_isrhand; }
        const std::string& getDeviceType() const { return _glove_type; }
        unsigned int getNumSensors() const { return _num_sensors; }
        int getGesture() const { return _gesture; }
        bool getRawSensorData(unsigned int index, unsigned short& value) const;
        bool getNormSensorData(unsigned int index, float& value) const;
        bool getAngleSensorData(unsigned int index, float& value) const;
        bool getOrientation(unsigned int index, GsQuat& rot) const;
		float getRangeValue();
        CalibRequest getCalibrationRequest() { CalibRequest r = _cal_req; _cal_req = NONE; return r;  }

        bool printSensorData(std::string& ret, bool raw = true);
        bool printAngleData(std::string& ret);
        static std::string PrintSensorType(unsigned int id);
		void recalibrate( bool param1 );
		AngleRange* getRange(int i)
		{
			return &_angle_ranges[i];
		}
	protected:
       

        float computeAngle(unsigned int i);
		
	public:

    protected:
        std::string _glove_type;
        bool _glove_isrhand;
		bool _need_recalibration;
        int _num_sensors;
        int _gesture;

        unsigned short* _rawdata;
        float* _normdata;
        float* _angledata;

        std::vector<AngleRange> _angle_ranges;

        CalibRequest _cal_req;
	};


	unsigned short getMinCalibration(int i){return _min_calibration[i];}
	unsigned short getMaxCalibration(int i){return _max_calibration[i];}

	// Creates a Data Glove interface
	DataGlove(unsigned int usb_port_id);
	// Destructor
	~DataGlove();
	bool loadConfig( GsString file );
	void saveConfig( GsString file );

	float* getData(){return _data_norm;}
    unsigned int getUSBPortID() const { return _usb_port_id; }
	int numSensors(){return _glove_num_sensors;}

	// Opens a connection
	bool openConnection(); 
	// Closes a connection
	void closeConnection();
	bool isConnected()
	{
		return _connected;
	}
		void printData();
	// Reads values for the specified object updating the InterfaceData
	bool readDeviceData();

    static void ScanUSBPort(std::vector<std::string>& names);
		Sensor* getSensor()
		{
		return _sensor;
		}
		void setPose( int num );
		float getPoseInterp();
		int numPoses(){return poses.size();}
protected:
	// Recalibrate DGloveData
	bool calibrate(Sensor::CalibRequest req, unsigned short* data);
    // Reset calibration to default values
    void resetCalibration();
    // Normalizing acoording with the calibration ranges
	void normalize(unsigned short* indata, float* outdata);
    // Return the glove string formatted
    std::string printGloveTypeFormatted();
	void setIsConnected(bool b)
	{
		_connected = b;
	}
	
protected:
	std::vector<std::vector<unsigned short>> poses;
	Sensor* _sensor;
    fdGlove* _glove_interface;
    int _glove_type;
    int _glove_hand;
    int _glove_num_sensors;
	bool _connected;
    unsigned short* _min_calibration;
    unsigned short* _max_calibration;

    unsigned short* _data_raw;
    float* _data_norm;

    unsigned int _usb_port_id;

	public:

		std::vector<unsigned short>* getPose(int i) {return &poses[i];}
};


#endif //_DV_INTERDGLOVE_H

