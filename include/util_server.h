#pragma once

#include <gsim/gs_vec.h>
#include <gsim/gs_color.h>
#include <gsim/gs_string.h>

typedef struct 
{
	GsVec pos;
	GsColor col;
	int index;
}marker_position;


class Server
{
public:
	int startServer(unsigned short port = 54321);
	void sendclose();
	void sendint(int no);
	void sendfloat(float f);
	void sendstring(GsString str);
	void sendmarker(marker_position marker);
	bool update();
};

class Client
{
public:
	GsString _hostname;
	unsigned short _port;
	bool connectToServer(GsString hostname = "127.0.0.1", unsigned short port = 54321);
	bool update(marker_position* marker);
	bool start();
	Client(){start();}
};
