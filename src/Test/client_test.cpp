

#include <gsim/gs_output.h>
#include <gsim/dv_netclient.h>

int main ( int argc, char** argv )
{
	
	
	DvNetClient* _netClient = new DvNetClient;
	
	_netClient = new DvNetClient;
	_netClient->setServerPort(54321);

	
	std::string hname("localhost");
	_netClient->setServerHostname(hname);
	_netClient->startClient();
	

	while(true)
	{
		_netClient->handleNetMessages();

		if(_netClient->isServerStatusChanged())
			phout << "Status changed: object release and recreation" << gsnl;

		if(_netClient->isConnected())
		{
			std::vector<DvNetClient::Interface*> itnerflist = _netClient->getInterfaces();
		}
		
	}

	return 0;
}


