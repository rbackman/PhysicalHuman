

/*******************************
 *
 * RakNet TEST
 * 
 *******************************/

#include "util_server.h"

// #include <string>
// #include <sstream>
// #include <iostream>

#include <raknet/Kbhit.h>
#include <raknet/RakPeerInterface.h>
#include <raknet/NetworkIDManager.h>
#include <raknet/RakNetTypes.h>
#include <raknet/RakSleep.h>
#include <raknet/MessageIdentifiers.h>
#include <raknet/BitStream.h>
#include <raknet/RakNetTypes.h>
#include "common.h"
using namespace RakNet;

RakNet::RakPeerInterface* g_PeerInterface = NULL;


// Defining application RakTest Message IDs
enum RakTestMessageID 
{
    ID_MSG_INTEGER = ID_USER_PACKET_ENUM,
    ID_MSG_FLOAT,
    ID_MSG_STRING,
    ID_MSG_MARKER,
	ID_MSG_QUIT,
    ID_MSG_USER_DEFINED
};

int receiveint(RakNet::Packet* packet)
{
	RakNet::BitStream in(packet->data, packet->length, true);
	unsigned char msgid;
	in.Read(msgid); // Removing the Message ID
	int noreceived = 0;
	in.ReadAlignedBytes((unsigned char*) &noreceived, sizeof(noreceived));
	phout << "   --- the integer received is: " << noreceived << gsnl;
	return noreceived;
}
float receivefloat(RakNet::Packet* packet)
{
	RakNet::BitStream in(packet->data, packet->length, true);
	unsigned char msgid;
	in.Read(msgid); // Removing the Message ID
	float noreceived = 0;
	in.ReadAlignedBytes((unsigned char*) &noreceived, sizeof(noreceived));
	phout << "   --- the float received is: " << noreceived << gsnl;
	return noreceived;
}
GsString receivestring(RakNet::Packet* packet)
{
	RakNet::BitStream in(packet->data, packet->length, true);
	unsigned char msgid;
	in.Read(msgid); // Removing the Message ID
	int strsize = 0;
	in.ReadAlignedBytes((unsigned char*) &strsize, sizeof(strsize));

	phout << "   --- the string size is: " << strsize << gsnl;
	if(strsize > 0) {
		char* strbuff = new char[strsize + 1];
		in.ReadAlignedBytes((unsigned char*) strbuff, strsize);
		strbuff[strsize] = '\0';
		GsString constrname(strbuff);
		delete[] strbuff;
		phout << "   --- the string received is: " << constrname << gsnl;
		return constrname;
	}
	return "empty";
}
marker_position receivemarker( RakNet::Packet* packet )
{

	RakNet::BitStream in(packet->data, packet->length, true);
	unsigned char msgid;
	in.Read(msgid); // Removing the Message ID
	int markersize =  sizeof(marker_position);
	char* strbuff = new char[markersize];
	in.ReadAlignedBytes((unsigned char*) strbuff,markersize);
	marker_position* p = (marker_position*)strbuff;
	phout<<"received marker "<<p->index<<" with position " << p->pos<<gsnl;
	return *p;
}

void Server::sendclose()
{
    RakNet::BitStream out;
    out.Write((RakNet::MessageID) ID_MSG_QUIT);
    g_PeerInterface->Send(&out, MEDIUM_PRIORITY, RELIABLE_SEQUENCED, 0, RakNet::UNASSIGNED_SYSTEM_ADDRESS, true);
}
void Server::sendint(int no)
{

	phout << "   --- the integer sent is: " << no << gsnl;
    RakNet::BitStream out;
    out.Write((RakNet::MessageID) ID_MSG_INTEGER);
    out.WriteAlignedBytes((const unsigned char*) &no, sizeof(no));
    // Sending the message in Broadcast to all the clients connected
    g_PeerInterface->Send(&out, MEDIUM_PRIORITY, RELIABLE_SEQUENCED, 0, RakNet::UNASSIGNED_SYSTEM_ADDRESS, true);
}
void Server::sendfloat(float no)
{

    phout << "   --- the float sent is: " << no << gsnl;
    RakNet::BitStream out;
    out.Write((RakNet::MessageID) ID_MSG_FLOAT);
    out.WriteAlignedBytes((const unsigned char*) &no, sizeof(no));
    // Sending the message in Broadcast to all the clients connected
    g_PeerInterface->Send(&out, MEDIUM_PRIORITY, RELIABLE_SEQUENCED, 0, RakNet::UNASSIGNED_SYSTEM_ADDRESS, true);
}
void Server::sendmarker( marker_position marker )
{
	RakNet::BitStream out;
	out.Write((RakNet::MessageID) ID_MSG_MARKER);
	// Sending the message in Broadcast to all the clients connected
	out.WriteAlignedBytes((const unsigned char*) &marker, sizeof(marker));
	g_PeerInterface->Send(&out, MEDIUM_PRIORITY, RELIABLE_SEQUENCED, 0, RakNet::UNASSIGNED_SYSTEM_ADDRESS, true);
}
void Server::sendstring(GsString str)
{
    phout << "   --- the string sent is: " << str << gsnl;
    unsigned int strsize = str.len();
    RakNet::BitStream out;
    out.Write((RakNet::MessageID) ID_MSG_STRING);
    out.WriteAlignedBytes((const unsigned char*) &strsize, sizeof(strsize));
    // Sending the message in Broadcast to all the clients connected
    out.WriteAlignedBytes((const unsigned char*) str.pt(), sizeof(unsigned char*) * strsize);
    g_PeerInterface->Send(&out, MEDIUM_PRIORITY, RELIABLE_SEQUENCED, 0, RakNet::UNASSIGNED_SYSTEM_ADDRESS, true);
}
int Server::startServer(unsigned short port)
{
	g_PeerInterface = RakNet::RakPeerInterface::GetInstance();

    // Initializing the socket descriptor
    RakNet::SocketDescriptor sd;
   
    phout << "   - Initializing server connection" << gsnl;
    sd.port = port;
   

    // Startup the Peer Interface
    switch(g_PeerInterface->Startup(32, &sd, 1)) 
	{
    case RakNet::RAKNET_STARTED:
        phout << "   - Initialization process done!" << gsnl;
        break;
    case RakNet::RAKNET_ALREADY_STARTED:
        phout << "The server instance has been already started"<< gsnl;
        return -1;
    case RakNet::INVALID_SOCKET_DESCRIPTORS:
        phout << "The socket descriptor is invalid"<< gsnl;
        return -1;
    case RakNet::INVALID_MAX_CONNECTIONS:
        phout << "Maximum connections value is not valid"<< gsnl;
        return -1;
    case RakNet::SOCKET_PORT_ALREADY_IN_USE:
        phout << "The port specified is already in use"<< gsnl;
        return -1;
    case RakNet::PORT_CANNOT_BE_ZERO:
        phout << "The port specified cannot be zero"<< gsnl;
        return -1;
    case RakNet::SOCKET_FAMILY_NOT_SUPPORTED:
    case RakNet::SOCKET_FAILED_TO_BIND:
    case RakNet::SOCKET_FAILED_TEST_SEND:
    case RakNet::FAILED_TO_CREATE_NETWORK_THREAD:
    default:
        phout << "Startup internal error." << gsnl;
        return -1;
    }

	 g_PeerInterface->SetMaximumIncomingConnections(12);
return -1;
}

bool Server::update()
{
	RakNet::Packet *packet = NULL;
	packet = g_PeerInterface->Receive();
	while( packet != NULL ) 
	{
		switch (packet->data[0]) 
		{

			
		case ID_CONNECTION_REQUEST_ACCEPTED:
			phout << " - Net message: ID_CONNECTION_REQUEST_ACCEPTED" << gsnl;
			phout << "   - Connection accepted from server: " << packet->systemAddress.ToString() << gsnl;
			break;
		case ID_ALREADY_CONNECTED:
			phout << " - Net message: ID_ALREADY_CONNECTED" << gsnl;
			phout << "   - The client seems to be already connected" << gsnl;
			break;

			// SERVER MESSAGES
		case ID_NEW_INCOMING_CONNECTION:
			phout << " - Net message: ID_NEW_INCOMING_CONNECTION" << gsnl;
			phout << "   - Connection accepted from client: "  << packet->systemAddress.ToString() << gsnl;
			break;

			// CLIENT/SERVER MESSAGES
		case ID_CONNECTION_LOST:
			phout << " - Net message: ID_CONNECTION_LOST" << gsnl;
			phout << " - Connection lost from peer: " << packet->systemAddress.ToString() << gsnl;
			break;
		case ID_DISCONNECTION_NOTIFICATION:
			phout << " - Net message: ID_DISCONNECTION_NOTIFICATION" << gsnl;
			phout << " - Disconnect notification from peer: " << packet->systemAddress.ToString() << gsnl;
			break;

			// TEST MESSAGES
		case ID_MSG_INTEGER:
			phout << " - Net message: ID_MSG_INTEGER: " << receiveint(packet)<< gsnl;

			break;
		case ID_MSG_FLOAT:
			phout << " - Net message: ID_MSG_FLOAT: " <<  receivefloat(packet)<<gsnl;

			break;
		case ID_MSG_STRING:
			phout << " - Net message: ID_MSG_STRING" << receivestring(packet)<< gsnl;
			;
			break;
		
		case ID_MSG_QUIT:
			phout << " - Net message: ID_MSG_STRING" << gsnl;
			//exit = true; // I have got a Quit message from the server... I am closing
			break;
		}

		g_PeerInterface->DeallocatePacket(packet);
		packet = g_PeerInterface->Receive();
	}

	return false;
}




bool Client::connectToServer(GsString hostname, unsigned short port)
{
	
	
	_hostname = hostname;
	_port = port;
	phout << "Connecting to server: " << hostname << gsnl;


	RakNet::ConnectionAttemptResult res = g_PeerInterface->Connect( hostname , port, 0, 0);

	switch(res) {
	case CANNOT_RESOLVE_DOMAIN_NAME:
		phout << "NetClient - The client can't resolve the domain name" << gsnl;
		return false;
	case ALREADY_CONNECTED_TO_ENDPOINT:
		phout << "NetClient - The client is already connected to the endpoint" << gsnl;
		return false;
	case CONNECTION_ATTEMPT_ALREADY_IN_PROGRESS:
		phout << "NetClient - Connection attempt already in progress" << gsnl;
		return false;
	case INVALID_PARAMETER:
	case SECURITY_INITIALIZATION_FAILED:
		phout << "NetClient - Internal Error. Start process failed" << gsnl;
		return false;
	default:
		return true;
		break;
	}


 return false;
}


bool Client::update(marker_position* marker)
{
  
        RakNet::Packet *packet = NULL;
        packet = g_PeerInterface->Receive();
        while( packet != NULL ) 
		{
            switch (packet->data[0]) 
			{

            // CLIENT NETWORK MESSAGES
            case ID_CONNECTION_ATTEMPT_FAILED:
                phout << " - Net message: ID_CONNECTION_ATTEMPT_FAILED" << gsnl;
                phout << "   - Trying to reconnect" << gsnl;
                connectToServer(_hostname, _port);
                break;
            case ID_NO_FREE_INCOMING_CONNECTIONS:
                phout << " - Net message: ID_NO_FREE_INCOMING_CONNECTIONS" << gsnl;
                phout << "   - Trying to reconnect" << gsnl;
                 connectToServer(_hostname, _port);
                break;
            case ID_CONNECTION_REQUEST_ACCEPTED:
                phout << " - Net message: ID_CONNECTION_REQUEST_ACCEPTED" << gsnl;
                phout << "   - Connection accepted from server: " << packet->systemAddress.ToString() << gsnl;
                break;
            case ID_ALREADY_CONNECTED:
                phout << " - Net message: ID_ALREADY_CONNECTED" << gsnl;
                phout << "   - The client seems to be already connected" << gsnl;
                break;

            // SERVER MESSAGES
            case ID_NEW_INCOMING_CONNECTION:
                phout << " - Net message: ID_NEW_INCOMING_CONNECTION" << gsnl;
                phout << "   - Connection accepted from client: "  << packet->systemAddress.ToString() << gsnl;
                break;

            // CLIENT/SERVER MESSAGES
            case ID_CONNECTION_LOST:
                phout << " - Net message: ID_CONNECTION_LOST" << gsnl;
                phout << " - Connection lost from peer: " << packet->systemAddress.ToString() << gsnl;
                break;
            case ID_DISCONNECTION_NOTIFICATION:
                phout << " - Net message: ID_DISCONNECTION_NOTIFICATION" << gsnl;
                phout << " - Disconnect notification from peer: " << packet->systemAddress.ToString() << gsnl;
                break;

            // TEST MESSAGES
            case ID_MSG_INTEGER:
                phout << " - Net message: ID_MSG_INTEGER: " << receiveint(packet)<< gsnl;
                
                break;
            case ID_MSG_FLOAT:
                phout << " - Net message: ID_MSG_FLOAT: " <<  receivefloat(packet)<<gsnl;
               
                break;
            case ID_MSG_STRING:
                phout << " - Net message: ID_MSG_STRING" << receivestring(packet)<< gsnl;
                ;
                break;
			case ID_MSG_MARKER:
				*marker = receivemarker(packet);
				phout << " - Net message: ID_MSG_MARKER: " << marker->pos<< gsnl;
				return true;
				break;
            case ID_MSG_QUIT:
                phout << " - Net message: ID_MSG_STRING" << gsnl;
                //exit = true; // I have got a Quit message from the server... I am closing
                break;
            }
		
        g_PeerInterface->DeallocatePacket(packet);
        packet = g_PeerInterface->Receive();
    }

		return false;
}

bool Client::start()
{
	if(g_PeerInterface==NULL)
		g_PeerInterface = RakNet::RakPeerInterface::GetInstance();



	RakNet::SocketDescriptor socketDescriptor;
	socketDescriptor.port = 0;

	RakNet::StartupResult res = g_PeerInterface->Startup(1, &socketDescriptor, 1);

	switch(res) {
	case RakNet::RAKNET_ALREADY_STARTED:
		phout<<" NetClient - The client instance has been already started" << gsnl;
		return false;
	case RakNet::INVALID_SOCKET_DESCRIPTORS:
		phout << "NetClient - The socket descriptor is invalid" << gsnl;
		return false;
	case RakNet::INVALID_MAX_CONNECTIONS:
		phout << "NetClient - Maximum connection is invalid" << gsnl;
		return false;
	case RakNet::SOCKET_PORT_ALREADY_IN_USE:
		phout << "NetClient - The port specified is already in use" << gsnl;
		return false;
	case RakNet::PORT_CANNOT_BE_ZERO:
		phout << "NetClient - The port specified cannot be zero" << gsnl;
		return false;
	case RakNet::SOCKET_FAMILY_NOT_SUPPORTED:
	case RakNet::SOCKET_FAILED_TO_BIND:
	case RakNet::SOCKET_FAILED_TEST_SEND:
	case RakNet::FAILED_TO_CREATE_NETWORK_THREAD:
		phout << "NetClient - Internal Error. Start process failed" << gsnl;
		return false;
	default:
		break;
	}
	return false;
}



// 
// int main(int argc, char** argv)
// {
//     if(argc < 2) {
//         printInstructions(std::string(argv[0]));
//         return -1;
//     }
// 
//     std::string hostname("localhost");
//     unsigned short port = 12345;
//     std::string modestr = std::string(argv[1]);
//     bool isserver = false;
// 
//     if(modestr == "-c") {
//         isserver = false;
// 
//         if(argc > 4) {
//             phout << " *** too many arguments specified *** " << gsnl;
//             printInstructions(std::string(argv[0]));
//             return -1;
//         }
// 
//         if(argc == 4) {
//             std::stringstream nstream(argv[3]); // convert string to port number
//             if(!(nstream >> port)) {
//                 phout << " *** port number not specified correctly *** " << gsnl;
//                 return -1;
//             }
//         }
// 
//         hostname = argv[2];
//     } else if(modestr == "-s") {
//         isserver = true;
//     
//         if(argc > 3) {
//             phout << " *** too many arguments specified *** " << gsnl;
//             printInstructions(std::string(argv[0]));
//             return -1;
//         }
// 
//         if(argc == 3) {
//             std::stringstream nstream(argv[2]); // convert string to port number
//             if(!(nstream >> port)) {
//                 phout << " *** port number not specified correctly *** " << gsnl;
//                 return -1;
//             }
//         }
//     } else {
//         phout << " *** mode specified not correct: -c or -s are allowed *** " << gsnl;
//         printInstructions(std::string(argv[0]));
//         return -1;
//     }
// 
//     // Creating RakNet Peer interface
//     g_PeerInterface = RakNet::RakPeerInterface::GetInstance();
// 
//     // Initializing the socket descriptor
//     RakNet::SocketDescriptor sd;
//     if(isserver) {
//         phout << "   - Initializing server connection" << gsnl;
//         sd.port = port;
//     } else {
//         phout << "   - Initializing client connection" << gsnl;
//         sd.port = 0; // Client connection does not require port -> MUST BE 0
//     }
// 
//     // Startup the Peer Interface
//     switch(g_PeerInterface->Startup(isserver ? 32 : 1, &sd, 1)) {
//     case RakNet::RAKNET_STARTED:
//         phout << "   - Initialization process done!" << gsnl;
//         break;
//     case RakNet::RAKNET_ALREADY_STARTED:
//         phout << "The server instance has been already started"<< gsnl;
//         return -1;
//     case RakNet::INVALID_SOCKET_DESCRIPTORS:
//         phout << "The socket descriptor is invalid"<< gsnl;
//         return -1;
//     case RakNet::INVALID_MAX_CONNECTIONS:
//         phout << "Maximum connections value is not valid"<< gsnl;
//         return -1;
//     case RakNet::SOCKET_PORT_ALREADY_IN_USE:
//         phout << "The port specified is already in use"<< gsnl;
//         return -1;
//     case RakNet::PORT_CANNOT_BE_ZERO:
//         phout << "The port specified cannot be zero"<< gsnl;
//         return -1;
//     case RakNet::SOCKET_FAMILY_NOT_SUPPORTED:
//     case RakNet::SOCKET_FAILED_TO_BIND:
//     case RakNet::SOCKET_FAILED_TEST_SEND:
//     case RakNet::FAILED_TO_CREATE_NETWORK_THREAD:
//     default:
//         phout << "Startup internal error." << gsnl;
//         return -1;
//     }
// 
//     bool exit = false;
//     if(isserver) {
//         phout << "Waiting for clients to connect..." << gsnl;
//         phout << "" << gsnl;
//         phout << "Press: 'q' to exit" << gsnl;
//         phout << "       'i' to send an integer" << gsnl;
//         phout << "       'f' to send a float" << gsnl;
//         phout << "       's' to send a string" << gsnl;
// 
//         g_PeerInterface->SetMaximumIncomingConnections(12);
//     } else
//         connectToServer(hostname, port);
//     
//     char c = 0;
//     do {
//         RakNet::Packet *packet = NULL;
//         packet = g_PeerInterface->Receive();
//         while( packet != NULL ) {
//             switch (packet->data[0]) {
// 
//             // CLIENT NETWORK MESSAGES
//             case ID_CONNECTION_ATTEMPT_FAILED:
//                 phout << " - Net message: ID_CONNECTION_ATTEMPT_FAILED" << gsnl;
//                 phout << "   - Trying to reconnect" << gsnl;
//                 connectToServer(hostname, port);
//                 break;
//             case ID_NO_FREE_INCOMING_CONNECTIONS:
//                 phout << " - Net message: ID_NO_FREE_INCOMING_CONNECTIONS" << gsnl;
//                 phout << "   - Trying to reconnect" << gsnl;
//                 connectToServer(hostname, port);
//                 break;
//             case ID_CONNECTION_REQUEST_ACCEPTED:
//                 phout << " - Net message: ID_CONNECTION_REQUEST_ACCEPTED" << gsnl;
//                 phout << "   - Connection accepted from server: " << std::string(packet->systemAddress.ToString()) << gsnl;
//                 break;
//             case ID_ALREADY_CONNECTED:
//                 phout << " - Net message: ID_ALREADY_CONNECTED" << gsnl;
//                 phout << "   - The client seems to be already connected" << gsnl;
//                 break;
// 
//             // SERVER MESSAGES
//             case ID_NEW_INCOMING_CONNECTION:
//                 phout << " - Net message: ID_NEW_INCOMING_CONNECTION" << gsnl;
//                 phout << "   - Connection accepted from client: " + std::string(packet->systemAddress.ToString()) << gsnl;
//                 break;
// 
//             // CLIENT/SERVER MESSAGES
//             case ID_CONNECTION_LOST:
//                 phout << " - Net message: ID_CONNECTION_LOST" << gsnl;
//                 phout << " - Connection lost from peer: " + std::string(packet->systemAddress.ToString()) << gsnl;
//                 break;
//             case ID_DISCONNECTION_NOTIFICATION:
//                 phout << " - Net message: ID_DISCONNECTION_NOTIFICATION" << gsnl;
//                 phout << " - Disconnect notification from peer: " + std::string(packet->systemAddress.ToString()) << gsnl;
//                 break;
// 
//             // TEST MESSAGES
//             case ID_MSG_INTEGER:
//                 phout << " - Net message: ID_MSG_INTEGER" << gsnl;
//                 receiveint(packet);
//                 break;
//             case ID_MSG_FLOAT:
//                 phout << " - Net message: ID_MSG_FLOAT" << gsnl;
//                 receivefloat(packet);
//                 break;
//             case ID_MSG_STRING:
//                 phout << " - Net message: ID_MSG_STRING" << gsnl;
//                 receivestring(packet);
//                 break;
//             case ID_MSG_QUIT:
//                 phout << " - Net message: ID_MSG_STRING" << gsnl;
//                 exit = true; // I have got a Quit message from the server... I am closing
//                 break;
//             }
// 
//             g_PeerInterface->DeallocatePacket(packet);
//             packet = g_PeerInterface->Receive();
//         }
// 
//         if(isserver) { // waiting for a user input
//             if(kbhit()) {
//                 c = getch(); //std::cin.get();
//                 switch(c) {
//                     case 'q': sendclose(); exit = true; break;
//                     case 'f': sendfloat(); break;
//                     case 'i': sendint(); break;
//                     case 's': sendstring(); break;
//                     default: break;
//                 }
//             }
//         }
//         
//         RakSleep(30); // Don't hog the CPU
//     } while(!exit);
// 
// 
//     g_PeerInterface->Shutdown(150, 0, LOW_PRIORITY); //This method sends a DISCONNECT signal... it could be used directly to notify the server or client
//     RakNet::RakPeerInterface::DestroyInstance(g_PeerInterface);
//     g_PeerInterface = NULL;
// 
//     return 0;
// }






