/** \file laserServer.cpp
 *  \ingroup hwmodule
 *  \brief Laserscanner server for sharing laserscanner data from stage
 *
 *  This plugin provides the laserscanner interface to share laserscanner
 * data from the stage server
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1762 $
 *  $Date: 2011-12-07 12:08:40 +0100 (Wed, 07 Dec 2011) $
 *
 */
/***************************************************************************
 *              Copyright 2010 Anders Billesø Beck, DTU                    *
 *                         abb@elektro.dtu.dk                              *
 *                                                                         *
 ***************************************************************************/
/************************** Library version  ***************************/
 #define LASERSERVER_VERSION  	"1.0"
/************************** Version control information ***************************/
 #define REVISION         "$Rev: 1762 $:"
 #define DATE             "$Date: 2011-12-07 12:08:40 +0100 (Wed, 07 Dec 2011) $:"
/**********************************************************************************/
#include <stdio.h> 
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

//C++ includes
#include <vector>
#include <string>

//RHD Includes
#include <globalfunc.h>

#include "laserServer.h"

using namespace Stg;
using namespace std;

int laserPort;

typedef struct  {
  int fd;
  volatile int ready;     /* >0 to enable data to client */
} clStruct;

vector<clStruct> aclients;  //Vector of connected clients
string scangetStr;        //Scanget package string

void* serverThread(void *);
pthread_t laser_server_thread;
pthread_attr_t attr;



int initLaserServer(int port) {
	
 //Set port for laserServer to listen
 laserPort = port;

 //Reserve a long string for scanget data
 scangetStr.reserve(4096);
	
  //Spawn server thread
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  
  // Spawning transmit thread for Laser scanner data
    if (pthread_create(&laser_server_thread, &attr, serverThread, &aclients)) {
      perror("simStage: Error can't start LaserServer thread");
    return -1;
  }

  return 1;

}

/** Transmit laserscanner data to all connected clients
 * The function takes a pointer to a ModelLaser object, along with
 * the actual simulation time.
 *
 * From the data a AURS scanget package is created and pushed to
 * all connected clients.
 **/
int laserServerTransmit(ModelLaser *laserscan, stg_usec_t simtime) {

	static uint32_t laserUpdates = 0;
	char tmpStr[1024];
	Stg::ModelLaser::Config laserConfig = laserscan->GetConfig();
	double fov = laserConfig.fov * (180.0 / M_PI); //Convert Field of View to degrees
	double maxDist = laserConfig.range_bounds.max; //Max limit for measurement

	//Check if laser has been updated and only transmit laserscan if it has
	//And check if any clients actually are connected
	if ((laserscan->GetUpdates() > laserUpdates) && (aclients.size() > 0)) {
	
		//Start filling laserscan details into string
		scangetStr = "<scanget unit=\"mm\" "; //Static stuff
		sprintf(tmpStr,"count=\"%d\" ",laserConfig.sample_count);
		scangetStr += tmpStr;
		sprintf(tmpStr,"interval=\"%6.5f\" ", fov / laserConfig.sample_count);
		scangetStr += tmpStr;
		sprintf(tmpStr,"min=\"%5.4f\" max=\"%5.4f\" ",-fov/2,fov/2);
		scangetStr += tmpStr;
		sprintf(tmpStr,"tod=\"%f\"",(double)simtime/1000000); //Simtime in secs
		scangetStr += tmpStr;
		scangetStr += ">"; //Terminate scanget tag

		//Start binary laserscanner package
		sprintf(tmpStr,"<bin size=\"%d\" codex=\"HEX\">",laserConfig.sample_count*4);
		scangetStr += tmpStr;

		//Generate all range data in 4-digit HEX values in little endian encoding
		char hexStr[5] = {0,0,0,0,0};
		vector<Stg::ModelLaser::Sample> laserSamples;
		laserSamples = laserscan->GetSamples();
		int dist;
		for (size_t i = 0; i < laserConfig.sample_count; i++) {
			//Return a short distance, if measurement is close to maxdist
			if (laserSamples[i].range >= (maxDist - 0.01)) dist = 1; //Dist 1mm in max case
			else dist = (int)(laserSamples[i].range*1000);
			//Generate discance in HEX
			sprintf(tmpStr,"%04x",dist);
			 hexStr[0] = tmpStr[2];
			 hexStr[1] = tmpStr[3];
			 hexStr[2] = tmpStr[0];
			 hexStr[3] = tmpStr[1];
			 hexStr[4] = 0;
			 scangetStr += hexStr;
		}

		//Finish up XML string
		scangetStr += "</bin></scanget>\n";

		//Push the scanget string to all connected clients
		for (size_t i = 0; i < aclients.size(); i++) {
			if (secureSend(aclients[i].fd,scangetStr.c_str(),scangetStr.length()) <= 0) {
				//Close client and remove client from client table at any error
				close(aclients[i].fd);
				aclients.erase(aclients.begin() + i); //Erase the client from vector
				printf("   SimStage3: LaserServer Closing client %d\n",i+1);
			}

		}

		//Update the laserScan counter
		laserUpdates = laserscan->GetUpdates();
	}

	//printf("%s\n",scangetStr.c_str());

	return 1;
}


/** Laserscanner server thread
 *
 * Thread for accepting incomming clients for laser data.
 *
 * A virtually unlimited number of clients is accepted by using a vector
 * for client id storage
 *
 **/
void *serverThread(void *clIn)
{
  //Get the input client struct from argument pointer
  vector<clStruct> *cli = (vector<clStruct> *)clIn;

  int serverSocket;
  int knockingClient;
  clStruct *newClient;

  if ((serverSocket = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror("LaserServer: socket creating error");
      pthread_exit(0);
    }

  //Setup socket and bind
  struct sockaddr_in sa;
  sa.sin_family =  AF_INET;
  sa.sin_port = htons((uint16_t)laserPort);
  sa.sin_addr.s_addr = INADDR_ANY;


  if (bind(serverSocket, (struct sockaddr *) &sa, sizeof(sa))) {
    perror("SimStage3: LaserServer Error bind");
    pthread_exit(0);
  }

  if (listen(serverSocket, 2)) {
    perror("SimStage3: LaserServer socket listen error");
    pthread_exit(0);
  }
  
  usleep(500000); //Wait 500 ms for everyting to init.

  //Init went ok, mark server as running
  printf("   SimStage3: LaserScanner server running\n");

  //Thread main loop - Wait for clients and connect them
  while ((knockingClient = accept(serverSocket, NULL, NULL)) >= 0) {

	 newClient = new clStruct;
	 newClient->fd = knockingClient;
	 newClient->ready = 0;

	 //Push client to client-vector
	 cli->push_back(*newClient);
	 delete newClient;

	 printf("Simstage3: LaserServer client %u connected successfully\n",aclients.size());
  }
   
  fprintf(stdout,"LaserServer: Accept error - Shutting down!\n");

  //Close clients
  for(size_t i = 0; i < cli->size(); i++)  close((*cli)[i].fd);

  //Close server socket
  close(serverSocket);

  exit(0); //Shutdown application!
}
