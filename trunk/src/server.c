/** \file server.c
 *  \ingroup core
 *  \brief TCP/IP socket server
 *
 *  This is the TCP socket server for the Robot Hardware Daemon
 *
 *  It maintains syncronisation between the variable databases in each end
 *  of the connection.
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1717 $
 *  $Date: 2011-10-11 12:31:38 +0200 (Tue, 11 Oct 2011) $
 */
 /***************************************************************************
 *                  Copyright 2008 Anders Billesø Beck                     *
 *                       anders.beck@get2net.dk                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/************************** Version control information ***************************/
 #define SERVER_VERSION   "$Rev: 1717 $:"
 #define DATE             "$Date: 2011-10-11 12:31:38 +0200 (Tue, 11 Oct 2011) $:"
/**********************************************************************************/

#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <expat.h>

#include <sys/poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "server.h"
#include "database.h"
#include "globalfunc.h"
#include "scheduler.h"

#define DEBUG 0

/** Global variables **/
struct config serverConfig;
struct client *clients;
int masterClient = -1;
int iterator, iterator2;
int *ioBuffer;
char *txBuffer, *txBufPtr;
int ioBufSize;

int serverSocket;
pthread_attr_t attr;
struct pollfd pollData;
static volatile char serverRunning;


/** Private function definitions **/
pthread_t server_thread;
void      *server_task(void *);
int       handshakeClient(int);
void      closeClient(int);
int       txBufferSymboltable(char);
int       txBufferDatatable(char, int);
int       recieveFromClients(void);
int       recieveClientPackages(int clId);
int       recieveWriteBuffer(int num);
int       updateVariableflags(void);
int		 transmitTxBuffer(int cId);
int		 loadTxBuffer(void *, int);


/** \brief Disconnect clients and shut down server
 *
 *
 * \returns int status
 * Status of shutdown - negative on error.
 */
int disconnectServer(void) {
  printf("   Server: Shutting down server and clients\n");
  for(iterator = 0; iterator < serverConfig.clients; iterator++)
  {
    if (clients[iterator].fd != -1)
      close(clients[iterator].fd);
    clients[iterator].fd  = -1;
    free(clients[iterator].rupdated);
    free(clients[iterator].wupdated);
  }

  close(serverSocket);
  serverRunning = -1;

  //Free memory
  free(clients);;
  free(ioBuffer);

  return 1;
}

/** \brief Close connection and free client
 *
 * \param[in] *int clientId
 */
void closeClient(int clientId) {
    printf("   Server: Closing client %d\n",clientId);
    close(clients[clientId].fd);
    clients[clientId].fd = -1;    //Mark client slot as free
    clients[clientId].ready = 0;
    if (clients[clientId].mode == 'w')
    {
      isMasterAlive--;
      if (isMasterAlive <= 0)
        masterClient = -1;
    }
    clients[clientId].mode = ' ';
}

/** \brief Server thread entry point
 *
 * This thread accepts connections from the clients
 * and is resposible for handshaking the clients
 *
 * \param[in] *void Currently not used
 *
 * \returns int status
 * Status of the server thread - negative on error.
 */
void *server_task(void *not_used)
{
  int i, client;

  #if (DEBUG)
	 printf("DEBUG Server: serverthread - wupdate:%p rupdate:%p\n",clients[0].wupdated,clients[0].rupdated);
#endif

  if ((serverSocket = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror("Server: socket creating error");
      serverRunning = -1;
      pthread_exit(0);
    }

  //Setup socket and bind
  struct sockaddr_in sa;
  sa.sin_family =  AF_INET;
  sa.sin_port = htons((uint16_t)serverConfig.port);
  sa.sin_addr.s_addr = INADDR_ANY;
  //inet_aton("127.0.0.1", &sa.sin_addr); //If host limitation should be used

  if (bind(serverSocket, (struct sockaddr *) &sa, sizeof(sa))) {
    perror("Server: Error bind");
    serverRunning = -1;
    pthread_exit(0);
  }

  if (listen(serverSocket, 1)) {
    perror("Server: socket listen error");
    serverRunning = -1;
    pthread_exit(0);
  }
  //Init went ok, mark server as running
  printf("   Server: Server thread is running\n");
  serverRunning = 1;

  //Thread main loop - Wait for clients and connect them
  while ((client = accept(serverSocket, 0, 0)) >= 0) {

    for (i = 0; i < serverConfig.clients; i++) {
      if (clients[i].fd == -1)
        {
          printf("Server: Client %d connected!!\n",i);
          clients[i].fd = client;
          clients[i].ready = 0;
          if (handshakeClient(i) < 0) {
            printf("Server: Error when handshaking client[%d]\n",i);
            closeClient(i);
          }
          clients[i].ready = 1; //Move this to recieve end
          break;
        }
    }
    if (i == serverConfig.clients) close(client);
  }

  for(i = 0; i < serverConfig.clients; i++) {
    if (clients[i].fd == -1) {
      closeClient(i);
    }
  }

  //Close server socket
  serverRunning = -1;
  close(serverSocket);

  fprintf(stderr,"Server: Error accepting client\n");
  fprintf(stderr,"Server: Shutting down server!\n");
  exit(0); //Shutdown application!
}

/** \brief Handshake client
 *
 * Handshaking between client and server
 *
 * Read or write status is established and the
 * symbol table is transferred to the client
 *
 * \param[in] int clientId
 *
 * \returns int status
 * Status of the handshake - negative on error.
 */

int handshakeClient(int clientId) {

	char dirReq[2];
	int i;

  //Read direction handshake from client
  if (secureRecv(clients[clientId].fd, &dirReq, sizeof(char)*2) <= 0) {
    printf("   Server: Error handshaking client %d.. Closing client",clientId);
    return -1;
  }

	if (dirReq[0] != 'a') {
		printf("   Server: Invalid RHD Server access request: %c%c\n",dirReq[0],dirReq[1]);
		return -1;
	}

  printf("   Server: Client request dir : %c",dirReq[1]);
  printf("---Database status:\n");
  printDatabase('r');
  printDatabase('w');

  //Read or write status?
  switch (dirReq[1]) {
    case 'w' :
      if (masterClient == -1 || serverConfig.allwriters) {
        masterClient = clientId;
        isMasterAlive++;
        //Send write access identifier to client
        if(secureSend(clients[clientId].fd,"aw",2) <= 0) {
            closeClient(clientId);
            return -1;
        }
        clients[clientId].mode = 'w';
      }
    break;
    case 'r' : //Return as reader if master used or only read access is defined
      //Send write access identifier to client
      if(secureSend(clients[clientId].fd,"ar",2) <= 0) {
          closeClient(clientId);
          return -1;
      }
      clients[clientId].mode = 'r';
    break;

  default: //Invalid direction
    printf("   Server: invalid direction: %c when handshaking client %d\n",dirReq[1],clientId);
    return -1;
  };

  //Buffer write and read symbol tables to client
  if(txBufferSymboltable('w') < 0) {
		closeClient(clientId);
      return -1;
  }
  #if (DEBUG)
	 printf("DEBUG Server: Handshaking client %d - wupdate:%p rupdate:%p\n",clientId,clients[0].wupdated,clients[0].rupdated);
#endif
  if(txBufferSymboltable('r') < 0) {
          closeClient(clientId);
          return -1;
  }

#if (DEBUG)
	 printf("DEBUG Server: Handshaking client %d - wupdate:%p rupdate:%p\n",clientId,clients[0].wupdated,clients[0].rupdated);
#endif


  //Buffer write and read datatables to client
  for(i = 0; i < getSymtableSize('w'); i++) clients[clientId].wupdated[i] = 1;
  if (txBufferDatatable('w',clientId) <= 0) {
	  closeClient(clientId);
	  return -1;
  }
  //Mark all read-variables as updated to ensure transmission to client
  for(i = 0; i < getSymtableSize('r'); i++) {
		clients[clientId].rupdated[i] = 1;
  }
  if (txBufferDatatable('r',clientId) <= 0) {
	  closeClient(clientId);
	  return -1;
  }
  //Finally transmit the txBuffer to clients
  transmitTxBuffer(clientId);

//   if (clientId == masterClient)
//     printf("   - w master status granted - now (%d masters)\n", isMasterAlive);
//   else
//     printf("   - r status granted (%d masters)\n", isMasterAlive);

  // debug print new client status
  printf("   New client status:\n");
  for(i = 0; i < serverConfig.clients; i++)
  {
    if (clients[i].mode > ' ')
    {
      if (i == clientId)
        printf("* ");
      else
        printf("  ");
      printf(" - client %2d is mode %c\n", i, clients[i].mode);
    }
  }
  printf("---\n");
  // debug end
  return 1;
}

/** \brief Syncronize variable database with clients
 *
 * \returns int status
 * Status of the transmission:
 * negative on error, zero if no clients connected and 1 successful transmission
 */
int syncClients(void) {

	int i;

  if (!serverRunning) return 0; //Detect shutdown...

    //Test if any clients connected at all...
	 for(i = 0; i < serverConfig.clients; i++) {
		if (clients[i].fd != -1) break;
	 }
	 if (i == serverConfig.clients) {
		return 0;
	 }

	//Reset updated flag
  lockDatabaseAccess(); //Lock  database mutex;
	resetUpdate('w');
  openDatabaseAccess();//Reopen mutex

  //Recieve data and ready-status from clients
  if (recieveFromClients() < 0)  return -1;

  //Transmit read-buffer to ready clients
  if(updateVariableflags() < 0) return -1; //Update each client structure

  //Loop through clients and transmit read and write tables
  for(i = 0; i < serverConfig.clients; i++) {
    if ((clients[i].fd != -1) && (clients[i].ready != 0)) {
		 //Prepare read and write datatable packages
		 if (i != masterClient || isMasterAlive > 1)
           txBufferDatatable('w',i); //No write table transmitted to master if one master only
		 txBufferDatatable('r',i);

		 //Send the txBuffer to clients in one TCP-package
		 if (transmitTxBuffer(i) <= 0) {
			 closeClient(i);
		 }
	 }
  }

  return 1; //Return 1 on success
}

/** \brief Update the updated flags for each connected client
 *
 * \returns int status
 * Status of the process. Negative on error.
 */
int updateVariableflags(void) {

    int i,j;
    symTableElement *tempSymtable;

    lockDatabaseAccess(); //Lock  database mutex;
    //Loop through all clients
    for (i = 0; i < serverConfig.clients; i++) {
        if (clients[i].fd != -1) {
			  tempSymtable = getSymtable('r');
            for (j = 0; j < getSymtableSize('r'); j++) {
                //Set the updated flag for each client
                clients[i].rupdated[j] |= tempSymtable[j].updated;
            }
				tempSymtable = getSymtable('w');
				for (j = 0; j < getSymtableSize('w'); j++) {
                //Set the updated flag for each client
                clients[i].wupdated[j] |= tempSymtable[j].updated;
            }
        }
    }
    openDatabaseAccess();//Reopen mutex

    return 1;

}

/** \brief Transfer symboltable to txBuffer
 *
 * \param[in] char dir
 * Direction of the symboltable (w/r)
 *
 * \returns int status
 * Status of the transmission - negative on error.
 */
int txBufferSymboltable(char dir) {

  symTableElement tempElement;
  int elCounter = 0;
  char pkgType = 't';


  //Load package type into txBuffer
  loadTxBuffer(&pkgType,1);

  //Buffer package direction
  loadTxBuffer(&dir,1);

  //Buffer package length
  elCounter = htonl(getSymtableSize(dir));
  loadTxBuffer(&elCounter,sizeof(int32_t));

  //printf("Sending a total of %d elements\n",getSymtableSize(dir));

  //Buffer the table (and preformat int32's to network byteorder)
  for(elCounter = 0; (elCounter < getSymtableSize(dir)); elCounter++) {
    //Copy to temp struct to perform htonl on required values
    memcpy(&tempElement, (getSymtable(dir) + elCounter),sizeof(symTableElement));
    tempElement.length = htonl(tempElement.length); //Only htonl on 32-bit values actually used after transfer
	 loadTxBuffer(&tempElement.length, sizeof(tempElement.length));
	 loadTxBuffer(&tempElement.name, sizeof(tempElement.name));
    //loadTxBuffer(&tempElement, sizeof(symTableElement));
    //printf("Element %d sent\n",elCounter);
  }

  return 1;
}

/** \brief Transfer datatable to txBuffer
 *
 * \param[in] char dir
 * Direction of the symboltable (w/r)
 *
 * \param[in] int cId
 * Client id of the symbol table
 *
 * \returns int status
 * Status of the transmission process. Negative on error.
 */
int txBufferDatatable(char dir, int cId) {

  //Get symtable pointer
  int j = 0;
  int *ioPtr;
  symTableElement *tempSymtable = getSymtable(dir);
  char pkgType = 'd';

  //Transfer direction header to txBufferr
  loadTxBuffer(&pkgType,1);
  loadTxBuffer(&dir,1);

   //Prepare ioBuffer for transmission
	ioBuffer[0] = 0; //Zero symbols payload so far
	ioPtr = ioBuffer + 1; //Point to first place in data area

   lockDatabaseAccess(); //Lock  database mutex;
   //Build data-package
	for(j = 0; j < getSymtableSize(dir); j++) {

		if (((dir == 'w') && (clients[cId].wupdated[j])) ||
		    ((dir == 'r') && (clients[cId].rupdated[j]))) {

			*ioPtr = j; //Save index
         ioPtr++;    //Move pointer

			//Copy timestamp
         memcpy(ioPtr, tempSymtable[j].timestamp, sizeof(struct timeval));
         ioPtr += sizeof(struct timeval) / sizeof(int32_t);
         //Copy data
         memcpy(ioPtr, tempSymtable[j].data, tempSymtable[j].length * sizeof(int32_t));
         ioPtr += tempSymtable[j].length;
         ioBuffer[0]++; //Increment payload counter
       }
    }
    openDatabaseAccess();//Reopen mutex

    //Transform all to network byteorder
    for(iterator = 0; iterator < (ioPtr - ioBuffer); iterator++) {
		ioBuffer[iterator] = htonl(ioBuffer[iterator]);
	 }

    //Transfer generated databuffer to txBuffer
    loadTxBuffer(ioBuffer,(ioPtr - ioBuffer)*sizeof(int32_t));

	 //Reset client
    clients[cId].ready = 0;
	 if (dir == 'w') {
		memset(clients[cId].wupdated,0,getSymtableSize(dir));
	 } else {
		memset(clients[cId].rupdated,0,getSymtableSize(dir));
	 }

	 return 1;
}


/** \brief Recieve client checkin
 *
 * Package parser of the client check ins
 *
 * \returns int status
 * Status of the reading process. Negative on error.
 */
int recieveFromClients(void) {

	pollData.events = POLLIN; //Check for normal data in socket inputbuffer
	int32_t		i, w;
	int32_t		masterWait = getSchedulerPeriod(); //Wait for 100 periods (in ms)

    //Clear updated
    for (i = 0; i < serverConfig.clients; i++) {
      //Set a waiting period, when polling for data from master client
      //in synchronous mode - with just one master
      if ((serverConfig.synchronized) &&
            (i == masterClient) &&
            (isMasterAlive == 1))
      {
        w = masterWait;
      } else {
        w = 0;
      }
      //Poll for data ready from client
      if (clients[i].fd != -1) {
          pollData.fd = clients[i].fd;
          //Test if any data is ready
          if(poll(&pollData,1,w)) {
            recieveClientPackages(i);
          }
      }
	}
	return 1;
}

int recieveClientPackages(int clId) {

	 char	pkgType[2], end = 0;

	//Reciever loop - Keeps recieving RHD messages until end-of-package is recieved
	while (!end) {
		//Recieve package type for parsing
		if (secureRecv(clients[clId].fd,&pkgType,sizeof(char) * 2) <= 0) {
			closeClient(clId);
			break;
		}

		//Parse the first package type identifyer byte
		switch (pkgType[0]) {
			//Ready package type recieved
		  case 'r':
			 #if DEBUG
			 printf("DEBUG: Received data table package: %c%c\n",pkgType[0],pkgType[1]);
			 #endif
			 //Client writing data or ready to read data
			 if (((pkgType[1] == 'w') && (clients[clId].mode == 'w')) ||
                  (pkgType[1] == 'r'))
              {
                clients[clId].ready = 1; //Mark client as ready
                #if DEBUG
                printf("Client is ready\n");
                #endif
              } else if (pkgType[1] == 'w')
              { //Fake write client
                closeClient(clId);
                end = 1;
                printf("   Server: Closing client[%d], as it tried to write without being write-enabled\n",clId);
              }
			break;
		  //Data-package type recieved
		  case 'd':
			#if DEBUG
			 printf("DEBUG: Receive write datapackage package: %c%c\n",pkgType[0],pkgType[1]);
			#endif

			//Only accept write packages
			if (pkgType[1] == 'w') {
				if (recieveWriteBuffer(clId) < 0) {
					closeClient(clId);
					end = 1;
					printf("   Server: Error receiving write package from client[%d] - Closing client",clId);
				}
			}
			break;

			//End-of-package recieved (or unknown type)
		  case 'e':
			 default:
			 #if DEBUG
			 printf("DEBUG: Recieved endofpackage: %c%c\n",pkgType[0],pkgType[1]);
			 #endif
			 end = 1;
			 break;
		};
	}
  return 1;
}

/** \brief Recieve write-databuffer from client
 *
 * Clients send a 0 int, if they are readers and wait to
 * check-in for update of read-buffer
 *
 * If client is writer, they send a data-package in the same format
 * as the read-package transmission
 *
 * \param[in] int num
 * Number of variable packages in the write package
 *
 * \returns int status
 * Status of the reading process. Negative on error.
 */
int recieveWriteBuffer(int clId) {

  symTableElement *tempSymtable = getSymtable('w');
  int32_t num, v, *d;

	//Read size of sync package from server
	if (secureRecv(clients[clId].fd,&num,sizeof(int32_t)) <= 0) {
		return -1;
	}

#if DEBUG
	printf("DEBUG: Recieved [%d][%d][%d][%d] write packages from client[%d]\n",(char)(num>>24),
			(char)(num>>16),(char)(num>>8),(char)(num),clId);
#endif
	num = ntohl(num); //Convert to host endian

  //Recieve the variables one at the time
  for(; num > 0; num--) {

    //Read variable id
    if (secureRecv(clients[clId].fd,ioBuffer,sizeof(int32_t)) <= 0) {
      closeClient(clId);
      return -1;
    }
    ioBuffer[0] = ntohl(ioBuffer[0]);

    //Read variable data
    if (secureRecv(clients[clId].fd,(ioBuffer+1),sizeof(struct timeval) + tempSymtable[ioBuffer[0]].length * sizeof(int32_t)) <= 0) {
      closeClient(clId);
      return -1;
    }

    //Convert and write timestamp
    lockDatabaseAccess(); //Lock  database mutex;
    tempSymtable[ioBuffer[0]].timestamp->tv_sec = ntohl(ioBuffer[1]);
    tempSymtable[ioBuffer[0]].timestamp->tv_usec = ntohl(ioBuffer[2]);
    //Convert data to host-byteorder and save data in symbol database.
    d = tempSymtable[ioBuffer[0]].data; // destination array
    tempSymtable[ioBuffer[0]].changed = 0;
    for(iterator2 = 0; (iterator2 <  tempSymtable[ioBuffer[0]].length); iterator2++)
    { //ioData [0]=size, [1]+[2] = timeval [3]... = data
      //(to explain strange array index below)
      v = ntohl(ioBuffer[iterator2 + sizeof(struct timeval)/sizeof(int32_t) + 1]);
      if (v != *d)
      { // value is changed
        *d = v;
        tempSymtable[ioBuffer[0]].changed = 1;
      }
      d++;
     //tempSymtable[ioBuffer[0]].data[iterator2]  = ntohl(ioBuffer[iterator2 + sizeof(struct timeval)/sizeof(int32_t) + 1]);
    }
    tempSymtable[ioBuffer[0]].updated = 1;
    // debug
    //printf("Received write variable %d %s: Data \n",
    //       ioBuffer[0],
    //       tempSymtable[ioBuffer[0]].name);
    //for(iterator2 = 0; (iterator2 <  tempSymtable[ioBuffer[0]].length); iterator2++)
    //   printf(" %d", tempSymtable[ioBuffer[0]].data[iterator2]);
    //printf("\n");
    // debug end
    openDatabaseAccess();//Reopen mutex
  }

  return 1;
}

/** \brief Load data into output tx buffer
 *
 * Loads data into the txBuffer for bufferedtransmission to clients
 *
 * \param[in] void *dataPtr
 * Datapointer to the input data area
 *
 * \param[in] int *dataLen
 * Size in bytes of the transmission data
 */

int loadTxBuffer(void *dataPtr, int dataLen) {

	//printf("Buffering[%d] (%p): ",dataLen,txBufPtr);
	memcpy(txBufPtr, dataPtr, dataLen);
	txBufPtr += dataLen; //shift bufferplace
	//printf("| (%p) len:%d\n",txBufPtr,(int)(txBufPtr-txBuffer));

	return 1;
}

/** \brief Transmit TX buffer to client
 *
 * \param[in] int cId
 * Client id
 */

int transmitTxBuffer(int cId) {

	char endPkgType[2] = {'e','e'};

	loadTxBuffer(&endPkgType,2); //Add end-package typeTag

	//Transmit databuffer to client
   if (secureSend(clients[cId].fd,txBuffer,(size_t)(txBufPtr - txBuffer)) <= 0) {
            //closeClient(cId);
				txBufPtr = txBuffer;//Reset pointers
				return -1;
   }

	txBufPtr = txBuffer; //Reset pointers
	return 1;

}

/** \brief Start server thread for connections
 *
 * Allocates ioBuffer memory and starts server thread
 *
 * \returns int status
 * Status of the startup process. Negative on error.
 */
int startServer(void) {

    int i;

    //Allocate clients
    clients = realloc(clients, serverConfig.clients * sizeof(struct client));
    memset(clients,0,serverConfig.clients * sizeof(struct client));

	 //Set socket file descriptor and allocate updated-buffer
    for(i = 0; i < serverConfig.clients; i++) {
        clients[i].fd = -1;
        clients[i].wupdated = realloc(clients[i].wupdated, getSymtableSize('w') * sizeof(char));
        memset(clients[i].wupdated,0,getSymtableSize('w') * sizeof(char));
		  clients[i].rupdated = realloc(clients[i].rupdated, getSymtableSize('r') * sizeof(char));
        memset(clients[i].rupdated,0,getSymtableSize('r') * sizeof(char));
    }


	 //Calculate transmission variable buffer
	 ioBufSize = (getDatabaseSize('r') + 1 + getDatabaseSize('w')) * sizeof(int32_t); //Data-buffer space
	 ioBufSize += (getSymtableSize('r') + 1 + getSymtableSize('w')) * sizeof(symTableElement); //Symbol buffer space
	 ioBufSize += (getSymtableSize('r') + 1 + getSymtableSize('w')) * sizeof(int32_t) * 2; //Header space
	 ioBufSize += 50; //Additional safety space

#if (DEBUG)
	 printf("DEBUG Server: Allocated %d bytes for ioBuffer and txBuffer\n",ioBufSize);
	 printf("DEBUG Server: Symboltable sizes r:%d, w:%d\n",getSymtableSize('r'),getSymtableSize('w'));
	 printf("DEBUG Server: wupdate:%p rupdate:%p\n",clients[0].wupdated,clients[0].rupdated);
#endif

	 //Allocate buffer memory
	ioBuffer = realloc(ioBuffer,ioBufSize);
	if (ioBuffer == NULL) return -1;
	memset(ioBuffer,0,ioBufSize);
	txBuffer = realloc(txBuffer,ioBufSize);
	if (txBuffer == NULL) return -1;
	memset(txBuffer,0,ioBufSize);

	txBufPtr = txBuffer; //Reset pointers

  //Setup and spawn server thread
	serverRunning = 0;
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  if (pthread_create(&server_thread, &attr, server_task, 0)) {
    perror("Server: Error can't start server thread");
    return -1;
  }

  //Wait for server thread to start and initialize
  while (serverRunning == 0) usleep(10000);

 return serverRunning;

}

/************************** XML Initialization **************************/
///Struct for shared parse data
typedef struct  {
    int depth;
    char skip;
    char enable;
  }parseInfo;

//Parsing functions
void XMLCALL serverStartTag(void *, const char *, const char **);
void XMLCALL serverEndTag(void *, const char *);

//XML File buffer size
#define XMLBUFLEN 8192
/** \brief Initialize the Server
 *
 * Reads the XML file and sets up the server settings
 *
 * Finally the server thread is started and the server
 * is ready to accept connections
 *
 * \param[in] *char filename
 * Filename of the XML file
 *
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
int serverInitXML(char *filename) {

  parseInfo xmlParse;
  char xmlBuf[XMLBUFLEN];
  int done = 0;
  int len;
  FILE *fp;


  printf("Server: Initializing RHD Server\n");

  //Clear configuration struct
  memset(&serverConfig,0,sizeof(struct config));

   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   if (! parser) {
    fprintf(stderr, "Server: Couldn't allocate memory for XML parser\n");
    return -1;
   }

   XML_SetElementHandler(parser, serverStartTag, serverEndTag);
   //Setup shared data
   xmlParse.depth = 0;
   xmlParse.skip =  0;
   xmlParse.enable = 0;
   XML_SetUserData(parser,&xmlParse);

  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("Server: Error reading: %s\n",filename);
    return -1;
  }
  len = fread(xmlBuf, 1, XMLBUFLEN, fp);
  fclose(fp);

  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "Server: XML Parse error at line %d: %s\n",
            (int)XML_GetCurrentLineNumber(parser),
            XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);

	return 1;

}

void XMLCALL
serverStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;

  //Check for the right 1. and 2. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) ||
        ((info->depth == 2) && (strcmp("server",el) != 0))) {
      info->skip = info->depth;
      return;
    }
  } else return;

  //Branch to parse the elements of the XML file.
  if (strcmp("port",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 3) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("value",attr[i]) == 0) serverConfig.port = atoi(attr[i+1]);
    printf("   Server: Using port %i\n",serverConfig.port);
  } else if (strcmp("clients",el) == 0) {
    //Check for the correct depth for this tag
    if(info->depth != 3) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2)
    {
      if (strcmp("number",attr[i]) == 0)
        serverConfig.clients = atoi(attr[i+1]);
      else if (strcmp("allwriters",attr[i]) == 0)
        serverConfig.allwriters = atoi(attr[i+1]);
    }
    printf("   Server: Accepting %d clients and allwriters is %d\n",
           serverConfig.clients, serverConfig.allwriters);
  } else if (strcmp("mode",el) == 0) { //Type of scheduler, for client synchronized scheduling
    //Check for the correct depth for this tag
    if(info->depth != 3) {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if ((strcmp("value",attr[i]) == 0) && (strcmp("synchronized",attr[i+1]) == 0)) {
		serverConfig.synchronized = 1;
		printf("   Server: Scheduler synchronized to master client\n");
	 }
  }

}

void XMLCALL
serverEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;

  if (info->depth < info->skip) info->skip = 0;
}

