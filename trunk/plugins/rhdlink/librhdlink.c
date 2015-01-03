/** \file librhdlink.c
 *  \brief Robot Hardware Daemon Library
 *  Socket interface to the Robot Hardware Daemon HAL 
 * 
 *  This files is based on librhd.c(Version 2.2 rev 1963) and has changed variable and function names in order to work with RHD link plugin.
 *  All function names has *Link in suffix.
 *  Global variables has been put in  a struct rhdlink_t
 *
 *  \author Peter Juhl Savnik
 *  \date 2015 January
 *  
 */
 /***************************************************************************
 *                  Copyright 2015 Peter Juhl Savnik                       *
 *                       s113556@student.dtu.dk                            *
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
 #define LIBRHDLINK_VERSION   "2.2"
 #define LIBRHDLINK_REVISION  "$Rev: 1963 $:"
 #define DATE             "$Date: 2012-07-28 19:16:28 +0200 (Sat, 28 Jul 2012) $:"
/**********************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>        
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <rhd.h>

#include "librhdlink.h"

//Enable debug printouts
#define DEBUG 0

/** Struct containning all global variables, allows to create multiple rhd link **/
typedef struct{
  int32_t         rhdSocket;
  int32_t         *rData, *wData;
  int32_t         *ioData, *ioPtr; //32-bit pointers to io-buffer
  int8_t	  *ioBuf, *ioBufPtr; //8-bit io-buffer
  int32_t         rTableLen, wTableLen;
  int32_t         rDataLen, wDataLen;
  char            connected;
  int             temp;
  symTableElement *rSymTableLink, *wSymTableLink;
} rhdLink_t;
  
/** Private function definitions **/
int rhdPackageParser(void);
int recieveSymboltable(char);
int recieveDatapackage (char dir);
int transmitWritepackage (void);
int transmitTxBuffer(void);
int loadTxBuffer(void *, int);
ssize_t secureWrite(int fd, const void *buf, ssize_t txLen);
ssize_t secureRead(int fd, void *buf, ssize_t txLen);


/** Variables definitions **/
rhdLink_t rhdLink;

/** Define initial condition **/
/*rhdLink.rTableLen = 0;
rhdLink.wTableLen = 0;
rhdLink.rDataLen = 0;
rhdLink.wDataLen = 0;
rhdLink.connected = 0;
*/

/** \brief Disconnect from RHD Server
 *
 * \returns char succes
 * Returns if disconnection was successfull
 */
char rhdDisconnectLink(void) {

  if (rhdLink.connected == 0)
    /* connected already */
    return -1;
  /* Disconnect */
  rhdLink.connected = 0;
  close(rhdLink.rhdSocket);
  /* set table length to zero */
  rhdLink.rTableLen = 0;
  rhdLink.wTableLen = 0;
  rhdLink.rDataLen = 0;
  rhdLink.wDataLen = 0;
  /* Free allocated memory */
  free(rhdLink.rSymTableLink);
  rhdLink.rSymTableLink = 0;
  free(rhdLink.wSymTableLink);
  rhdLink.wSymTableLink = 0;
  free(rhdLink.rData);
  rhdLink.rData = 0;
  free(rhdLink.wData);
  rhdLink.wData = 0;

  return 1;

}

/** \brief Connect to the RHD Server
 *
 * The connect call is *not* RT safe
 *
 * \param[in] char* hostname
 * Hostname to connect
 * If null is used, robot is connected to default host (see rhd.h)
 *
 * \param[in] int port
 * Port for the server
 * If null is used, robot is connected through the default port (see rhd.h)
 *
 * \param[in] char rw
 * Request Read or Write status (r or w as input)
 * 
 * \returns char rw
 * Returns the obtained read or write status (r or w returned) or negative on error
 */
char rhdConnectLink(char rw, char *hostname, int port) {

	struct sockaddr_in sa;
	int i;

  //Check for valid directions
  if ((rw != 'r') && (rw != 'w')) {
    //fprintf(stderr, "RHD Client: Invalid client status, only r or w can be used, you assigned %c\n",rw);
     return -1;
  }

  //Disconnect if already connected
  if (rhdLink.connected) {
    rhdDisconnectLink();
    rhdLink.connected = 0;
  }

  /*if (!hostname) {
     Disabled to get hostname from ENV variable!
    char *smr = getenv("SMR"); //I hate this function
    if (smr)  {
      char *p;
      hostname = strtok(smr, ":");
      p = strtok(0, "");
      if (p)  port = atoi(p);
    } 
  }*/ 

  //Resolve hostname and setup port
  if (hostname) {
    struct hostent *h = gethostbyname(hostname);
    if (!h) {
      fprintf(stderr, "RHD Client: Gethostbyname: can't find %s.\n", hostname);
      return -1;
    }
    sa.sin_family = h->h_addrtype;
    memcpy(&sa.sin_addr, h->h_addr_list[0], h->h_length);
  } else {
      sa.sin_family = AF_INET;
      sa.sin_addr.s_addr = INADDR_LOOPBACK;
      inet_aton(DEFAULTHOST, &sa.sin_addr);
  }
  sa.sin_port = htons(port ? port : DEFAULTPORT);

  //Connect to server and configure socket connection
  if ((rhdLink.rhdSocket = socket(PF_INET, SOCK_STREAM, 0)) < 0)  {
      perror("RHD Client: socket");
      return -1;
  }

  if (connect(rhdLink.rhdSocket, (struct sockaddr *)&sa, sizeof(sa)))  {
      //perror("RHD Client: connect");
      close(rhdLink.rhdSocket);
      return -1;
  }

  int x = 1;
  if (setsockopt(rhdLink.rhdSocket, SOL_TCP, TCP_NODELAY, &x, sizeof(x))) {
    perror("RHD Client: setsockopt - TCP NODELAY");
  }

  //Send access request to rhd server
  	char dirReq[2] = {'a', 0};
	dirReq[1] = rw; //Assign direction request
  //Send Direction request to server
  if (secureWrite(rhdLink.rhdSocket, &dirReq, sizeof(char)*2) < 0) {
    fprintf(stderr,"RHD Client: Error handshaking server\n");
    return -1;
  }

  //Run the package parser to recieve acknowledgement and symboltables
	if (rhdPackageParser() <= 0) {
		fprintf(stderr, "RHD Client: Fatal error recieving RHD Handshake packages, closing socket\n");
		close(rhdLink.rhdSocket);
		rhdLink.connected = 0;
		return -1;		
	}


	//Reset all updated flags in buffer (as all data is transferred in first handshake)
	for(i = 0; i < rhdLink.rTableLen; i++) rhdLink.rSymTableLink[i].updated = 0;
	for(i = 0; i < rhdLink.wTableLen; i++) rhdLink.wSymTableLink[i].updated = 0;

  if (rhdLink.connected <= 0) {
    fprintf(stderr, "RHD Client: Fatal error in RHD Server handshake, closing socket \n");
    close(rhdLink.rhdSocket);
    rhdLink.connected = 0;
    return -1;
  }

  return rhdLink.connected;

}

/** \brief Syncronize with the RHD Server
 * 
 * Updated write variables will be transmitted to the server,
 * if write access is granted. 
 * 
 * When transmission is finished, the client *waits* for the RHD Server
 * to respond with the updated read-variables
 * 
 * \returns char success
 * Returns success of the sync
 */
char rhdSyncLink(void) {

	char readyPkgType[2] = {'r','r'};
	int i;
	
#if DEBUG
	printf("DEBUG: Starting Sync\n");
#endif

	if (!rhdLink.connected) return -1;

	rhdLink.ioBufPtr = rhdLink.ioBuf; //Clear io buffer pointer for new tx package

	//Create the ready-package and set rr / rw, then load to ioBuffer
	readyPkgType[1] = rhdLink.connected; //Update ready-checkin package type
	loadTxBuffer(readyPkgType,sizeof(char)*2); //Load to buffer

	//Transmit write variables if write access is granted
	if (rhdLink.connected == 'w') {
		#if DEBUG
		printf("DEBUG: Transmitting write package\n");
		#endif
		if (transmitWritepackage() <= 0) {
			rhdDisconnectLink();
			return -1;
		}
	}

	//Finally transmit write package to client
	transmitTxBuffer();

#if DEBUG
	printf("DEBUG: Write package transmitted\n");
#endif

  //Reset all updated flags in buffer
  for(i = 0; i < rhdLink.rTableLen; i++) rhdLink.rSymTableLink[i].updated = 0;
  for(i = 0; i < rhdLink.wTableLen; i++) rhdLink.wSymTableLink[i].updated = 0;

  //Run the RHD package parser to recieve data from RHD Server
  if (rhdPackageParser() <= 0) {
	  rhdDisconnectLink();
	  return -1;
  }

  return 1;
}

/** \brief Reciever algorithm for parsing package from the RHD server
 *
 * \returns int status
 * Negative on communication errors
 */
int rhdPackageParser(void) {

  char pkgType[2];
  char end = 0;

  //Reciever loop - Keeps recieving RHD packages until end-of-package is recieved
  while (!end) {
    //Recieve package type for parsing
    if (secureRead(rhdLink.rhdSocket,&pkgType,sizeof(char) * 2) <= 0) {
    return -1;
    }

    //Parse the first package type identifyer byte
    switch (pkgType[0]) {
      //Access acknowledgement package type received
      case 'a':
        //Check for valid access types
        if ((pkgType[1] == 'w')||(pkgType[1] == 'r')) {
          #if DEBUG
          printf("DEBUG: Received connect package %c%c\n",pkgType[0],pkgType[1]);
          #endif
          rhdLink.connected = pkgType[1];
        } else {
          return -1;
        }
        break;

      //Symbol table package type recieved
      case 't':
        //Parse for read or write table package
        #if DEBUG
        printf("DEBUG: Received symbol table package: %c%c\n",pkgType[0],pkgType[1]);
        #endif
        if (recieveSymboltable(pkgType[1]) <= 0) {
          return -1;
        }
        break;

      //Data-package type recieved
      case 'd':
        #if DEBUG
        printf("DEBUG: Received data table package: %c%c\n",pkgType[0],pkgType[1]);
        #endif
        if (recieveDatapackage(pkgType[1]) <= 0) {
          return -1;
        }
        break;

      //End-of-package recieved (or unknown type)
      case 'e':
      default:
        #if DEBUG
        printf("DEBUG: Received endofpackage: %c%c\n",pkgType[0],pkgType[1]);
        #endif
        end = 1;
        break;
    };
  }

  return 1;

}


/** \brief Recieve and parse symbol table from server
 *
 * Memory for the datapool is allocated and pointers 
 * are re-created
 * 
 * \param[in] char dir
 * Direction of the symboltable (w/r)
 * 
 * \returns int status
 * Status of the transmission - negative on error.
 */
int recieveSymboltable(char dir) {
  symTableElement tempElement, *tempTable;
  int *tempDatapool, *tmpPtr;
  int i = 0, tempTableSize = 0, tempDataSize = 0;
  //Select the dataPools for read/write
  if (dir == 'w') {
    tempTable = rhdLink.wSymTableLink;
    tempDatapool = rhdLink.wData;
  } else if (dir == 'r') {
    tempTable = rhdLink.rSymTableLink;
    tempDatapool = rhdLink.rData;
  } else {
    return -1;
  }
  //Recieve size of symbol table from client and allocate memory for it
  if(secureRead(rhdLink.rhdSocket,&tempTableSize,sizeof(int32_t)) <= 0) 
    return -1;
  tempTableSize = ntohl(tempTableSize);
  tempTable = realloc(tempTable,(tempTableSize * sizeof(symTableElement)));
  if(tempTable == NULL) {
    fprintf(stderr,"RHD Client: Error allocating memory for %c-symboltable\n",dir);
    return -1;
  }
  //Clear table memory area
  memset(tempTable,0,(tempTableSize * sizeof(symTableElement)));
  //Recieve the table and restore host byteorder in selected elements
  for(i = 0; (i < tempTableSize); i++) {
    if(secureRead(rhdLink.rhdSocket,&tempElement.length,sizeof(tempElement.length)) <= 0) 
      return -1;
    if(secureRead(rhdLink.rhdSocket,&tempElement.name,sizeof(tempElement.name)) <= 0) 
      return -1;
    //Copy to temp struct and perform ntohl on required values
    tempElement.length = ntohl(tempElement.length);
    memcpy((tempTable + i), &tempElement,sizeof(symTableElement));    
    //Count the size of the datapool
    tempDataSize += sizeof(struct timeval)/sizeof(int32_t) + tempElement.length;
  }
  //Allocate memory for the datapool
  tempDatapool = realloc(tempDatapool,tempDataSize * sizeof(int32_t));
  if(tempDatapool == NULL) {
    fprintf(stderr,"RHD Client: Error allocating memory for %c-datapool\n",dir);
    return -1;
  }
  //Clear the dataPool memory area
  memset(tempDatapool,0,tempDataSize * sizeof(int32_t));
  //Restore pointers in the symbol table
  for(i = 0, tmpPtr = tempDatapool; (i < tempTableSize); i ++) {
    tempTable[i].timestamp = (struct timeval *) tmpPtr;
    tmpPtr += sizeof(struct timeval)/sizeof(int32_t);
    tempTable[i].data = tmpPtr;
    tmpPtr += tempTable[i].length;
  }

  //Reassign buffer pointers and sizes
  if (dir == 'w') {
    rhdLink.wSymTableLink = tempTable;
    rhdLink.wData = tempDatapool;
    rhdLink.wTableLen = tempTableSize;
    rhdLink.wDataLen = tempDataSize;
    //Allocate memory for the transmission datapool (header + n*(id + data))
    rhdLink.ioBuf = realloc(rhdLink.ioBuf,(rhdLink.wDataLen + 3 + rhdLink.wTableLen)  * sizeof(int32_t));
    if(rhdLink.ioBuf == NULL) {
      fprintf(stderr,"RHD Client: Error allocating memory for ioDatapool\n");
      return -1;
    }
    rhdLink.ioData = (int32_t *)(rhdLink.ioBuf+2); //Leave 2 bytes in ioBuf for package type
    memset(rhdLink.ioBuf,0,(rhdLink.wDataLen + 3 + rhdLink.wTableLen)  * sizeof(int32_t));
  } else {
    rhdLink.rSymTableLink = tempTable;
    rhdLink.rData = tempDatapool;
    rhdLink.rTableLen = tempTableSize;
    rhdLink.rDataLen = tempDataSize;
    //Allocate memory for the transmission datapool (header + n*(id + data))
    if ((rhdLink.wDataLen + 3 + rhdLink.wTableLen) < (rhdLink.rDataLen + 3 + rhdLink.rTableLen))
      rhdLink.ioBuf = realloc(rhdLink.ioBuf,(rhdLink.rDataLen + 3 + tempTableSize)  * sizeof(int32_t));
    if(rhdLink.ioBuf == NULL) {
      fprintf(stderr,"RHD Client: Error allocating memory for ioDatapool\n");
      return -1;
    }
    rhdLink.ioData = (int32_t *)(rhdLink.ioBuf+2); //Leave 2 bytes in ioBuf for package type
    memset(rhdLink.ioBuf,0,(rhdLink.rDataLen + 3 + tempTableSize)  * sizeof(int32_t));
  }
  return 1;
}


/** \breif Recieve RHD data package
 *
 * \param[in] char dir
 * Direction of the datatable in the package
 */
int recieveDatapackage (char dir) {
	
  symTableElement *tempTable;
  //int *tempDatapool;
  int pkgSize, i, u;
  int32_t v, *d;

  //Select the dataPools for read/write
  if (dir == 'w') {
    tempTable = rhdLink.wSymTableLink;
    //tempDatapool = wData;
  } else if (dir == 'r') {
    tempTable = rhdLink.rSymTableLink;
    //tempDatapool = rData;
  } else {
	  return -1;
  }

  //Read size of sync package from server
  if (secureRead(rhdLink.rhdSocket,&pkgSize,sizeof(int32_t)) <= 0) {
      return -1;
  }
  pkgSize = ntohl(pkgSize); //Convert to host-order

  //Recieve updated packages from server
  for(; pkgSize > 0; pkgSize--) {
      //Read variable id
      if (secureRead(rhdLink.rhdSocket,rhdLink.ioData,sizeof(int32_t)) <= 0) {
        return -1;
      }
      rhdLink.ioData[0] = ntohl(rhdLink.ioData[0]); //Convert to host order

      //Read variable data
      if (secureRead(rhdLink.rhdSocket,(rhdLink.ioData+1),sizeof(struct timeval) + (tempTable[rhdLink.ioData[0]].length * sizeof(int32_t))) <= 0) {
        return -1;
      }

      //Convert and write timestamp
      tempTable[rhdLink.ioData[0]].timestamp->tv_sec = ntohl(rhdLink.ioData[1]);
      tempTable[rhdLink.ioData[0]].timestamp->tv_usec = ntohl(rhdLink.ioData[2]);
      //Convert data to host-byteorder and copy to rDatabuffer.
      d = tempTable[rhdLink.ioData[0]].data;
      u = 0;
      for(i = 0; (i <  tempTable[rhdLink.ioData[0]].length); i++) {
        //ioData [0]=size, [1]+[2] = timeval [3]... = data
        //(to explain strange array index below)
        v = ntohl(rhdLink.ioData[i + sizeof(struct timeval)/sizeof(int32_t) + 1]);
        if (v != *d)
        {
          *d = v;
          u = 1;
        }
        d++;
      }
      // Mark variable as changed if so
      tempTable[rhdLink.ioData[0]].changed = u;
      // mark read variables as updated (data source is alive and setting data)
      if (dir == 'r') 
        tempTable[rhdLink.ioData[0]].updated = 1;
  }
  return 1;
}

/** \brief Transmit write table
 *
 * Creates a datapackage and transmits the write table
 */
int transmitWritepackage (void) {

	int32_t i;

	//Load message type
	loadTxBuffer("dw",sizeof(char)*2);

	//Set ioPtr
	rhdLink.ioData = (int32_t *) rhdLink.ioBufPtr;
	rhdLink.ioPtr  = rhdLink.ioData + 1; //Skip the first place for variable counter

	//Clear the variable count (first place in the buffer)
	rhdLink.ioData[0] = 0;

	for(i = 0; i < rhdLink.wTableLen; i++) {
		if (rhdLink.wSymTableLink[i].updated >= 1) {
			*rhdLink.ioPtr = i; //Save index
			rhdLink.ioPtr++;    //Move pointer
			//Copy timestamp
			memcpy(rhdLink.ioPtr, rhdLink.wSymTableLink[i].timestamp, sizeof(struct timeval));
			rhdLink.ioPtr += sizeof(struct timeval) / sizeof(int32_t);
			//Copy data
			memcpy(rhdLink.ioPtr, rhdLink.wSymTableLink[i].data, rhdLink.wSymTableLink[i].length * sizeof(int32_t));
			rhdLink.ioPtr += rhdLink.wSymTableLink[i].length;
			rhdLink.ioData[0]++;
			rhdLink.wSymTableLink[i].updated = 0; //Remove updated mark
		}
	}

#if DEBUG
	printf("DEBUG: Package transmitted: ");
	for (i = 0; i < ((ioPtr - ioData)*sizeof(int32_t)) + 2; i++)
		printf("[%d]",ioBuf[i]);
	printf("\n");
#endif

	//Transform all to network byteorder
	for(i = 0; i < (rhdLink.ioPtr - rhdLink.ioData); i++) {
		rhdLink.ioData[i] = htonl(rhdLink.ioData[i]);
	}

	//Set ioBufPtr to match filled buffer
	rhdLink.ioBufPtr = (int8_t *) rhdLink.ioPtr;

#if DEBUG
	printf("DEBUG: Size at network order: %d ",ioData[0]);

#endif

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

	#if DEBUG
	printf("Buffering[%d] (%p): ",dataLen,rhdLink.ioBufPtr);
	#endif
	memcpy(rhdLink.ioBufPtr, dataPtr, dataLen);
	rhdLink.ioBufPtr += dataLen; //shift bufferplace
	#if DEBUG
	printf("| (%p) len:%d\n",rhdLink.ioBufPtr,(int)(rhdLink.ioBufPtr-rhdLink.ioBuf));
	#endif

	return 1;
}

/** \brief Transmit TX buffer to client
 *
 * \param[in] int cId
 * Client id
 */

int transmitTxBuffer(void) {

	char endPkgType[2] = {'e','e'};

	loadTxBuffer(&endPkgType,2); //Add end-package typeTag
	
	//Transmit databuffer to client
   if (secureWrite(rhdLink.rhdSocket,rhdLink.ioBuf,(ssize_t)(rhdLink.ioBufPtr - rhdLink.ioBuf)) <= 0) {
				rhdLink.ioBufPtr = rhdLink.ioBuf;//Reset pointers
				return -1;
   }

	rhdLink.ioBufPtr = rhdLink.ioBuf; //Reset pointers
	return 1;

}


/** \brief Make sure that all is written to socket
 * 
 * \param[in] int fd
 * \param[in] int buf
 * \param[in] size_t txLen
 * 
 * \returns ssize_t txLen
 */
ssize_t secureWrite(int fd, const void *buf, ssize_t txLen) {

  int i, acc = 0;
  while (acc < txLen) {
    i = send(fd, buf+acc, txLen - acc, MSG_NOSIGNAL); //Ignore SIGPIPE
    if (i <= 0) return i;
    acc += i;
  }
  return (ssize_t)acc;
}

/** \brief Make sure that all is read from socket
 * 
 * \param[in] int fd
 * \param[in] int buf
 * \param[in] size_t txLen
 * 
 * \returns ssize_t txLen
 */
ssize_t secureRead(int fd, void *buf, ssize_t txLen) {

  int i, acc = 0;
  while (acc < txLen) {
    i = recv(fd, buf+acc, txLen - acc, MSG_NOSIGNAL); //Ignore SIGPIPE
    if (i <= 0) return i;
    acc += i;
  }
  return (ssize_t)acc;
}

/** \brief Get a pointer to the symbol table
 * 
 * \param[in] char dir
 * Direction of symbol table ('r' or 'w')
 *
 * \returns int size
 * Returns a pointer of the database in the given direction
 * Null is returned in case of error
 */
symTableElement*  getSymbolTableLink(char dir) {

  if (dir == 'r') {
    return (symTableElement *) rhdLink.rSymTableLink;
  } else if (dir == 'w') {
    return (symTableElement *) rhdLink.wSymTableLink;
  } else return NULL;
}


/** \brief Get the number of elements in the symbol table
 * 
 * \param[in] char dir
 * Direction of symbol table ('r' or 'w')
 *
 * \returns int size
 * Returns the number of elements in the symboltable in the given direction
 * Negative is returned in case of error
 */
int getSymbolTableSizeLink(char dir) {

  if (dir == 'r') {
    return rhdLink.rTableLen;
  } else if (dir == 'w') {
    return rhdLink.wTableLen;
  } else return -1;

}

/** \brief Update a variable in the data pool
 * 
 * \param[in] int id
 * Variable Id
 * 
 * \param[in] int index
 * Variable index in the array
 * 
 * \param[in] int value
 * Value of the variable 
 *
 * \returns int success
 * Returns the if the variable is successfully updated
 */
int writeValueLink(int id, int index, int value) {

//	printf("Writing value: %s\n",wSymTable[id].name);

  //Check bounds of update
  if ((id < 0) || (id >= rhdLink.wTableLen) || (index < 0) || 
      (index > rhdLink.wSymTableLink[id].length)) {
    return -1;
  }
  rhdLink.wSymTableLink[id].data[index] = value;
  rhdLink.wSymTableLink[id].updated = 1;
  gettimeofday(rhdLink.wSymTableLink[id].timestamp,NULL);

  return 1;

}

/** \brief Update a named variable in the data pool
 * 
 * \param[in] char *varName8
 * Variable name
 * 
 * \param[in] int index
 * Variable index in the array
 * 
 * \param[in] int value
 * Value of the variable 
 *
 * \returns int success
 * Returns the if the variable is successfully updated
 */
int writeValueNamedLink(char *varName, int index, int value) {

    int i;
    for (i = 0; i < rhdLink.wTableLen; i++) {
      if (!strcmp(varName,rhdLink.wSymTableLink[i].name)) {
        return writeValueLink(i,index,value);
      }
    }
    //Return error-value if no name found
    return -1;  

}

/** \brief Update a array in the data pool
 * 
 * \param[in] int id
 * Variable Id
 * 
 * \param[in] int length
 * Length of the input array
 * 
 * \param[in] int* array
 * Pointer to the array
 *
 * \returns int success
 * Returns the if the variable is successfully updated
 */
int writeArrayLink(int id, int length, int* array) {
  
//Check bounds of update
  if ((id < 0) || (id >= rhdLink.wTableLen) || (length < 0) || 
      (length > rhdLink.wSymTableLink[id].length)) {
    return -1;
  }
  memcpy(rhdLink.wSymTableLink[id].data, array, length * sizeof(int32_t));
  rhdLink.wSymTableLink[id].updated = 1;
  gettimeofday(rhdLink.wSymTableLink[id].timestamp,NULL);

  return 1;
}

/** \brief Update a named  array in the data pool
 * 
 * \param[in] char *arrayName
 * Array name
 * 
 * \param[in] int length
 * Length of the input array
 * 
 * \param[in] int* array
 * Pointer to the array
 *
 * \returns int success
 * Returns the if the variable is successfully updated
 */
int writeArrayNamed(char *arrayName, int length, int* array) {

    int i;
    for (i = 0; i < rhdLink.wTableLen; i++) {
      if (!strcmp(arrayName,rhdLink.wSymTableLink[i].name)) {
        return writeArrayLink(i,length,array);
      }
    }
  //Return error-value if no name found
  return -1;  
}


/** \brief Read an variable from the datapool
 * 
 * \param[in] int id
 * Variable Id
 * 
 * \param[in] int length
 * Length of the input array
 *
 * \returns int value
 */
int readValueLink(int id, int index) {
  
  //Check bounds of update
  if ((id < 0) || (id >= rhdLink.rTableLen) || (index < 0) || 
      (index > rhdLink.rSymTableLink[id].length)) {
    return 0;
  }
  return rhdLink.rSymTableLink[id].data[index];
}

/** \brief Read named variable from the datapool
 * 
 * \param[in] int id
 * Variable Id
 *
 * \returns int array
 */
  int readValueNamedLink(char *valName, int index) {
  
    int i;
    for (i = 0; i < rhdLink.rTableLen; i++) {
      if (!strcmp(valName,rhdLink.rSymTableLink[i].name)) {
        return readValueLink(i,index);
      }
    }
  //Return error-value if no name found
  return 0;
}

/** \brief Read an array from the datapool
 * 
 * \param[in] int id
 * Variable Id
 *
 * \returns int* array
 */
int*  readArrayLink(int id) {
  //Check bounds of update
  if ((id < 0) || (id >= rhdLink.rTableLen)) {
    return NULL;
  }
  return (int32_t*)rhdLink.rSymTableLink[id].data;
}

/** \brief Read named array from the datapool
 * 
 * \param[in] char *arrayName
 * Variable name
 *
 * \returns int* array
 */
  int * readArrayNamedLink(char *arrayName) {
  
    int i;
    for (i = 0; i < rhdLink.rTableLen; i++) {
      if (!strcmp(arrayName,rhdLink.rSymTableLink[i].name)) {
        return readArrayLink(i);
      }
    }
    //Return error-value if no name found
    return NULL;  
  }
  
  
 /** \brief Check if read-variable has been updated
 * 
 * \param[in] int id
 * Variable id
 *
 * \returns int updated
 */
int isReadLink(int id) {

  if (id >= rhdLink.rTableLen) return -1;
  else return rhdLink.rSymTableLink[id].updated;
  
}

 /** \brief Check if named read-variable has been updated
 * 
 * \param[in] int id
 * Variable id
 *
 * \returns int updated
 */
int isReadNamedLink(char *varName) {

  int i;
  for (i = 0; i < rhdLink.rTableLen; i++) {
    if (!strcmp(varName,rhdLink.rSymTableLink[i].name)) {
      return isReadLink(i);
    }
  }
  //Return error-value if no name found
  return -1;  

}




