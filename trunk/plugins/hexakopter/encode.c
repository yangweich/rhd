/**
 * Interface for Mikrokopter flight controller board
 * message coding and encoding
 *
 * $Rev: 82 $
 * $Id: encode.c 82 2012-12-16 14:43:23Z jcan $
 *
 *******************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// #define FC_ADDRESS 1 
// #define NC_ADDRESS 2 
// #define MK3MAG_ADDRESS 3 
// #define BL_CTRL_ADDRESS 5 
//#define MAX 170
//unsigned char txBuffer[MAX];
//unsigned char OUTBuffer[MAX];

//////////////////////////////////////////////////////////

int addCRC(char * txBuffer, unsigned int wieviele)
{
  unsigned int tmpCRC = 0,i;
  for(i = 0; i < wieviele;i++)
  {
    tmpCRC += txBuffer[i];
  }
  tmpCRC %= 4096;
  txBuffer[i++] = '=' + tmpCRC / 64;
  txBuffer[i++] = '=' + tmpCRC % 64;
  txBuffer[i++] = '\r';
  return i;
}

//////////////////////////////////////////////////////////////////////

int packData(char * txBuffer,  unsigned char cmd,unsigned char addr, unsigned char *snd, int len)
{
    unsigned int pt = 0;
    unsigned char a,b,c;
    unsigned char ptr = 0;
    int length = 0;
    // add header
    txBuffer[pt++] = '#';               // Start-Byte
    txBuffer[pt++] = 'a' + addr;        // Adress
    txBuffer[pt++] = cmd;               // Command
    while(len)
    { // add coded parameters
        if(len) { a = snd[ptr++]; len--;} else a = 0;
        if(len) { b = snd[ptr++]; len--;} else b = 0;
        if(len) { c = snd[ptr++]; len--;} else c = 0;
        txBuffer[pt++] = '=' + (a >> 2);
        txBuffer[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
        txBuffer[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
        txBuffer[pt++] = '=' + ( c & 0x3f);
    }
//    printf("Pt: %d\n",pt);
    length = addCRC(txBuffer, pt);
    return length;
}

//////////////////////////////////////////////////////////////////////

int decode64(unsigned char *ptrOut, unsigned char len, unsigned char ptrIn,unsigned char max, unsigned char *Buffer)
{
    unsigned char a,b,c,d;
    unsigned char ptr = 0;
    unsigned char  x,y,z;
        
      //  printf("%c",Buffer[0]);
    //    printf("%c",Buffer[1]);
  //      printf("%c",Buffer[2]);
        
    while(len)
    {
        a = Buffer[ptrIn++] - '=';
        b = Buffer[ptrIn++] - '=';
        c = Buffer[ptrIn++] - '=';
        d = Buffer[ptrIn++] - '=';
        if(ptrIn > max - 2) break;     // dont process more data than recieved
        x = (a << 2) | (b >> 4);
        y = ((b & 0x0f) << 4) | (c >> 2);
        z = ((c & 0x03) << 6) | d;
        if(len--){
            ptrOut[ptr++] = x; 
            //printf("%3d, ",ptrOut[ptr]);
  //          printf("%c",ptrOut[ptr]);
        }
            else break;
        if(len--){
            ptrOut[ptr++] = y; 
    //        printf("%c",ptrOut[ptr]);
       //     printf("%3d,",ptrOut[ptr]);
        }
            else break;
        if(len--){ 
            ptrOut[ptr++] = z; 
    //        printf("%c",ptrOut[ptr]);
         //   printf("%3d,",ptrOut[ptr]);
        }
        else break;
    }
//printf("\n \r");
  return ptr;
}   

