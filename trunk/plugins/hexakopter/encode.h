/**
 * Interface for Mikrokopter flight controller board
 * message coding and encoding
 *
 * $Rev: 82 $
 * $Id: encode.h 82 2012-12-16 14:43:23Z jcan $
 *
 *******************************************************************/
#ifndef ENCODE_H
  #define ENCODE_H

  #ifdef __cplusplus
  extern "C" {
  #endif

/** add CRC bytes
 * \param txBuffer is the buffer to receive to packed message (must be big enough (170 is enough)
 * \param viefile is count of bytes to pack
 * \returns total number of bytes in message */
int addCRC(char * txBuffer, unsigned int wieviele);

/** encode a message
 * \param txBuffer is the buffer to receive to packed message (must be big enough (170 is enough)
 * \param cmd is command character
 * \param addr is address of receiving board 1..5,
 * \param snd is message parameters as binary unsigned chars
 * \param len is length of message parameters
 * \returns total number of bytes in message */
int packData(char * txBuffer, unsigned char cmd,unsigned char addr, unsigned char *snd, int len);

/**
 * Decode a parameter part of message from mikrokopter
 * \param ptrOut is pointer to structure to receive data
 * \param len is length of structure to receive the message
 * \param ptrIn is index to raw parameter part of message (in Buffer) - usually 3
 * \param max is index to last byte to decode (in Buffer)
 * \param Buffer is buffer with message to decode */
int decode64(unsigned char *ptrOut, unsigned char len, unsigned char ptrIn,unsigned char max, unsigned char *Buffer);

  #ifdef __cplusplus
  }
  #endif

#endif

