//##########################################################
//##                      dxl_hal.h		                  ##
//##     low level Servo control utility functions        ##
//##                                   Version 2012.06.01 ##
//##########################################################

#ifndef _DYNAMIXEL_HAL_HEADER
#define _DYNAMIXEL_HAL_HEADER


#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

int dxl_hal_open(int deviceIndex, float baudrate);
void dxl_hal_close();
int dxl_hal_set_baud( float baudrate );
void dxl_hal_clear();
int dxl_hal_tx( unsigned char *pPacket, int numPacket );
int dxl_hal_rx( unsigned char *pPacket, int numPacket );
void dxl_hal_set_timeout( int NumRcvByte );
int dxl_hal_timeout();
uint64_t CurrentMicrosec();

int dxl_hal_poll(); // return 1 if there is data available, 0 if a timeout event occured
//uint64_t CurrentMicrosec();

#ifdef __cplusplus
}
#endif

#endif
