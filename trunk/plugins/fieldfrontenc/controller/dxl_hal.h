#ifndef _DYNAMIXEL_HAL_HEADER
#define _DYNAMIXEL_HAL_HEADER


#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
//#include <sys/types.h>

//int dxl_hal_open(int devIndex, float baudrate);
void dxl_hal_open(uint8_t divisor, float baudrate);
void dxl_hal_close(void);
void dxl_hal_clear(void);
int8_t dxl_hal_tx( unsigned char *pPacket, int numPacket );
int dxl_hal_rx( unsigned char *pPacket, int numPacket );
//void dxl_hal_set_timeout( int NumRcvByte );
//int dxl_hal_timeout(void);
/// timer count - in timer units - set by timer interrupt
//extern volatile uint16_t gwCountNum;
/// time to send/receive one byte to/from servo
volatile uint16_t gfByteTransTime_us;
/// timer counter units 10 us should give maximum delay of ~640 ms
#define counterUnit_us 10


#ifdef __cplusplus
}
#endif

#endif
