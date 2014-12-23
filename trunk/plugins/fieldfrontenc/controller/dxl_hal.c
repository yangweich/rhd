#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/delay.h>
#include "dxl_hal.h"

// makes the interrupt routine enable further (nested) interrupts / chr
//#define ISR_NOBLOCK

#define MAXNUM_DXLBUFF	256
// Porting - field steer
// port C.0: 1 = read enable rs232
// port C.1: 1 = transmit enable rs232
// port C.2: 0 = read enable rs485
// port C.3: 1 = transmit enable rs485
// RX port C = 0b0001
// TX port C = 0b1110
#define DIR_TXD  PORTC &= ~(0x01), PORTC |= 0x0e
#define DIR_RXD  PORTC &= ~(0x0e), PORTC |= 0x01


volatile unsigned char gbDxlBuffer[MAXNUM_DXLBUFF] = {0};
volatile unsigned char gbDxlBufferHead = 0;
volatile unsigned char gbDxlBufferTail = 0;
//volatile uint16_t gfByteTransTime_us;
//volatile unsigned int gwCountNum;
//volatile unsigned int gwTimeoutCount;
//volatile unsigned int gwReturnDelayCountNum;

int dxl_hal_get_qstate(void);
void dxl_hal_put_queue( unsigned char data );
unsigned char dxl_hal_get_queue(void);


/**
 * Sets receive interrupt and USART ports for communication with dynamixel servos on field steer board
 * \param divisor is baudrate divisor - 34 for 57600 bit/s on teensy2++ - at CPU frequency and double baudrate
 * \param baudrate actual desired baudrate - 57600 - used to calculate receive time
 * \returns 1 */
void dxl_hal_open(uint8_t divisor, float baudrate)
{
  //float byteTransferTime_us;
  // USART Baud rate: actually dynamexal is 57142 bit/sec, and
  // this fits with double speed (With 16 MHz Clock) and UBRR=34
  UBRR1H = (divisor >> 8) & 0x7F;   //Make sure highest bit(URSEL) is 0 indicating we are writing to UBRRH
  UBRR1L =  divisor;
  //Double the UART Speed
  UCSR1A = (1 << U2X1) | (1 << TXC1);
  //Enable Rx and Tx in UART and Rx interrupt
  UCSR1B = (1 << RXEN1) | (1 << RXCIE1) | (1 << TXEN1);
  UCSR1C = (1 << UCSZ10) | (1 << UCSZ11);  //8-Bit Characters, no parity 1 stop bit
  // dynamixel communication using UART0
  //byteTransferTime_us = 1000000.0 / (float)baudrate * 12.0 + 0.5;
  //gfByteTransTime_us = byteTransferTime_us;
  //gwReturnDelayCountNum = (unsigned int)(250.0 / gfByteTransTime_us);
  // initialize simplex direction
  DIR_RXD;
  // clear 16 bit read flag
  UDR1 = 0xFF;
  // set buffer as empty
  gbDxlBufferHead = 0;
  gbDxlBufferTail = 0;
}

/**
 * we do not actually close anything */
void dxl_hal_close(void)
{
	// Closing device
}

/**
 * Clear buffer */
void dxl_hal_clear(void)
{ // Clear communication buffer
  gbDxlBufferHead = gbDxlBufferTail;
}

/**
 * Transmit a package to servo bus
 * \param packet is pointer to packet specification and
 * \param numPacket is number of bytes to transmit
 * \returns number of bytes send */
int8_t dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
  // Transmiting date
  // *pPacket: data array pointer
  // numPacket: number of data array
  // Return: number of data transmitted. -1 is error.
  int8_t count = -1;
  // disable interrupt
  cli();
  // set simplex direction to transmit
  DIR_TXD;
  for(count = 0; count < numPacket; count++ )
  { // wait for data register is empty - bit 5 (UDRE) in reg A
    while(!bit_is_set(UCSR1A, UDRE1));
    // clear transmit complete flag
    UCSR1A |= 0x40;
    // send next byte
    UDR1 = pPacket[count];
  }
  // bit 6 is transmit complete
  while( !bit_is_set(UCSR1A,6) );
  // set simplex direction to receive
  DIR_RXD;
  // enable interrupt
  sei();
  // return number of bytes send
  return count;
}

/**
 * Recieving date
  \param *pPacket: data array pointer
  \param numPacket: number of data array
  \param Return: number of data recieved. -1 is error.
*/
int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
	int count, numgetbyte;

	if( gbDxlBufferHead == gbDxlBufferTail )
		return 0;

	numgetbyte = dxl_hal_get_qstate();
	if( numgetbyte > numPacket )
		numgetbyte = numPacket;

	for( count=0; count<numgetbyte; count++ )
		pPacket[count] = dxl_hal_get_queue();

	return numgetbyte;
}

// void dxl_hal_set_timeout( int NumRcvByte )
// {
// #define turnAroundTime_us 250
// 	// Start stop watch
// 	// NumRcvByte: number of recieving data(to calculate maximum waiting time)
// 	gwCountNum = 0;
// 	gwTimeoutCount = (NumRcvByte + 10) * (gfByteTransTime_us/counterUnit_us) + turnAroundTime_us/counterUnit_us;
// }

// int dxl_hal_timeout(void)
// {
// 	// Check timeout
// 	// Return: 0 is false, 1 is true(timeout occurred)
// 	// gwCountNum++; set by timer interrupt
//
// 	if( gwCountNum > gwTimeoutCount)
// 	{
// 		return 1;
// 	}
//
// //	_delay_us(gfByteTransTime_us);
// 	return 0;
// }

int dxl_hal_get_qstate(void)
{
	short NumByte;

	if( gbDxlBufferHead == gbDxlBufferTail )
		NumByte = 0;
	else if( gbDxlBufferHead < gbDxlBufferTail )
		NumByte = gbDxlBufferTail - gbDxlBufferHead;
	else
		NumByte = MAXNUM_DXLBUFF - (gbDxlBufferHead - gbDxlBufferTail);

	return (int)NumByte;
}

void dxl_hal_put_queue( unsigned char data )
{
	if( dxl_hal_get_qstate() == (MAXNUM_DXLBUFF-1) )
		return;

	gbDxlBuffer[gbDxlBufferTail] = data;

	if( gbDxlBufferTail == (MAXNUM_DXLBUFF-1) )
		gbDxlBufferTail = 0;
	else
		gbDxlBufferTail++;
}

unsigned char dxl_hal_get_queue(void)
{
	unsigned char data;

	if( gbDxlBufferHead == gbDxlBufferTail )
		return 0xff;

	data = gbDxlBuffer[gbDxlBufferHead];

	if( gbDxlBufferHead == (MAXNUM_DXLBUFF-1) )
		gbDxlBufferHead = 0;
	else
		gbDxlBufferHead++;

	return data;
}
/**
 * Interrupt routine for RX data - UART 1 (USART 0 is USB)*/
SIGNAL(USART1_RX_vect)
{ // got a byte - put into rx queue
  if (0)
  {
//    PORTD |= 0x80;
	dxl_hal_put_queue( UDR1 );
//    PORTD &= ~0x80;
  }
  else
  {
    // debug trying to speed up ISR
    // by not using call to dxl_hal_put_char(data)
    uint8_t NumBytes;
    if( gbDxlBufferHead == gbDxlBufferTail )
        NumBytes = 0;
    else if( gbDxlBufferHead < gbDxlBufferTail )
        NumBytes = gbDxlBufferTail - gbDxlBufferHead;
    else
        NumBytes = MAXNUM_DXLBUFF - (gbDxlBufferHead - gbDxlBufferTail);
    //
    if( NumBytes < (MAXNUM_DXLBUFF-1) )
    {
      gbDxlBuffer[gbDxlBufferTail] = UDR1;
        // test not needed, as tail is unsigned char
//       if( gbDxlBufferTail == (MAXNUM_DXLBUFF-1) )
//           gbDxlBufferTail = 0;
//       else
          gbDxlBufferTail++;
    }
    // debug end
  }
}