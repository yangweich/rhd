//##########################################################
//##                      Dynamixel.c	                  ##
//##         	Servo control utility functions           ##
//##                                   Version 2012.06.01 ##
//##########################################################
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//Function for clocking
uint64_t gfEndTime = 0;
#include "dxl_hal.h"
#include "dynamixel.h"

//Init Values
#define ID					(2)
#define LENGTH				(3)
#define INSTRUCTION			(4)
#define ERRBIT				(4)
#define PARAMETER			(5)
#define DEFAULT_BAUDNUMBER	(1)

unsigned char gbInstructionPacket[MAXNUM_TXPARAM+10] = {0};
unsigned char gbStatusPacket[MAXNUM_RXPARAM+10] = {0};
unsigned char gbRxPacketLength = 0;
unsigned char gbRxGetLength = 0;

int gbCommStatus = COMM_RXSUCCESS;
int giBusUsing = 0;

/**
 * \param device is devise string - must include ttyUSB and a number, only the number is used.
 * \param baudrate is actual baudrate that the servo is configured for e.g. 500000
 * \returns 1 on success */
int dxl_initialize_usb(const char * device, int baudrate)
{
  char * p1;
  int deviceIndex = 0;
  p1 = strstr(device, "ttyUSB");
  if (p1 != NULL)
    deviceIndex = strtol(p1+6, NULL, 0);
  else
    printf("Failed to find device %s (should be ttyUSB type)\n", device);
  if( dxl_hal_open(deviceIndex, baudrate) == 0 )
    return 0;

  gbCommStatus = COMM_RXSUCCESS;
  giBusUsing = 0;
  return 1;
}

int dxl_initialize(int deviceIndex, int baudnum )
{
	float baudrate;
	//calculate correct baudrate
	baudrate = 2000000.0f / (float)(baudnum + 1);

	if( dxl_hal_open(deviceIndex, baudrate) == 0 )
		return 0;

	gbCommStatus = COMM_RXSUCCESS;
	giBusUsing = 0;
	return 1;
}

void dxl_terminate(void)
{
	dxl_hal_close();
}

void dxl_tx_packet(void)
{
	unsigned char i;
	unsigned char TxNumByte, RealTxNumByte;
	unsigned char checksum = 0;

	if( giBusUsing == 1 )
		return;

	giBusUsing = 1;

	if( gbInstructionPacket[LENGTH] > (MAXNUM_TXPARAM+2) )
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}

	if( gbInstructionPacket[INSTRUCTION] != INST_PING
		&& gbInstructionPacket[INSTRUCTION] != INST_READ
		&& gbInstructionPacket[INSTRUCTION] != INST_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_REG_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_ACTION
		&& gbInstructionPacket[INSTRUCTION] != INST_RESET
		&& gbInstructionPacket[INSTRUCTION] != INST_SYNC_WRITE )
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}

	gbInstructionPacket[0] = 0xff;
	gbInstructionPacket[1] = 0xff;
	for( i=0; i<(gbInstructionPacket[LENGTH]+1); i++ )
		checksum += gbInstructionPacket[i+2];
	gbInstructionPacket[gbInstructionPacket[LENGTH]+3] = ~checksum;

	if( gbCommStatus == COMM_RXTIMEOUT || gbCommStatus == COMM_RXCORRUPT )
		dxl_hal_clear();

	TxNumByte = gbInstructionPacket[LENGTH] + 4;
	RealTxNumByte = dxl_hal_tx( (unsigned char*)gbInstructionPacket, TxNumByte );

	if( TxNumByte != RealTxNumByte )
	{
		gbCommStatus = COMM_TXFAIL;
		giBusUsing = 0;
		return;
	}

	//Time out set, at 57600 with parameter = 0, timeout = 210ms.
	if( gbInstructionPacket[INSTRUCTION] == INST_READ )
		dxl_hal_set_timeout( gbInstructionPacket[PARAMETER+1] +  6);
	else
		dxl_hal_set_timeout( 6 );

	gbCommStatus = COMM_TXSUCCESS;
}
void dxl_rx_packet(void)
{
	unsigned char i, j, nRead;
	unsigned char checksum = 0;

	if( giBusUsing == 0 )
		return;

	if( gbInstructionPacket[ID] == BROADCAST_ID )
	{
		gbCommStatus = COMM_RXSUCCESS;
		giBusUsing = 0;
		return;
	}

	if( gbCommStatus == COMM_TXSUCCESS )
	{
		//printf("transmit succes\n");
		gbRxGetLength = 0;
		gbRxPacketLength = 6;
	}
	/*  old implementation
	nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
	gbRxGetLength += nRead;
	if( gbRxGetLength < gbRxPacketLength )
	{
		if( dxl_hal_timeout() == 1 )//1
		{

			//printf("timed out\n");
			if(gbRxGetLength == 0)
				gbCommStatus = COMM_RXTIMEOUT;
			else
				gbCommStatus = COMM_RXCORRUPT;
			giBusUsing = 0;
			return;
		}
	}
    */

    /* *************************** */

    if(dxl_hal_poll() == 1)
    {
        //printf(" read\n");
        nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
        gbRxGetLength += nRead;
    }
    else
    {
//        printf("byte timeout\n");
    }

    if( gbRxGetLength < gbRxPacketLength )
    {
        if(dxl_hal_timeout() == 1)
        {
            if(gbRxGetLength == 0)
                gbCommStatus = COMM_RXTIMEOUT;
            else
                gbCommStatus = COMM_RXCORRUPT;

            giBusUsing = 0;
            return;
        }
    }



    /* ****************************** */


	// Find packet header
	for( i=0; i<(gbRxGetLength-1); i++ )
	{
		if( gbStatusPacket[i] == 0xff && gbStatusPacket[i+1] == 0xff )
		{
			break;
		}
		else if( i == gbRxGetLength-2 && gbStatusPacket[gbRxGetLength-1] == 0xff )
		{
			break;
		}
	}
	if( i > 0 )
	{
		for( j=0; j<(gbRxGetLength-i); j++ )
			gbStatusPacket[j] = gbStatusPacket[j + i];

		gbRxGetLength -= i;
	}

	if( gbRxGetLength < gbRxPacketLength )
	{
		gbCommStatus = COMM_RXWAITING;
		return;
	}

	// Check id pairing
	//printf("udsendt id %d, return id %d\n",gbInstructionPacket[ID],gbStatusPacket[ID]);
	if( gbInstructionPacket[ID] != gbStatusPacket[ID])
	{
		gbCommStatus = COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}

	gbRxPacketLength = gbStatusPacket[LENGTH] + 4;
	if( gbRxGetLength < gbRxPacketLength )
	{
		nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
		gbRxGetLength += nRead;
		if( gbRxGetLength < gbRxPacketLength )
		{
			gbCommStatus = COMM_RXWAITING;
			return;
		}
	}

	// Check checksum
	for( i=0; i<(gbStatusPacket[LENGTH]+1); i++ )
		checksum += gbStatusPacket[i+2];
	checksum = ~checksum;

	if( gbStatusPacket[gbStatusPacket[LENGTH]+3] != checksum )
	{
		gbCommStatus = COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}

	gbCommStatus = COMM_RXSUCCESS;
	giBusUsing = 0;
}

void dxl_txrx_packet(void)
{
	dxl_tx_packet();

	if( gbCommStatus != COMM_TXSUCCESS )
		return;

	do{
		dxl_rx_packet();
	}while( gbCommStatus == COMM_RXWAITING );
	//gfEndTime is made from dynamixel.c
	//printf("finished time = %llu",gfEndTime);
}

int dxl_get_result(void)
{
	return gbCommStatus;
}

void dxl_set_txpacket_id( int id )
{
	gbInstructionPacket[ID] = (unsigned char)id;
}

void dxl_set_txpacket_instruction( int instruction )
{
	gbInstructionPacket[INSTRUCTION] = (unsigned char)instruction;
}

void dxl_set_txpacket_parameter( int index, int value )
{
	gbInstructionPacket[PARAMETER+index] = (unsigned char)value;
}

void dxl_set_txpacket_length( int length )
{
	gbInstructionPacket[LENGTH] = (unsigned char)length;
}

int dxl_get_rxpacket_error( int errbit )
{
	if( gbStatusPacket[ERRBIT] & (unsigned char)errbit )
		return 1;

	return 0;
}
/**
 * get rx error byte */
unsigned char dxl_get_rxpacket_error_byte(void)
{
  return gbStatusPacket[ERRBIT];
}

int dxl_get_rxpacket_length(void)
{
	return (int)gbStatusPacket[LENGTH];
}

int dxl_get_rxpacket_parameter( int index )
{
	return (int)gbStatusPacket[PARAMETER+index];
}

int dxl_makeword( int lowbyte, int highbyte )
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

int dxl_get_lowbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff;
	return (int)temp;
}

int dxl_get_highbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff00;
	temp = temp >> 8;
	return (int)temp;
}

void dxl_ping( int id )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_PING;
	gbInstructionPacket[LENGTH] = 2;

	dxl_txrx_packet();
}

int dxl_read_byte( int id, int address )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 1;
	gbInstructionPacket[LENGTH] = 4;

	dxl_txrx_packet();

	return (int)gbStatusPacket[PARAMETER];
}
void dxl_reset( int id)
{
	while(giBusUsing);

		gbInstructionPacket[ID] = (unsigned char)id;
		gbInstructionPacket[INSTRUCTION] = INST_RESET;
		gbInstructionPacket[LENGTH] = 2;

		dxl_txrx_packet();

}
void dxl_write_byte( int id, int address, int value )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)value;
	gbInstructionPacket[LENGTH] = 4;

	dxl_txrx_packet();
}

int dxl_read_word( int id, int address )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 2;
	gbInstructionPacket[LENGTH] = 4;

	dxl_txrx_packet();

	return dxl_makeword((int)gbStatusPacket[PARAMETER], (int)gbStatusPacket[PARAMETER+1]);
}
//en test
/*int dxl_read_values( int id)
{
	int val[3];
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)42;
	gbInstructionPacket[PARAMETER+2] = 00;
	gbInstructionPacket[PARAMETER+3] = (unsigned char)43;
	gbInstructionPacket[PARAMETER+5] = 00;
	gbInstructionPacket[PARAMETER+6] = (unsigned char)46;
	gbInstructionPacket[LENGTH] = 2+4;

	dxl_txrx_packet();

	val[0] = (int)gbStatusPacket[PARAMETER];
	val[1] = (int)gbStatusPacket[PARAMETER+2];
	val[2] = (int)gbStatusPacket[PARAMETER+4];
	printf("%d, %d, %d, %d, %d, %d\n",val[0],(int)gbStatusPacket[PARAMETER+1],val[1],(int)gbStatusPacket[PARAMETER+3],val[2],(int)gbStatusPacket[PARAMETER+5]);
	printf("volt %d, temp %d, moveing %d",val[0],val[1],val[2]);
	return val;
}*/

void dxl_write_word( int id, int address, int value )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+2] = (unsigned char)dxl_get_highbyte(value);
	gbInstructionPacket[LENGTH] = 5;

	dxl_txrx_packet();
}
