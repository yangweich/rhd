#include <avr/io.h>
#include "dxl_hal.h"
#include "dynamixel.h"

#define MID                   (2)
#define LENGTH                (3)
#define INSTRUCTION           (4)
#define ERRBIT                (4)
#define PARAMETER             (5)
#define DEFAULT_BAUDNUMBER    (1)

unsigned char gbInstructionPacket[MAXNUM_TXPARAM + 10] = {0};
unsigned char gbStatusPacket[MAXNUM_RXPARAM + 10] = {0};
unsigned char gbRxPacketLength = 0;
unsigned char gbRxGetLength = 0;
int gbCommStatus = COMM_RXSUCCESS;
/// is bus to servo in use 1= in use 0= not used
int giBusUsing = 0;
volatile unsigned char gwRxTimeout;

uint16_t dxlErrTx = 0;
uint16_t dxlErrTimeout = 0;
uint16_t dxlErrCurrupt = 0;
uint16_t dxlErrSuccess = 0;
uint16_t dxlErrFail = 0;
uint16_t dxlRxTimeoutCnt = 0;
uint16_t dxlBusUseCnt = 0;
uint16_t dxlRxTimeoutCntMax = 0;
uint16_t dxlBusAlreadyInUseCnt = 0;
/**
 * Initialize dynamixel bus (rs485 or rs232
 * \param divisor is baudrate divisor
 * \param baudrate is baudrate - for delay calculation
 * \returns 1 */
uint8_t dxl_initialize(float baudrate )
{
  uint8_t divisor;
  divisor = F_CPU/8.0/baudrate - 0.5;
  dxl_hal_open( divisor, baudrate);
  gbCommStatus = COMM_RXSUCCESS;
  giBusUsing = 0;
  return divisor;
}

void dxl_terminate()
{
	dxl_hal_close();
}

///////////////////////////////////////////////////////////////////////
/** transmit a package
 * must leave the giBusUsing as 1 if transmit is successful
 */

void dxl_tx_packet()
{
	unsigned char i;
	unsigned char TxNumByte, RealTxNumByte;
	unsigned char checksum = 0;

	if( giBusUsing == 1 )
  {
    dxlBusAlreadyInUseCnt++;
    // we are in another cmmunication cycle, so stop
		return;
  }

	giBusUsing = 1;
  // this process is using the bus

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

// 	if( gbInstructionPacket[INSTRUCTION] == INST_READ )
// 		dxl_hal_set_timeout( gbInstructionPacket[PARAMETER+1] + 6 );
// 	else
// 		dxl_hal_set_timeout( 6 );

	gbCommStatus = COMM_TXSUCCESS;
  // so giBusUsing is left at 1,
  // so that rx_paxket knows
  // there is something in the queue
  // reset to 0 by next receive
}

/**
 * Receive a packet, as describet in gbInstructionPacket array */
void dxl_rx_packet()
{
  unsigned char i, j, nRead;
  unsigned char checksum = 0;

  if( giBusUsing == 0 )
    return;
  if( gbInstructionPacket[MID] == BROADCAST_ID )
  {
    gbCommStatus = COMM_RXSUCCESS;
    giBusUsing = 0;
    return;
  }
  if( gbCommStatus == COMM_TXSUCCESS )
  {
    gbRxGetLength = 0;
    gbRxPacketLength = 6;
  }
  PORTD |= 0x80;
  nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
  gbRxGetLength += nRead;
  if( gbRxGetLength < gbRxPacketLength )
  {
    if(gwRxTimeout) // dxl_hal_timeout() == 1 )
    {
      if(gbRxGetLength == 0)
        gbCommStatus = COMM_RXTIMEOUT;
      else
        gbCommStatus = COMM_RXCORRUPT;
      giBusUsing = 0;
      return;
    }
  }
  PORTD &= ~0x80;
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
  if( gbInstructionPacket[MID] != gbStatusPacket[MID])
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
  // finished with receive, so ready for another go
  giBusUsing = 0;
}

///////////////////////////////////////////////////////////////////////
/** transmit and receive a package */

void dxl_txrx_packet()
{
  gwRxTimeout = 0;
  // tx sets the giBusUse flag if
  // the bus is available and if the transmit is successful
  dxl_tx_packet();

  if( gbCommStatus != COMM_TXSUCCESS )
  {
    dxlErrTx++;
    return;
  }

  dxlRxTimeoutCnt=0;
  do
  { // wait until full packet or timeout
    PORTD |= 0x20;
    dxl_rx_packet();
    dxlRxTimeoutCnt++;
    PORTD &= ~0x20;
  } while( gbCommStatus == COMM_RXWAITING );
  // debug
  if (dxlRxTimeoutCnt > dxlRxTimeoutCntMax)
    dxlRxTimeoutCntMax = dxlRxTimeoutCnt;
  // debug end
  switch (gbCommStatus)
  {
    case COMM_RXCORRUPT:
      dxlErrTimeout++;
      break;
    case COMM_RXTIMEOUT:
      dxlErrCurrupt++;
      break;
    case COMM_RXSUCCESS:
      dxlErrSuccess++;
      break;
    case COMM_RXFAIL:
      dxlErrFail++;
      break;
    default:
      break;
  }
}

int dxl_get_result()
{
	return gbCommStatus;
}

const char * dxl_get_result_str(int val)
{
  switch(val)
  {
    case COMM_TXSUCCESS: return "TXSUCCESS"; break;
    case COMM_RXSUCCESS: return "RXSUCCESS"; break;
    case COMM_TXFAIL:    return "TXFAIL"; break;
    case COMM_RXFAIL:    return "RXFAIL"; break;
    case COMM_TXERROR:   return "TXERROR"; break;
    case COMM_RXWAITING: return "RXWAITING"; break;
    case COMM_RXTIMEOUT: return "RXTIMEOUT"; break;
    case COMM_RXCORRUPT: return "RXCORRUPT"; break;
    default: return "none"; break;
  }
}


void dxl_set_txpacket_id( int id )
{
	gbInstructionPacket[MID] = (unsigned char)id;
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

int dxl_get_rxpacket_length()
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
	while(giBusUsing) dxlBusUseCnt++;

	gbInstructionPacket[MID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_PING;
	gbInstructionPacket[LENGTH] = 2;

	dxl_txrx_packet();
}

int dxl_read_byte( int id, int address )
{
	while(giBusUsing) dxlBusUseCnt++;

	gbInstructionPacket[MID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 1;
	gbInstructionPacket[LENGTH] = 4;

	dxl_txrx_packet();

	return (int)gbStatusPacket[PARAMETER];
}

void dxl_write_byte( int id, int address, int value )
{
	while(giBusUsing) dxlBusUseCnt++;

	gbInstructionPacket[MID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)value;
	gbInstructionPacket[LENGTH] = 4;

	dxl_txrx_packet();
}

int dxl_read_word( int id, int address )
{
	while(giBusUsing) dxlBusUseCnt++;

	gbInstructionPacket[MID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 2;
	gbInstructionPacket[LENGTH] = 4;

	dxl_txrx_packet();

	return dxl_makeword((int)gbStatusPacket[PARAMETER], (int)gbStatusPacket[PARAMETER+1]);
}

void dxl_write_word( int id, int address, int value )
{
	while(giBusUsing) dxlBusUseCnt++;

	gbInstructionPacket[MID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+2] = (unsigned char)dxl_get_highbyte(value);
	gbInstructionPacket[LENGTH] = 5;

	dxl_txrx_packet();
}
