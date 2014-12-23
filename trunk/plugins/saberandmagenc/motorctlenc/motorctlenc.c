#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/parity.h>
#include <math.h>
#include "motorctlenc.h"

//================================================================
//Define Global Variables
//================================================================


// value logging
uint16_t stepCnt = 0;
uint8_t logMotor = 0;
// #define STEP_BUF_SIZE 3
// int16_t stepA[STEP_BUF_SIZE];
// int16_t stepB[STEP_BUF_SIZE];
// int16_t stepC[STEP_BUF_SIZE];
// int16_t stepD[STEP_BUF_SIZE];
const char * lfcr = "\n\r";

// serial communication
#define RX_BUFF_SIZE 23
char rxbuff[2][RX_BUFF_SIZE];
uint8_t rxbuffCnt = 0;
volatile uint8_t rxbuffN = 0;
char * rxBuffp = rxbuff[0];
uint16_t rxCnt = 0;
uint16_t i2cCnt = 0;
uint16_t timer1Cnt = 0;
uint16_t timer2Cnt = 0, timer1Cnta, timer1Cntb, timer1Cntc, timer1Cntd, timer1Cnte;


int8_t ctrlNCnt;
uint16_t ctlCnt[2] = {0,0}; //control cycle counter
uint16_t statusEvery = 30; // send status every this often

//int16_t chirpTA = 50, chirpT, chirpT1, chirpT2, chirpT3, chirpRef; // chirp timer
//int8_t sinus[20] = { 0, 39, 75, 103, 122, 127, 122, 103, 75, 39, 0, -39, -75, -103, -122, -128, -122, -103, -75, -39};
//int8_t sinus[40] = { 0, 20, 39, 58, 75, 90, 103, 114, 122, 125, 127, 125, 122, 114, 103, 90, 75, 58, 39, 20, 
//                     0,-20,-39,-58,-75,-90,-103,-114,-122,-125,-128,-125,-122,-114,-103,-90,-75,-58,-39};
//int8_t sinus[32]; // = {  0, 25, 49, 71, 90, 106, 118, 125, 127, 125, 118, 106, 90, 71, 49, 25,
           //   0,-25,-49,-71,-90,-106,-118,-125,-128,-125,-118,-106,-90,-71,-49,-25};
//int16_t chirpW;
//int16_t chirpPwm;

// Encoder
#define EBL 5
// NB number of buffes fixed to 4 - change require code change too.
#define EBL_N 4
uint8_t encBuf[EBL_N][EBL];
uint8_t encBufCnt = 0;
uint8_t encBufN = 0;
uint8_t encBufOK[EBL_N] = {0,0,0,0};
uint16_t encCnt = 0;
#define ENC_MEDIAN_VALUES 5
uint16_t encLlast, encRlast;
uint8_t  encLadd = 0, encRadd = 0;
uint16_t encL, encR, encLRaw[ENC_MEDIAN_VALUES], encRRaw[ENC_MEDIAN_VALUES];
uint16_t encTmp[ENC_MEDIAN_VALUES];
uint8_t /*encL_OCF, encL_COF, encL_LIN,*/ encL_MAG, encLhold; //, encL_PAR;
uint8_t /*encR_OCF, encR_COF, encR_LIN,*/ encR_MAG, encRhold; //, encR_PAR;
//uint16_t encLParityErrCnt = 0, encRParityErrCnt = 0;

// timer state
uint8_t timerState = 0;
uint8_t encCycleRead = 0;
uint8_t delayCnt;
uint16_t timerStateCnt = 0;
// timer A clock (CLK_IO / prescaler)
#define TIMER_A_CLOCK (16/1)
// encoder read time in us * CLK_IO frequency in MHz
#define ENCODER_READ_TIME_US_N 390
#define ENCODER_READ_CNT_N (ENCODER_READ_TIME_US_N * TIMER_A_CLOCK)
#define CYCLE_TIME_US 10000
int16_t remaining_cycleTime;
char s2[4] = "ab\0"; // debug
int8_t  remaining_cycles;
char s3[4] = "12\0"; // debug

// motor control
uint16_t motorLeftPWM = 1500; // in us
uint16_t motorRightPWM = 1500;
uint16_t motorLeftPw  = 1500 * TIMER_A_CLOCK; // approx 1500us
uint16_t motorRightPw = 1500 * TIMER_A_CLOCK; // approx 1500us
int8_t  offsetL = 0, offsetR = 0; // offset to get zero velocity -50..50
// string buffer - may not be used in interrupt
#define MSTL 76
char s[MSTL];

// velocity (calculated for 10ms period)
#define MAX_ENC_VEL 2000
// timer count for 10ms period (timer 2 prescaler 1024 and 16MHz)
#define TIMER_CNT_10MS 156
uint8_t cycleValueCnt = 0; // for magnetic encoder read timing
//char s2[4] = "abc\0"; // debug
int8_t velCalcTime = 0; // for timer 2 overflow count
//char s3[4] = "123\0"; // debug
uint16_t cycleLastEncL = 0;
uint16_t cycleLastEncR = 0;
int16_t velLenc = 0, velRenc = 0; // measured velocity (in encoder tics each 10ms)
int16_t velLencErr = 0, velRencErr = 0; // velocity error - encoder value excessive
// pid controller values
int16_t velCLenc = 0, velCRenc = 0; // velocity command
int16_t velELenc = 0, velERenc = 0; // velocity error
// control
uint8_t velCtrl = 0; // control mode 2=ff-lag-p, 1=control velocity, 0=not
int16_t velErrL = 0; // last error in velocity for left wheel
int16_t velErrR = 0; // last error in velocity for left wheel
int16_t velCtlL = 0; // last control value (left)
int16_t velCtlR = 0; // last control value (right)
/// velCKFF maximum is 8*16 to avoid overflow for signed 16 bit
#define VEL_CTL_KFF_MAX 128
/// velCKP maximum is 64 to avoid overflow of signed 16 bit
#define VEL_CTL_KP_MAX 64
/// velocity feed forward control value - 16 should be about right
int8_t velCKFF = 16;
/// proportional error gain
int8_t velCKP = 1;

void calculateVelocity(void);
void doVelControl(void);
void setMotorVelocity(int16_t velLeft, int16_t velRight);
uint8_t isANumber(unsigned char ch);
int16_t calcEncVelocity(uint16_t encNow, uint16_t encLast, int16_t timeCnt);
void sendStringLn(const char * str);
uint16_t median(uint16_t * val);


//================================================================
//Interrupt Routines
//================================================================
/**
 * UART 0 received a byte interrupt - command input */
ISR(USART_RX_vect)
{  // Code to be executed when the USART receives a byte here
  *rxBuffp = UDR0;
  if (*rxBuffp < ' ')
  {  // line finished switch buffer
    *rxBuffp = '\0';
    if (rxbuffN > 0)
      rxbuffN--;
    else
      rxbuffN++;
    rxBuffp = rxbuff[rxbuffN];
    *rxBuffp = '\0';
    rxbuffCnt = 0;
  }
  else if (rxbuffCnt < RX_BUFF_SIZE - 2)
  { // space for more data
    *(++rxBuffp) = '\0';
    rxbuffCnt++;
  }
  rxCnt++;
}

/**
 * SPI data transfer complete interrupt (8 bit) */
ISR(SPI_STC_vect)
{ // save data from AS5045 magnetic encoder
  encBuf[encBufN][encBufCnt] = SPDR;
  encBufCnt++;
  if (encBufCnt < EBL)
  { // not all data received
    // start transfer of next byte
    SPDR = 0x3C;
  }
  else
  { // stop read
    PORTB |= (1 << PORTB2); // 0xfb;  // stop SPI, set SS to high
    // filled buffer is valid
    encBufOK[encBufN] = 1;
    encBufN = (encBufN + 1) & 0x3;
    // new buffer is now invalid and empty
    encBufOK[encBufN] = 0;
    encBufCnt = 0;
  }
  i2cCnt++;
}

ISR(TIMER2_OVF_vect)
{ // overflow if 16.38 ms has pased and no velocity calculation
  // (assumes prescaler of 1024 and 16 MHz)
  velCalcTime++;
  timer2Cnt++;
}


ISR(TIMER1_COMPA_vect)
{
  timer1Cnt++;
  switch (timerState)
  {
  case 1: // motor 1 start (right)
    PORTD |= (1 << 4); // start motor 1 pulse, on pin 4 on port D (S1 in motor ctrl)
    OCR1A = motorRightPw; // Motor 1 pulse width (timer value)
    timerStateCnt++;
    timer1Cnta = TCNT2;
    break;
  case 2: // motor 2 start (left)
    PORTD &= ~(1 << 4); // stop motor 1 pulse
    PORTD |= (1 << 5); // start motor 2 pulse, on pin 5 on port D (S2 in motor ctrl)
    OCR1A = motorLeftPw; // Motor 1 pulse width (timer value)
    timer1Cntb = TCNT2;
    break;
  case 3: // encoder read start
    PORTD &= ~(1 << 5); // stop motor 2 pulse
    // calculate remaining cycle time - leave 0.5ms for pre-motor control
//     remaining_cycleTime = CYCLE_TIME_US - 500 -
//                           (motorLeftPWM /*/ TIMER_A_CLOCK*/) -
//                           (motorRightPWM /*/ TIMER_A_CLOCK*/);
    // remaining_cycles = remaining_cycleTime / ENCODER_READ_TIME_US_N;
    remaining_cycles = 5; //- should give 5 reads
    // wait a little longer first time to reach
    // a fixed cycle time
    OCR1A = ENCODER_READ_CNT_N /*+
            (remaining_cycleTime - remaining_cycles * ENCODER_READ_TIME_US_N) *
            TIMER_A_CLOCK*/;
    PORTB &= ~(1 << PORTB2); // 0xfb;  // start SPI, set SS to low
    PORTB &= ~(1 << PORTB2); // wait at least 0.5 us
    PORTB &= ~(1 << PORTB2); // wait at least 0.5 us
    SPDR = 0x3C; // start SPI read
    encCycleRead = 0; // for measurements in one cycle (up to 5x4 = 20)
    delayCnt = 1; // for reads in one cycle (approx 20 - depends on motor puls width)
    //cycleValueCnt = 0; // full read cycle - up to 4 in one cycle
    timer1Cntc = TCNT2;
    break;
  case 4: // encoder read
    OCR1A = ENCODER_READ_CNT_N; // ~400us sample rate for encoder 2kHz
    if (encBufCnt == 0)
    {  // start new read
      PORTB &= ~(1 << PORTB2); // 0xfb;  // start SPI, set SS to low
      PORTB &= ~(1 << PORTB2); // wait at least 0.5 us
      PORTB &= ~(1 << PORTB2); // wait at least 0.5 us
      SPDR = 0x3C;
    }
    delayCnt++;
    if (delayCnt < (remaining_cycles))
//    if (encCycleRead < (ENC_MEDIAN_VALUES - 1))
      // stay in this read mode for (5) reads
      timerState--;
    timer1Cntd = TCNT2;
    break;
//    case 5: // wait further before next cycle
//      delayCnt++;
//      if (delayCnt < remaining_cycles)
//        timerState--;
//      break;
  default: // finished - restart
    timerState = 0;
    OCR1A = 100 * TIMER_A_CLOCK; // ~1ms wait after encoder read
    timer1Cnte = TCNT2;
    break;
  }
  timerState++;
}

/**
 * Decode message from 2 AS5045 encoders received in one buffer (encBuf)
 * encoder value is stored in a arw encoder value buffer encXRaw, to
 * allow filtering of stary error values (buffer with 5 values).
 * OCF when 1: offset compensation finished is good
 * COF Cordic overflow - when 1:value is form last good measurement
 * LIN liniarity is bad, when 1: magnet alignmenet out of order
 * MAG when 0 magnets are good and stable, 1 is increase, 2 is decrease 3 is bad.
 * PAR is parity, when 1 parity is OK.
 * conts number of parity errors */
void encoderMsgDecode(uint8_t * buf)
{ // decode bytes from AS5045 magnetic encoder
  // left status bits
//   encL_OCF = (buf[1] >> 2) & 0x01;
//   encL_COF = (buf[1] >> 1) & 0x01;
//   encL_LIN =  buf[1] & 0x01;
//   encL_MAG =  buf[2] >> 6;
  if (encLhold == 0)
  { // status
    // bit 5: !COF
    encL_MAG = (((~buf[1] & 0x04) + (buf[1] & 0x03)) << 2) + (buf[2] >> 6);
    if (encL_MAG != 0)
      encLhold = 10;
  }
  if (encLhold > 0)
    encLhold--;
//   encL_PAR = parity_even_bit(
//                parity_even_bit(buf[0] << 1) +
//                parity_even_bit(buf[1]) +
//                parity_even_bit(buf[2] >> 4));
  // right status bits
//   encR_OCF = (buf[4] >> 7) & 0x01;
//   encR_COF = (buf[4] >> 6) & 0x01;
//   encR_LIN = (buf[4] >> 5) & 0x01;
//   encR_MAG = (buf[4] >> 3) & 0x03;
  if (encRhold == 0)
  { // noerror last time
    encR_MAG = (((~buf[4] & 0x80) + (buf[4] & 0x78)) >> 3); 
    if (encR_MAG != 0)
      encRhold = 10;
  }
  if (encRhold > 0)
    encRhold--;
//   encR_PAR = parity_even_bit(
//                parity_even_bit(buf[2] << 4) +
//                parity_even_bit(buf[3]) +
//                parity_even_bit(buf[4] >> 2));
//   if ((encL_PAR == 1) && (encR_PAR == 1))
  { // both are good, so extract encoder values
    uint16_t enc;
    int16_t di;
    // left encoder
    enc = ((buf[0] & 0x7f) << 5) + (buf[1] >> 3);
    if (enc > encLlast)
    { // check if added 3 bits need change
      di = enc - encLlast;
      if (di > (1 << 11))
        // reverse movement crossing 12 bit limit
        encLadd = (encLadd - 1) & 0x07;
    }
    else
    {
      di = encLlast - enc;
      if (di > (1 << 11))
        // forward movement crossing 12 bit limit
        encLadd = (encLadd + 1) & 0x07;
    }
    encLlast = enc;
    encLRaw[encCycleRead] = (encLadd << 12) + enc;
    // right encoder
    enc = ((buf[2] & 0xf) << 8) + buf[3];
    if (enc > encRlast)
    { // check if added 3 bits need change
      di = enc - encRlast;
      if (di > (1 << 11))
        // reverse movement crossing 12 bit limit
        encRadd = (encRadd - 1) & 0x07;
    }
    else
    {
      di = encRlast - enc;
      if (di > (1 << 11))
        // forward movement crossing 12 bit limit
        encRadd = (encRadd + 1) & 0x07;
    }
    encRRaw[encCycleRead] = (encRadd << 12) + enc;
    encRlast = enc;
    if (encCycleRead < ENC_MEDIAN_VALUES)
      encCycleRead++;
  }
//   else
//   {
//     if (encL_PAR != 1)
//       encLParityErrCnt++;
//     if (encR_PAR != 1)
//       encRParityErrCnt++;
//   }
}


/**
 * ADC interrupt routine */
ISR(ADC_vect)
{
  // start new conversion
  //ADCSRA = AD_START_CONV;
  sei();
}



/**
 * Send an integer with an explanation
 * \param pre is some text to send before the integer value
 * \param value is the integer value to send (6 chars reserver - decomal notation
 * \param post is some final characters - i.e. \n\r */
// void sendInt2(const char * pre, int value1, int value2, const char * post, int fmt)
// {
//   const int MSL = 20;
//   char s[MSL];
//   sendString(pre);
//   switch (fmt)
//   { // unsigned values
//     case 1: snprintf(s, MSL, "%1u %1u", value1, value2); break;
// //    case 2: snprintf(s, MSL, "%2u %2u", value1, value2); break;
// //    case 3: snprintf(s, MSL, "%3u %3u", value1, value2); break;
// //    case 4: snprintf(s, MSL, "%4u %4u", value1, value2); break;
// //    case 5: snprintf(s, MSL, "%5u %5u", value1, value2); break;
// //    case 6: snprintf(s, MSL, "%6u %6u", value1, value2); break;
//     case 8: snprintf(s, MSL, "%2x %2x", value1, value2); break;
//     case 16: snprintf(s, MSL, "%4x %4x", value1, value2); break;
//     // signed values
// //    case -1: snprintf(s, MSL, "%1d %1d", value1, value2); break;
// //    case -2: snprintf(s, MSL, "%2d %2d", value1, value2); break;
// //    case -3: snprintf(s, MSL, "%3d %3d", value1, value2); break;
//     case -4: snprintf(s, MSL, "%4d %4d", value1, value2); break;
//     case -5: snprintf(s, MSL, "%5d %5d", value1, value2); break;
//     case -6: snprintf(s, MSL, "%6d %6d", value1, value2); break;
//     // default is flexible
//     default:
// //      snprintf(s, MSL, "%d %d", value1, value2);
//       break;
//   };
//   sendString(s);
//   if (post != NULL)
//     sendString(post);
// }

/**
 * Send an integer with an explanation
 * \param pre is some text to send before the integer value
 * \param value is the integer value to send (6 chars reserver - decomal notation
 * */
void sendInt1(const char * pre, int value, int fmt)
{
//   char s[10];
  sendString(pre);
  switch (fmt)
  { // unsigned values
    //case 1: snprintf(s, 10, "%1u", value); break;
    case 2: snprintf(s, 10, "%2u", value); break;
    //case 3: snprintf(s, 10, "%3u", value); break;
    //case 4: snprintf(s, 10, "%4u", value); break;
    case 5: snprintf(s, 10, "%5u", value); break;
    //case 6: snprintf(s, 10, "%6u", value); break;
    // signed values
    //case -1: snprintf(s, 10, "%1d", value); break;
    //case -2: snprintf(s, 10, "%2d", value); break;
    //case -3: snprintf(s, 10, "%3d", value); break;
    case -4: snprintf(s, 10, "%4d", value); break;
    //case -5: snprintf(s, 10, "%5d", value); break;
    case -6: snprintf(s, 10, "%6d", value); break;
    // default is flexible
    default:
//      snprintf(s, 10, "%d", value);
      break;
  };
  sendString(s);
}

/**
 * Write string to serial port
 * waits for last character to be send.
 * \param c is the charracter to send */
inline void serial_write(uint8_t c)
{
  while ( !(UCSR0A & (1 << UDRE0)) )
     ;
  UDR0 = c;
}

/**
 * Send status of control mode */
void sendCtrlStatus(void)
{
//   const int MSL = 20;
//   char s[MSL];  
  snprintf(s, MSTL, "E%x,%x", encL, encR);
  sendString(s);
  snprintf(s, MSTL, "V%d,%d", velLenc, velRenc);
  sendString(s);
  snprintf(s, MSTL, "M%u,%u", encL_MAG, encR_MAG);
  sendString(s);
  snprintf(s, MSTL, "S%d,%d", (PIND & 0x80) != 0, (PIND & 0x40) != 0);
  sendString(s);
  snprintf(s, MSTL, "W%d,%d", motorLeftPWM, motorRightPWM);
  sendString(s);
  snprintf(s, MSTL, "#%u %d", remaining_cycleTime, remaining_cycles);
  sendStringLn(s);
}
 
/**
 * Send string - and wait until send
 * \param str is the string to send */
void sendString(const char * str)
{
  int i = 0;
  while (1)
  { // send one char at a time until no more
    if (str[i] == '\0')
      break;
    serial_write(str[i++]);
  }
}

/**
 * Send string terminate with newline carrige-return \n\r - and wait until send
 * \param str is the string to send */
// void sendStringLn(const char * str)
// {
//   int i = 0;
//   while (1)
//   { // send one char at a time until no more
//     if (str[i] == '\0')
//       break;
//     serial_write(str[i++]);
//   }
//   serial_write('\n');
//   serial_write('\r');
// }

///////////////////////////////////


/////////////////////////////////////


void sendHelp(void)
{
  // normal help
  sendStringLn("# status/help for SaMe");
  snprintf(s , MSTL, "w=%d,%d # RC-PW -500..500 L,R",
                       1500 - motorLeftPw/TIMER_A_CLOCK,
                       1500 - motorRightPw/TIMER_A_CLOCK);
  sendStringLn(s);
  snprintf(s, MSTL, "v=%d,%d # vel L,R ref -2000..+2000 (enc/10ms)", velCLenc, velCRenc);
  sendStringLn(s);
  snprintf(s, MSTL, "p=%d # vel control 0=not", velCtrl);
  sendStringLn(s);
  snprintf(s, MSTL, "kf=%d # vel feed-fwd, MAX %d", velCKFF, VEL_CTL_KFF_MAX);
  sendStringLn(s);
  snprintf(s, MSTL, "kp=%d # vel prop-gain, MAX %d", velCKP, VEL_CTL_KP_MAX);
  sendStringLn(s);
//   snprintf(s, MSL, "c=%d,%d # chirp Max,ref value", chirpTA, chirpT3);
//   sendStringLn(s);
  snprintf(s, MSTL, "l=N # List control status MAX %d", 255);
  sendStringLn(s);
  snprintf(s, MSTL, "g=%d # status every x10ms", statusEvery);
  sendStringLn(s);
  snprintf(s, MSTL, "o=%d,%d # RC-PWM-Offset L,R -50..50", offsetL, offsetR);
  sendStringLn(s);
//   snprintf(s, MSTL, "s=%d,%d # log cnt,MAX", stepCnt, STEP_BUF_SIZE);
//   sendStringLn(s);
//   snprintf(s, MSTL,  "h # help");
//   sendStringLn(s);
//  sendStringLn("# status: E encoder, V encoder/10ms, M mag-status, S start,emerg sw");
};


//==================================================
//Core functions
//==================================================
//Function: ioinit
//Purpose:      Initialize AVR I/O, UART and Interrupts
//Inputs:       None
//Outputs:      None
void ioinit(void)
{
  // SPI setup
  //1 = output, 0 = input
  DDRB = (1 << PORTB2) |(1<<MOSI)|(1<<SCK);//Enable SS and SPI pins as outputs
  PORTB |= (1<<MISO);                       //Enable pull-up on MISO pin
  PORTB |= (1 << PORTB2); // set SS to 1 (disable SPI slave)
  // SPI interrupt, SPI as master, set SPI clock rate fck/16
  SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << SPR1);
  SPSR = 0; // not double speed (bit 0 writable only)
  
  //DDRC = 0xFF;  //Initialize Port C to all inputs
  //PORTC = 0xFF;       //Activate all pull-ups on Port C

  DDRD = 0xFF;  //Set Port D to all outputs
  DDRD &= ~(1<<RX);     //Set bit 0(Rx) to be an input
  PORTD = (1<<RX);      //Enable the pull-up on Rx
  // set start D6 and stop D7 as input 
  DDRD &= ~(0xC0); // set bit D6 and D7 as input (uses extern pull up)

  // USART Baud rate: 115200 (With 16 MHz Clock)
  UBRR0H = (MYUBRR >> 8) & 0x7F;   //Make sure highest bit(URSEL) is 0 indicating we are writing to UBRRH
  UBRR0L = MYUBRR;
  //Double the UART Speed
  UCSR0A = (1 << U2X0);                           
  //Enable Rx and Tx in UART and Rx interrupt
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); 
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);  //8-Bit Characters
  //  stdout = &mystdout; //Required for printf init

  //
  // Init timer 2
  //Set Prescaler to 8. (Timer Frequency set to 16Mhz)
  //Used for time calculation
  TCCR2A = 0;   // no PWM, no output
  TCCR2B = 0x7; // Divde 16MHz clock by 1024 to give 64us each count (max is then 16.3 ms)
  TCNT2  = 0;   // reset counter to 0
  TIMSK2 = (1 << TOIE2); // timer overflow interrupt (16 ms)

  //Init Timer 1 for CTC mode
   TCCR1A = 0; //Set no output pin effect
   TCCR1B = (1 << WGM12) | (1 << CS10);  //Set CTC mode, clk_IO/1 prescaler
   OCR1A = 0xfffe; // initial value
   TIMSK1 = (1 << OCIE1A); // on compare A (timer reset)

  //Initialize the ADC for Reads to control motor shut-down in case of over current
//   ADMUX = 0;    //Set the ADC channel to 0.
//   ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Enable the ADC, auto-triggering, the ADC Interrupt and set the ADC prescaler to 64                                                                                                                                    //ADC clock = 250kHz
//   ADCSRA |= (1<<ADSC);  //Start the first conversion

}

///////////////////////////////////////////////////

void sendStringLn(const char * str)
{
  sendString(str);
  sendString(lfcr);
}

void sendStringN2V(void)
{
  sendStringLn("# error: not 2 values");
}

//////////////////////////////////////////////////

int main(void)
{
  int8_t n = 0, v;
  int cnt = 0;
  int rxN = 0;
  int16_t  s16, t16 = 0;
  uint16_t encCntLast = 0;
  // zero command rx buffer
  rxbuff[0][0] = '\0';
  rxbuff[1][0] = '\0';
  //Initialize AVR I/O, UART and Interrupts
  ioinit();
  // start greating
  sendHelp();
  // enable interrupt
  sei();
  while(1)
  { // run main loop
    uint8_t encb = (encBufN - 1) & 0x3;
    // encoder PCI interface
    if (encBufOK[encb])
    { // new encoder message is available
      encoderMsgDecode(encBuf[encb]);
      encBufOK[encb] = 0; // buffer used
      if (encCycleRead >= ENC_MEDIAN_VALUES)
      { // enough data for filtered value
//         switch(cycleValueCnt)
//         {
//           case 0:
            encL = median(encLRaw);
            encR = median(encRRaw);
            encCnt++;
//             break;
//           case 2:
            calculateVelocity();
            // shift pin (d8 on arduino nano) to test update rate
            PORTB ^= 0x1;
//             break;
//           case 3:
            // do control in step two (of 4-5 steps each 10ms cycle)
            if (((PIND & 0x80) != 0) && ((PIND & 0x40) == 0))
            { // drive allowed
              if (velCtrl)
                doVelControl();
            }
            else
              // stop
              setMotorVelocity(0, 0);
//             break;
//           case 4:
            if ((statusEvery > 0) && ((encCnt % statusEvery) == 0))
              sendCtrlStatus();
//             break;
//           default:
//             break;
//         }
        // increase step counter
        cycleValueCnt++;
      }
    }
//     else
//       // make main loop idle signal
//       PORTB ^= 0x2;
    // get new data
    if (rxN != rxbuffN)
    {
      n = 0;
      char * cp = rxbuff[rxN], *c2;
      // restart logging
      // int16_t sinus8[32] = {  0, 25, 49, 71, 90, 106, 118, 125, 127, 125, 118, 106, 90, 71, 49, 25,
      //        0,-25,-49,-71,-90,-106,-118,-125,-128,-125,-118,-106,-90,-71,-49,-25};
      //
      switch (*cp++)
      {
        case 'w': // motor pw
          // sets motor control directly
          // range -500 (reverse) to +500 (forward)
          s16 = strtol(cp, &c2, 0);
          if (cp != c2)
          {
            cp = c2;
            t16 = strtol(cp, &c2, 0);
          }
          if (cp != c2)
          { // test for emergency stop
            if (((PIND & 0x80) != 0) && ((PIND & 0x40) == 0))
              setMotorVelocity(s16, t16);
          }
          else
            sendStringN2V(); //"PWM - not 2 values");
          n = -1;
          break;
        case 'o': // motor pwm offset
          // sets motor control directly
          // range -500 (reverse) to +500 (forward)
          v = strtol(cp, &c2, 10);
          if (cp != c2)
          {
            cp = c2;
            offsetL = v;
            offsetR = strtol(cp, &c2, 10);
          }
          if (cp == c2)
            sendStringN2V(); //Ln("offset - not 2 values");
          n = 0;
          break;
        case 'v': // motor velocity encoder tics/10ms
          // about +/- 2000
          s16 = strtol(cp, &c2, 10);
          if (cp != c2)
          {
            cp = c2;
            t16 = strtol(cp, &c2, 0);
          }
          if (cp != c2)
          {
            velCLenc = s16;
            velCRenc = t16;
          }
          else
            sendStringN2V();
          n = -1;
          break;
        case 'p': // motor vel ctrl 0=no ctrl, 1=ctrl
          velCtrl = strtol(cp, NULL, 0);
          if (velCtrl == 0)
            setMotorVelocity(0,0);
          n = 0;
          break;
        case 'k': // speed controller konstants
          switch (*cp++)
          {
            case 'p':
              velCKP = strtol(cp, NULL, 0);
              break;
            case 'f':
              velCKFF = strtol(cp, NULL, 0);
              break;
            default:
              break;
          }
          n = 0;
          break;
        case 'l': // list fast status (n times)
          if (*cp >= '0')
            n = strtol(cp, NULL, 0);
          else
            n = 3;
          n++;
          break;
//         case 'c': // chirp
//           // start chirp with a count 20 => ca 1Hz as lowest frq
//           chirpTA = strtol(cp, NULL, 0);
//           for (int i = 0; i < 32; i++)
//           {
//             int16_t a = (sinus8[i] * chirpTA) >> 8;
//             sinus[i] = a;
//             //sendInt2("sinus", sinus8[i], a, "\n\r", -4);
//           }
//           // change around this value
//           chirpRef = 0; // posRef[0];
//           chirpT = 70; // 70 * 0.4ms between phase shift at lowest frq
//           chirpT1 = 0;
//           chirpT2 = chirpT;
//           chirpT3 = 6;
//           stepCnt = 0;
//           break;
//         case 'd': // deadband (minimum PWM8 if vel != 0)
//           if (*cp >= '0')
//             deadband = strtol(cp, NULL, 0);
//           break;
        case 'g': // start print regular
          if (*cp < '0')
            statusEvery = 1; // start status every time
          else
            statusEvery = strtol(cp, NULL, 0); // 0 is stop status
          n = 1;
          break;
//         case 's': // dump values and stop
//           sendStringLn("sample current forceval fctrl PWMused)");
//           for (int i = 0; i < STEP_BUF_SIZE; i++)
//           {
//             sendInt1(" ", i, 5);
//             sendInt1("; ", stepA[i], -4);
//             sendInt1("; ", stepB[i], -4);
//             sendInt1("; ", stepC[i], -4);
//             sendInt1("; ", stepD[i], -4);
//             sendString(lfcr);
//           }
//           n = 1;
//           break;
        default:
          break;
      }
      if (n == 0)
        sendHelp();
      //n = 0;
      rxN = rxbuffN;
    }
    if ((n > 1) && (encCntLast != encCnt))
    {
      sendCtrlStatus();
      encCntLast = encCnt;
      n--;
    }
    cnt++;
  }
  return (0);
}



/**
 * sort values and return median
 * Sorts ENC_MEDIAN_VALUES values */
uint16_t median(uint16_t * val)
{
   uint8_t i, n, m;
   for (n = 0; n < ENC_MEDIAN_VALUES; n++)
   {
     if (n == 0)
       encTmp[0] = val[0];
     else
     {
       i = n;
       m = n;
       // if next value is smaller, then put it earlier
       while ((i > 0) && (val[n] < encTmp[i - 1]))
         i--;
       for (m = n; m > i; m--)
         encTmp[m] = encTmp[m - 1];
       encTmp[i] = val[n];
     }
   }
  //return median value
  return encTmp[ENC_MEDIAN_VALUES >> 1];
//  return val[ENC_MEDIAN_VALUES/2];
}

/**
 * Calculate velocity based on last encoder values,
 * velocity is encoder tics in one sample time,
 * for left and right motor (wheel) */
void calculateVelocity(void)
{
  int16_t vel;
  int16_t timeCnt; // in units of 64us
  // if more than 16ms, then velValcTime is increased, else
  // vel time is taken from timer 2 - prescaler 1024 and 16MHz clock => 64us/count
  timeCnt = velCalcTime * 256 + TCNT2;
  // reset time for next calculation
  TCNT2 = 0;
  velCalcTime = 0; // 16.384 ms per count
  //
  // calculate velocity for each wheel
  vel =  calcEncVelocity(encL, cycleLastEncL, timeCnt);
  if (vel < MAX_ENC_VEL && vel > -MAX_ENC_VEL)
    // no detectable encoder error
    velLenc = vel;
  else
    velLencErr++;
  vel = -calcEncVelocity(encR, cycleLastEncR, timeCnt);
  if (vel < MAX_ENC_VEL && vel > -MAX_ENC_VEL)
    // no detectable encoder error
    velRenc = vel;
  else
    velRencErr++;
  // timer 1 counts time since last velocity calculation (max 16 ms)
  //velCalcTime = 0;
  // save last
  cycleLastEncL = encL;
  cycleLastEncR = encR;
}

/**
 * Do velocity control */
void doVelControl(void)
{ // proportional and feed forward control
/// ERR_MAX is max used speed error for PID control
#define ERR_MAX 500
  int16_t vErr;
  int16_t vFF;  // feed forward control value
  int16_t vP;   // proportional control value
  int16_t mPWL, mPWR; // -500..+500 change to pulse width relative to 1500 us
  // left motor
  // commanded vel is in encoder tics per 10ms, i.e. -2000..2000
  vFF = (velCLenc / 8) * velCKFF;
  vErr = velCLenc - velLenc;
  if (vErr > ERR_MAX)
    vErr = ERR_MAX;
  else if (vErr < -ERR_MAX)
    vErr = -ERR_MAX;
  if (velCtrl == 1)
  { // p-ff controller
    vP = vErr * velCKP / 2;
    mPWL = (vP + vFF) / 8;
  }
  else
  { // lag-ff controller (one pole one zero)
    vP = (velCKP * (vErr - velErrL / 5)) / 32;
    mPWL = (velCtlL * 10)/9 + vP;
    velCtlL = mPWL;
    mPWL += vFF / 8;
  }
  velErrL = vErr;
  // right motor
  vFF = (velCRenc / 8) * velCKFF;
  vErr = velCRenc - velRenc;
  if (vErr > ERR_MAX)
    vErr = ERR_MAX;
  else if (vErr < -ERR_MAX)
    vErr = -ERR_MAX;
  if (velCtrl == 1)
  { // p-ff controller
    vP = vErr * velCKP / 2;
    mPWR = (vP + vFF) / 8;
  }
  else
  { // lag-ff controller (one pole one zero)
    vP = (velCKP * (vErr - velErrR / 5)) / 32;
    mPWR = (velCtlR * 10)/9 + vP;
    velCtlR = mPWR; // save old control value - before FF
    mPWR += vFF / 8;
  }
  velErrR = vErr; // save old velocity error value
  // implement
  setMotorVelocity(mPWL, mPWR);
}


/**
 * set motor RC pulse width as a value
 * from -500 to 500, this will result.
 * +500 is max speed forward
 * -500 is max speed reverse
 * RC pulse of 1000 us vith value 500, and
 * 2000 us for value -500,
 * with 0 giving 1500 us as is zero velocity
 * \param velLeft if value for left wheel
 * \param velRight if value for right wheel */
void setMotorVelocity(int16_t velLeft, int16_t velRight)
{ // limit left
  if (velLeft > 470)
    velLeft = 500;
  else if (velLeft > 10)
    velLeft += 30;
  else if (velLeft < -470)
    velLeft = -500;
  else if (velLeft < -10)
    velLeft -= 30;
  // limit right
  if (velRight > 470)
    velRight = 500;
  else if (velRight > 10)
    velRight += 30;
  else if (velRight < -470)
    velRight = -500;
  else if (velRight < -10)
    velRight -= 30;
  // set values
  motorLeftPWM = (1500 - velLeft + offsetL);
  motorLeftPw = motorLeftPWM * TIMER_A_CLOCK;
  motorRightPWM = (1500 - velRight + offsetR);
  motorRightPw = motorRightPWM * TIMER_A_CLOCK;
}

/**
 * is this character start of a number?
 * \param ch is the char to test.
 * \returns 1 if either of "+-0123456789", else 0. */
uint8_t isANumber(unsigned char ch)
{
  return (((ch >= '0') && (ch <= '9')) || (ch == '+') || (ch == '-'));
}

/**
 * calculate velocity in encoder tics since last 10ms cycle
 * Assumes 15 bit encoder values.
 * \param encNow is the new reading
 * \param encLast is the last reading (about 10ms old)
 * \param timerCnt is time value in units of 64us (from timer 2)
 * \returns velocity in encoder counts in 10ms */
int16_t calcEncVelocity(uint16_t encNow, uint16_t encLast, int16_t timerCnt)
{
  uint16_t di;
  int16_t vel;
  if (encNow > encLast)
  { // forward or rolled under
    di = encNow - encLast;
    if (di > 0x4000)
      // rolled under - adjust to backwards
      vel = (di - 0x8000);
    else
      vel = di;
  }
  else
  { // backwards or rolled over
    di = encLast - encNow;
    if (di > 0x4000)
      // rolled over - adjust to forward
      vel = -(di - 0x8000);
    else
      vel = -di;
  }
  if (vel > 180 || vel < -180)
  { // big number,
    if (timerCnt > 170 || timerCnt < 130)
      // time is important - use it anyhow
      return (vel * (TIMER_CNT_10MS/10)) / (timerCnt/10);
    else
      // time is almost 10ms, so assume 10ms
      return vel;
  }
  else
    // use full resolution of timer time
    return (vel * (TIMER_CNT_10MS)) / (timerCnt);
}


