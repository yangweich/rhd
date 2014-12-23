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
#define STEP_BUF_SIZE 7
int16_t stepA[STEP_BUF_SIZE];
int16_t stepB[STEP_BUF_SIZE];
int16_t stepC[STEP_BUF_SIZE];
int16_t stepD[STEP_BUF_SIZE];
const char * lfcr = "\n\r";

// serial communication
#define RX_BUFF_SIZE 25
char rxbuff[2][RX_BUFF_SIZE];
uint8_t rxbuffCnt = 0;
volatile uint8_t rxbuffN = 0;
char * rxBuffp = rxbuff[0];
uint16_t rxCnt = 0;


int8_t ctrlNCnt;
uint16_t ctlCnt[2] = {0,0}; //control cycle counter
uint16_t statusEvery = 30; // send status every this often

int16_t chirpTA = 50, chirpT, chirpT1, chirpT2, chirpT3, chirpRef; // chirp timer
//int8_t sinus[20] = { 0, 39, 75, 103, 122, 127, 122, 103, 75, 39, 0, -39, -75, -103, -122, -128, -122, -103, -75, -39};
//int8_t sinus[40] = { 0, 20, 39, 58, 75, 90, 103, 114, 122, 125, 127, 125, 122, 114, 103, 90, 75, 58, 39, 20, 
//                     0,-20,-39,-58,-75,-90,-103,-114,-122,-125,-128,-125,-122,-114,-103,-90,-75,-58,-39};
int8_t sinus[32]; // = {  0, 25, 49, 71, 90, 106, 118, 125, 127, 125, 118, 106, 90, 71, 49, 25,
           //   0,-25,-49,-71,-90,-106,-118,-125,-128,-125,-118,-106,-90,-71,-49,-25};
int16_t chirpW;
int16_t chirpPwm;

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
uint16_t remaining_cycleTime;
uint8_t  remaining_cycles;

// motor control
uint16_t motorLeftPw  = 1500 * TIMER_A_CLOCK; // approx 1500us
uint16_t motorRightPw = 1500 * TIMER_A_CLOCK; // approx 1500us
//void  setLED(uint8_t led, uint8_t value);
uint16_t median(uint16_t * val);

// velocity
uint8_t cycleValueCnt = 0;
uint16_t cycleLastEncL = 0;
uint16_t cycleLastEncR = 0;
int16_t velLenc = 0, velRenc = 0; // measured velocity
int16_t velCLenc = 0, velCRenc = 0; // velocity command
int16_t velELenc = 0, velERenc = 0; // velocity error
uint8_t velCtrl = 0; // 1=control velocity, 0=not
void calculateVelocity(void);
void doVelControl(void);
void setMotorVelocity(int16_t velLeft, int16_t velRight);
uint8_t isANumber(unsigned char ch);
int16_t calcEncVelocity(uint16_t encNow, uint16_t encLast);


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
}

ISR(TIMER1_COMPA_vect)
{
  switch (timerState)
  {
  case 1: // motor 1 start
    PORTD |= (1 << 4); // start motor 1 pulse, on pin 4 on port D (S1 in motor ctrl)
    OCR1A = motorLeftPw; // Motor 1 pulse width (timer value)
    timerStateCnt++;
    break;
  case 2: // motor 2 start
    PORTD &= ~(1 << 4); // stop motor 1 pulse
    PORTD |= (1 << 5); // start motor 2 pulse, on pin 5 on port D (S2 in motor ctrl)
    OCR1A = motorRightPw; // Motor 1 pulse width (timer value)
    break;
  case 3: // encoder read start
    PORTD &= ~(1 << 5); // stop motor 2 pulse
    // calculate remaining cycle time - leave 0.5ms for pre-motor control
    remaining_cycleTime = CYCLE_TIME_US - 500 -
                          (motorLeftPw / TIMER_A_CLOCK) -
                          (motorRightPw / TIMER_A_CLOCK);
    remaining_cycles = remaining_cycleTime / ENCODER_READ_TIME_US_N;
    // wait a little longer first time to reach
    // a fixed cycle time
    OCR1A = ENCODER_READ_CNT_N +
            (remaining_cycleTime - remaining_cycles * ENCODER_READ_TIME_US_N) *
            TIMER_A_CLOCK;
    PORTB &= ~(1 << PORTB2); // 0xfb;  // start SPI, set SS to low
    PORTB &= ~(1 << PORTB2); // wait at least 0.5 us
    PORTB &= ~(1 << PORTB2); // wait at least 0.5 us
    SPDR = 0x3C; // start SPI read
    encCycleRead = 0; // for measurements in one cycle (up to 5x4 = 20)
    delayCnt = 1; // for reads in one cycle (approx 20 - depends on motor puls width)
    cycleValueCnt = 0; // full read cycle - up to 4 in one cycle
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
    break;
//    case 5: // wait further before next cycle
//      delayCnt++;
//      if (delayCnt < remaining_cycles)
//        timerState--;
//      break;
  default: // stop reads, start motor ctrl
    timerState = 0;
    OCR1A = 500 * TIMER_A_CLOCK; // ~1ms wait after encoder read
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
void sendInt(const char * pre, int value1, int value2, const char * post, int fmt)
{
  const int MSL = 20;
  char s[MSL];
  sendString(pre);
  switch (fmt)
  { // unsigned values
    case 1: snprintf(s, MSL, "%1u %1u", value1, value2); break;
//    case 2: snprintf(s, MSL, "%2u %2u", value1, value2); break;
//    case 3: snprintf(s, MSL, "%3u %3u", value1, value2); break;
//    case 4: snprintf(s, MSL, "%4u %4u", value1, value2); break;
//    case 5: snprintf(s, MSL, "%5u %5u", value1, value2); break;
//    case 6: snprintf(s, MSL, "%6u %6u", value1, value2); break;
    case 8: snprintf(s, MSL, "%2x %2x", value1, value2); break;
    case 16: snprintf(s, MSL, "%4x %4x", value1, value2); break;
    // signed values
//    case -1: snprintf(s, MSL, "%1d %1d", value1, value2); break;
//    case -2: snprintf(s, MSL, "%2d %2d", value1, value2); break;
//    case -3: snprintf(s, MSL, "%3d %3d", value1, value2); break;
    case -4: snprintf(s, MSL, "%4d %4d", value1, value2); break;
    case -5: snprintf(s, MSL, "%5d %5d", value1, value2); break;
    case -6: snprintf(s, MSL, "%6d %6d", value1, value2); break;
    // default is flexible
    default:
//      snprintf(s, MSL, "%d %d", value1, value2);
      break;
  };
  sendString(s);
  if (post != NULL)
    sendString(post);
}

/**
 * Send an integer with an explanation
 * \param pre is some text to send before the integer value
 * \param value is the integer value to send (6 chars reserver - decomal notation
 * */
void sendInt1(const char * pre, int value, int fmt)
{
  char s[10];
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
void sendCtrlStatus(char * post)
{
//  sendInt1("enc:", timerStateCnt, 5);
  sendInt("E ", encL, encR, " ", 16);
  sendInt("V ", velLenc, velRenc, " ", -4);
//   sendInt("OCF ", encL_OCF, encR_OCF, " ", 1);
//   sendInt("COF ", encL_COF, encR_COF, " ", 1);
//   sendInt("LIN ", encL_LIN, encR_LIN, " ", 1);
   sendInt("M ", encL_MAG, encR_MAG, " ", 1);
   sendInt("S ", (PIND & 0x80) != 0, (PIND & 0x40) != 0, " ", 1);
//   sendInt("PAR ", encL_PAR, encR_PAR, " ", 1);
//  sendInt("err ", encLParityErrCnt, encRParityErrCnt, " ", 16);
//   sendInt("vC ", velCLenc, velCRenc, " ", -4);
//   sendInt("mpw ", 1500 - motorLeftPw/TIMER_A_CLOCK,
//                   1500 - motorRightPw/TIMER_A_CLOCK , " ", -4);

   if (post != NULL)
    sendString(post);
  else
    sendString("\n\r");
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

///////////////////////////////////


/////////////////////////////////////

void sendHelp(void)
{
  // fast status of data from encoder
//   sendInt("enc 0 (", encBufOK[0], encBuf[0][0], "", 8);
//   sendInt(" ", encBuf[0][1], encBuf[0][2], "", 8);
//   sendInt(" ", encBuf[0][3], encBuf[0][4], ")\n\r", 8);
//   sendInt("enc 1 (", encBufOK[1], encBuf[1][0], "", 8);
//   sendInt(" ", encBuf[1][1], encBuf[1][2], "", 8);
//   sendInt(" ", encBuf[1][3], encBuf[1][4], ")\n\r", 8);
//   sendInt("enc 2 (", encBufOK[2], encBuf[2][0], "", 8);
//   sendInt(" ", encBuf[2][1], encBuf[2][2], "", 8);
//   sendInt(" ", encBuf[2][3], encBuf[2][4], ")\n\r", 8);
//   sendInt("enc 3 (", encBufOK[3], encBuf[3][0], "", 8);
//   sendInt(" ", encBuf[3][1], encBuf[3][2], "", 8);
//   sendInt(" ", encBuf[3][3], encBuf[3][4], ")\n\r", 8);
//  sendInt("err   (", encLParityErrCnt, encRParityErrCnt,")\n\r", 16);
  //
  // normal help
  sendString("---- options for GoGo motor ctrl\n\r");
  sendInt("- w99 99 (", 1500 - motorLeftPw/TIMER_A_CLOCK,
                       1500 - motorRightPw/TIMER_A_CLOCK,   ") RC-PW -500..500\n\r", -5);
  sendInt("- v99 99 (", velCLenc, velCRenc,   ") vel ref -2000..+2000 (enc/10ms)\n\r", -5);
  sendInt("- p99 (", velCtrl, velCtrl,   ") vel control 0=not\n\r", -5);
  
  //sendInt("-    m9   (", ctrlMode,ctrlMode,     ") Set control mode (0=PWM, 1=position, 2=adc7 is ref)\n\r", -5);
  sendInt("-    c9   (", chirpTA,chirpT3,       ") chirp Max ref value\n\r", -5);
  sendInt("-    l9   (", 0, 1,                  ") List control status (times)\n\r", -5);
  sendInt("-    g[N] (", statusEvery, statusEvery, ") send every N\n\r", -5);
//  sendInt("-    d99  (", deadband, deadband,    ") deadband pwm\n\r", -5);
  sendInt("-    s    (", STEP_BUF_SIZE, stepCnt,") List step log\n\r", -5);
  sendInt("-    h    (", 0, 1,                  ") This help\n\r", -5);
  sendString("\n\r");
  sendString("Periodic: E encoder, V encoder/5ms, M mag-status, S start,emerg sw\n\r");
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
  //Used for delay routines
  //TCCR2B = (1<<CS20);         //Divde clock by 1 for 16 Mhz Timer 2 Frequency

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

void sendLine(const char * str)
{
  sendString(str);
  sendString(lfcr);
}

//////////////////////////////////////////////////

int main (void)
{	
  uint8_t n = 0;
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
      { // find filtered value
        encL = median(encLRaw);
        encR = median(encRRaw);
        encCnt++;
        // calculate velocity in step one
        if (cycleValueCnt == 0)
          calculateVelocity();
        // do control in step two (of 4-5 steps each 10ms cycle)
        if (cycleValueCnt == 1 && velCtrl)
          doVelControl();
        // increase step counter
        cycleValueCnt++;
        if ((statusEvery > 0) && ((encCnt % statusEvery) == 0))
          sendCtrlStatus(NULL);
      }
    }
    // get new data
    if (rxN != rxbuffN)
    {
      n = 0;
      char * cp = rxbuff[rxN], *c2;
      // restart logging
      int16_t sinus8[32] = {  0, 25, 49, 71, 90, 106, 118, 125, 127, 125, 118, 106, 90, 71, 49, 25,
              0,-25,-49,-71,-90,-106,-118,-125,-128,-125,-118,-106,-90,-71,-49,-25};
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
            setMotorVelocity(s16, t16);
          else
            sendString("not 2 values\n\r");
          n = 0;
          break;
        case 'v': // motor velocity encoder tics/10ms
          // about +/- 2000
          s16 = strtol(cp, &c2, 0);
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
            sendString("not 2 values\n\r");
          n = 0;
          break;
        case 'p': // motor vel ctrl 0=no ctrl, 1=ctrl
          velCtrl = strtol(cp, NULL, 0);
          if (velCtrl == 0)
            setMotorVelocity(0,0);
          n = 0;
          break;
        case 'l': // list fast status (n times)
          if (*cp >= '0')
            n = strtol(cp, NULL, 0);
          else
            n = 3;
          n++;
          break;
        case 'c': // chirp
          // start chirp with a count 20 => ca 1Hz as lowest frq
          chirpTA = strtol(cp, NULL, 0);
          for (int i = 0; i < 32; i++)
          {
            int16_t a = (sinus8[i] * chirpTA) >> 8;
            sinus[i] = a;
            sendInt("sinus", sinus8[i], a, "\n\r", -4);
          }
          // change around this value
          chirpRef = 0; // posRef[0];
          chirpT = 70; // 70 * 0.4ms between phase shift at lowest frq
          chirpT1 = 0;
          chirpT2 = chirpT;
          chirpT3 = 6;
          stepCnt = 0;
          break;
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
        case 's': // dump values and stop
          sendLine("sample current forceval fctrl PWMused)");
          for (int i = 0; i < STEP_BUF_SIZE; i++)
          {
            sendInt1(" ", i, 5);
            sendInt1("; ", stepA[i], -4);
            sendInt1("; ", stepB[i], -4);
            sendInt1("; ", stepC[i], -4);
            sendInt1("; ", stepD[i], -4);
            sendLine("");
          }
          n = 1;
          break;
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
      sendCtrlStatus(NULL);
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
 * Calculate velocity based on last encoder values
 * for left and right motor (wheel) */
void calculateVelocity(void)
{
  velLenc =  calcEncVelocity(encL, cycleLastEncL);
  velRenc = -calcEncVelocity(encR, cycleLastEncR);
  // save last
  cycleLastEncL = encL;
  cycleLastEncR = encR;
}

/**
 * Do velocity control */
void doVelControl(void)
{ // proportional and feed forward control
  int16_t vErr;
  int16_t vFF;  // feed forward control value
  int16_t vP;   // proportional control value
  int16_t mPWL, mPWR; // change to pulse width relative to 1500 us
  if (((PIND & 0x80) != 0) && ((PIND & 0x40) == 0))
  { // not emergency stop    
    // left motor
    vFF = velCLenc / 2;
    vErr = velCLenc - velLenc;
    vP = vErr / 4;
    mPWL = vP + vFF;
    // right motor
    vFF = velCRenc / 2;
    vErr = velCRenc - velRenc;
    vP = vErr / 4;
    mPWR = vP + vFF;
    setMotorVelocity(mPWL, mPWR);
  }
  else
    // emergency stop
    setMotorVelocity(0, 0);
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
  if (velLeft > 500)
    velLeft = 500;
  else if (velLeft > 10)
    velLeft += 30;
  else if (velLeft < -500)
    velLeft = -500;
  else if (velLeft < -10)
    velLeft -= 30;
  // limit right
  if (velRight > 500)
    velRight = 500;
  else if (velRight > 10)
    velRight += 30;
  else if (velRight < -500)
    velRight = -500;
  else if (velRight < -10)
    velRight -= 30;
  // set values
  motorLeftPw = (1500 - velLeft) * TIMER_A_CLOCK;
  motorRightPw = (1500 - velRight) * TIMER_A_CLOCK;
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
 * \param encLast is the last reading (10ms old)
 * \returns velocity in encoder counts in 10ms */
int16_t calcEncVelocity(uint16_t encNow, uint16_t encLast)
{
  uint16_t di;
  int16_t vel;
  if (encNow > encLast)
  {
    di = encNow - encLast;
    if (di > 0x4000)
      // rolled under - adjust
      vel = (0x8000 - di);
    else
      vel = -di;
  }
  else
  {
    di = encLast - encNow;
    if (di > 0x4000)
      // rolled over - adjust
      vel = -(0x8000 - di);
    else
      vel = di;
  }
  return vel;
}