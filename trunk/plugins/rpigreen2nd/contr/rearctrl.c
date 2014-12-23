#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/parity.h>
#include <math.h>
#include "rearctrl.h"

//================================================================
//Define Global Variables
//================================================================

const char * lfcr = "\n\r";

// serial communication
#define RX_BUFF_SIZE 100
char rxbuff[2][RX_BUFF_SIZE];
uint8_t rxbuffCnt = 0;
volatile uint8_t rxbuffN = 0;
char * rxBuffp = rxbuff[0];
uint16_t rxCnt = 0;
uint16_t timer1Cnt = 0;
uint16_t timer2Cnt = 0;
uint8_t localEcho;
int8_t ctrlNCnt;
uint8_t pushStatus = 0;
// timer state
uint8_t timerState = 0;
// string buffer - may not be used in interrupt
#define MSTL 76
char s5[MSTL];
#define AD_CHANNELS 4
uint8_t admuxCnt = 0;
uint16_t adValue[AD_CHANNELS];
// timer count value for each servo (16000 is 1ms, 24 is 1.5ms and 32000 is 2 ms)
uint16_t servoTimerRef[8] = {24000,24000,24000,24000,24000,24000,24000,24000};
uint8_t servoEnabled[8] = {0,0,0,0,0,0,0,0};
// current index (servo number)
uint8_t servoTimerIdx = 0;
uint8_t sensorCnt = 0;



NB! mangler speed af servoer, især lift servo der er alt for hurtig
nok også elbow og grabber

Desuden fejler USB forbindelse efter kold-start

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


ISR(TIMER2_OVF_vect)
{ // overflow if 16.38 ms has pased and no velocity calculation
  // (assumes prescaler of 1024 and 16 MHz)
  // velCalcTime++;
  timer2Cnt++;
}


ISR(TIMER1_COMPA_vect)
{
  // increase timer for simplex receive timeout
  //gwCountNum +=OCR1A;
  // debug timer
  timer1Cnt++;
  // timer state is
  // 1 set up 1st servo
  // 2 is switch to next servo - and stay, until all servos are complete
  // 3 start AD conversion
  // 4 finished - wait for new cycle to be requested
  switch (timerState)
  {
  case 1: // time to talk to servos
    // reset timer
    
    // set compare for first servo
    servoTimerIdx = 0;
    // OCR is count for next event - 16000 is 1ms
    OCR1A = servoTimerRef[servoTimerIdx];
    // output first value on port B - first servo start edge
    if (servoEnabled[servoTimerIdx])
      PORTB |= 1 << servoTimerIdx;
    else
      PORTB = 0;
    break;
  case 2: // talk to first 2 on port B
    servoTimerIdx++;
    if (servoTimerIdx < 2)
    { // set new time
      OCR1A = servoTimerRef[servoTimerIdx];
      // output end-edge for last servo and start edge for this servo
      if (servoEnabled[servoTimerIdx])
        PORTB = 1 << servoTimerIdx;
      else
        PORTB = 0;
      // stay here one cycle more
      timerState--;
    }
    else
    { // finished on port B
      PORTB = 0;
      // start on port D
      PORTD = 1 << servoTimerIdx;
    }
    break;
  case 3: // talk to the rest on port D
    servoTimerIdx++;
    if (servoTimerIdx < 8)
    { // set new time
      OCR1A = servoTimerRef[servoTimerIdx];
      // output end-edge for last servo and start edge for this servo
      PORTD = 1 << servoTimerIdx;
      // stay here one cycle more
      timerState--;
    }
    else
    { // stop last servo and set AD (debug) timing bit
      PORTD = 0x00;
      // set interrupt time to 0.1ms
      OCR1A = 1600;
    }
    break;
  case 4:
    // start AD conversion series
    admuxCnt = 0;
    // start first A/D conversion
    ADMUX = ((1<<REFS1) | (1<<REFS0)) + admuxCnt;
    ADCSRA |= 1 << ADSC;    
    break;
  default: // finished - restart - to maintain servo pulses
    if (timerState > 100)
      // about 10ms longer than servo pulses
      // but should/could be reset by status request
      timerState = 0;
    break;
  }
  timerState++;
}

/**
 * ADC interrupt routine */
ISR(ADC_vect)
{
  adValue[admuxCnt] = ADC;
  // set mux to new channel
  admuxCnt++;
  if (admuxCnt < AD_CHANNELS)
  { // start new conversion, if more channels to test
    ADMUX = ((1<<REFS1) | (1<<REFS0)) + admuxCnt;
    // start new conversion
    ADCSRA |= 1 << ADSC;
  }
  else
  {
    sensorCnt++;
    //PORTB = 0x00;
  }
  // if all values are fetched, then admuxCnt remains 8/4 (AD_CHANNELS)
  // until reset to first channel - and AD started
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
void sendStatus(void)
{
  snprintf(s5, MSTL, "R%d,%d,%d,%d\n\r", adValue[0], adValue[1], adValue[2], adValue[3]);
  sendString(s5);
  snprintf(s5, MSTL, "T%d,%d,%d,%d,%d,%d,%d,%d\n\r", 
           servoTimerRef[0], servoTimerRef[1], servoTimerRef[2], servoTimerRef[3],
           servoTimerRef[4], servoTimerRef[5], servoTimerRef[6], servoTimerRef[7]);
  sendString(s5);
  snprintf(s5, MSTL, "E%d,%d,%d,%d,%d,%d,%d,%d\n\r",
           servoEnabled[0], servoEnabled[1], servoEnabled[2], servoEnabled[3],
           servoEnabled[4], servoEnabled[5], servoEnabled[6], servoEnabled[7]);
  sendString(s5);
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

/////////////////////////////////////

inline uint8_t  limitu8(uint8_t n, uint8_t nmin, uint8_t nmax)
{
  if (n < nmin)
    return nmin;
  else if (n > nmax)
    return nmax;
  else
    return n;
}

/////////////////////////////////////

void sendHelp(void)
{
  // normal help
  sendString("# status/help for green2nd - arm and IR scanner:\n\r");
  sendString("# Servo are disabled by default, and all servos set to 24000.\n\r");
  snprintf(s5, MSTL, "# s[=1]  send status once, or s=1 push status every cycle.\n\r");
  sendString(s5);
  snprintf(s5, MSTL, "# w=M,T  target position of servo M [T=16000 (1ms) .. 32000 (2ms)]\n\r");
  sendString(s5);
  snprintf(s5, MSTL, "# t=0,1,2,3,4,5,6,7 target position of all servos [16000..32000]\n\r");
  sendString(s5);
  snprintf(s5, MSTL, "# e=M,E  enable servo M , E=0 is disabled\n\r");
  sendString(s5);
  snprintf(s5, MSTL, "# a=0,1,2,3,4,5,6,7 enable/disable all servos , 0 is disabled\n\r");
  sendString(s5);
  snprintf(s5, MSTL, "# i=1    local echo in commands\n\r");
  sendString(s5);
  snprintf(s5, MSTL, "# h      help\n\r");
  sendString(s5);
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
  DDRB  |= 0x03;  //Initialize Port B-0,1 to output (servo 0,1 (lift,elbow))
  PORTB &= 0xFC; //set pin 0,1 to 0 (servo signal)

  DDRD = 0xFF;  //Set Port D to all outputs (except RX)
  DDRD &= ~(1<<RX);     //Set bit 0(Rx) to be an input
  PORTD =  (1<<RX);      //Enable the pull-up on Rx
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
//  TIMSK2 = (1 << TOIE2); // timer overflow interrupt (16 ms)

  //Init Timer 1 for 16bit servo pulse generation
  TCCR1A = 0; //Set no output pin effect
  TCCR1B = (1 << WGM12) | (1 << CS10);  //Set CTC mode, clk_IO/1 prescaler
  OCR1A = 0xfffe; // initial value
  TIMSK1 = (1 << OCIE1A); // on compare A (timer reset)

  //Initialize the ADC for Reads to control motor shut-down in case of over current
  ADMUX = ((1<<REFS1) | (1<<REFS0)) + admuxCnt;    // use internal 2.56V reference. Set the ADC channel to 0.
  ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1); //Enable the ADC, the ADC Interrupt and set the ADC prescaler to 64 (about 0.1ms/conversion)
  ADCSRA |= (1<<ADSC);  //Start the first conversion
}

//////////////////////////////////////////////////

int main(void)
{
  int8_t n = 0;
  int cnt = 0;
  uint8_t rxN = 0, rxC = 0;
  uint8_t lastSensorCnt = 100;
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
    if (lastSensorCnt != sensorCnt)
    { // new sensor data, so send status if requested
      lastSensorCnt = sensorCnt;
      if (pushStatus >= 1)
      {
        if (pushStatus == 2)
          // just once
          pushStatus = 0;
        sendStatus();
        timerState = 0;
        if (localEcho && !pushStatus)
          serial_write('>');
      }
    }
    // got new data from host
    if (rxN != rxbuffN)
    {
      n = 0;
      char * cp = rxbuff[rxN];
      uint16_t v;
      if (localEcho)
        sendString(lfcr);
      //
      switch (*cp++)
      {
        case 's': // status request
        case 'S':
          if (*cp == '=')
            pushStatus = strtol(++cp, &cp, 0);
          else
            pushStatus = 2;          
          n = -1;
          break;
        case 't': // servo set (all)
        case 'T':
          // sets motor control directly
          if (*cp == '=')
          {
            uint8_t m = 0;
            cp++;
            while (*cp >= ' ' && m < 8)
            {
              if (*cp == ',')
                cp++;
              v = strtol(cp, &cp, 0);
              if (v > 160) // 10us 
                // fails to work if time is 0
                // and servo do not like < ~ 13000
                servoTimerRef[m++] = v;
            }
            n = -1;
          }
          break;
        case 'a': // enable/disable all servos
        case 'A':
          if (*cp == '=')
          {
            uint8_t m = 0;
            cp++;
            while (*cp >= ' ' && m < 8)
            {
              if (*cp == ',')
                cp++;
              v = strtol(cp, &cp, 0);
              servoEnabled[m++] = v != 0;
            }
            n = -1;
          }
          break;
        case 'w': // servo set, just one
        case 'W':
          // sets motor control directly
          if (*cp == '=')
          {
            uint8_t m = 0;
            cp++;
            m = strtol(cp, &cp, 0);
            m = limitu8(m, 0, 7);
            if (*cp == ',')
              cp++;
            v = strtol(cp, &cp, 0);
            if (v > 160) // 10us 
              // fails to work if time is 0
              // and servo do not like ~ 13000
              servoTimerRef[m] = v;
            n = -1;
          }
          break;
        case 'e': // servo set, just one
        case 'E':
          // enable servo (else no pulse)
          if (*cp == '=')
          {
            uint8_t m = 0;
            cp++;
            m = strtol(cp, &cp, 0);
            m = limitu8(m, 0, 7);
            if (*cp == ',')
              cp++;
            v = strtol(cp, &cp, 0);
            servoEnabled[m] = v != 0;
            n = -1;
          }
          break;
        case 'i': // local echo (interactive)
        case 'I': // local echo
          if (*cp == '=')
            localEcho = strtol(++cp, &cp, 0);
          else
            localEcho = 1;          
          n = -1;
          break;
        case 'h':
        case 'H': // help
          break;
        default:
          snprintf(s5, MSTL, "Unknown command '%s'\n\r", rxbuff[rxN]);
          sendString(s5);
          n = -1;
          break;
      }
      if (n == 0)
        sendHelp();
      //n = 0;
      rxN = rxbuffN;
      rxC = 0;
      if (localEcho && !pushStatus)
        serial_write('>');
    }
    else if (localEcho)
    {
      while (rxbuffCnt > rxC)
      {
        if (rxC >= RX_BUFF_SIZE)
          break;
        serial_write(rxbuff[rxN][rxC++]);
      }
    }
    cnt++;
  }
  return (0);
}
