/* Simple example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2008 PJRC.COM, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#define REV "$Rev: 373 $"
#define REV_ID "$Id: greenuc.c 373 2013-12-28 08:24:12Z jcan $"

/**
 * Controller for steering servo and magnetic encoders
 * for DTU field robot */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <util/parity.h>
#include "usb_serial.h"

#define LED_CONFIG  (DDRD  |= (1 << 6))
#define LED_ON      (PORTD |= (1 << 6))
#define LED_OFF     (PORTD &= ~(1 << 6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

const char * lfcr = "\n\r";
void send_str(const char *s);
uint8_t recv_str(char *buf, uint8_t size);
void parse_and_execute_command(const char *buf, uint8_t num);
int localEcho = 0;
uint8_t timingChar = 'a';
uint8_t pushStatus = 0; // 1 = send status automatically to USB-hostwhen available

// Encoder (3 bytes for one (left) encoder 5 for two (left and right) and
// 8 bytes buffer for 3 encoders (left, right and tilt))
#define EBL 8
// NB number of buffes fixed to 4 - change require code change too.
#define EBL_N 4
uint8_t encBuf[EBL_N][EBL];
uint8_t encBufCnt = 0;
uint8_t encBufN = 0;
uint8_t encBufOK[EBL_N] = {0,0,0,0};
uint16_t encCnt = 0;
uint8_t par1, par2, par3;

// timer state
uint8_t timerState = 0;
//uint8_t encCycleRead = 0;
uint8_t encReadCnt, encDataCnt;
uint16_t timerStateCnt = 0;
// timer A clock (CLK_IO / prescaler)
#define TIMER_A_CLOCK (16/1)
// encoder read time in us * CLK_IO frequency in MHz
char s2[4] = "ab\0"; // debug
//int8_t  encReadCycles;
char s3[4] = "12\0"; // debug
int spiCnt = 0;
//uint16_t timer1Cnta, timer1Cntb, timer1Cntc, timer1Cntd, timer1Cnte;
uint16_t timer1Cnt = 0;
#define MSTL 150
char s5[MSTL];

uint8_t admuxCnt = 2;
uint16_t adValue[8];
// number of servos to generate pulses for (up to 7)
#define USED_SERVO_CNT 2
// servo initial position
// less than 100 is off (no servo pulse) 16000 is 1ms, 32000 is 2ms
uint16_t servoTimerStop[7] = {90,90,24000,25000,26000,27000,28000};
// current index (servo number)
uint8_t servoTimerIdx = 0;
uint8_t sensorCnt = 0;

/**
 * set servo with index servoTimerIdx, 
 * to the value in servoTimerStop[servoTimerIdx]
 * if too short (< 100) then disable servo, by not
 * making a pulse */
void setServoPulse(void)
{
  if (servoTimerStop[servoTimerIdx] >= 100)
  { // set compare for first servo
    // OCR is count for next event - 16000 is 1ms
    OCR1A = servoTimerStop[servoTimerIdx];
    // output first value on port B - first servo start edge
    PORTB = 1 << servoTimerIdx;
  }
  else 
    PORTB = 0;
}
///
/** timer 1 (16 bit) interrupt
 * this is used to split time into encoder read and communication with servos
 * state 1 and 2 is reserved time for serco communication,
 * state 3 and 4 is encoder read states */
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
    servoTimerIdx = 0;
    setServoPulse();
//     if (servoTimerStop[servoTimerIdx] >= 100)
//     { // set compare for first servo
//       // OCR is count for next event - 16000 is 1ms
//       OCR1A = servoTimerStop[servoTimerIdx];
//       // output first value on port B - first servo start edge
//       PORTB = 1 << servoTimerIdx;
//     }
//     else 
//       PORTB = 0;
    break;
  case 2: // stop talking to servos
    servoTimerIdx++;
    if (servoTimerIdx < USED_SERVO_CNT)
    { // continue for remaining servos
      setServoPulse();
//       if (servoTimerStop[servoTimerIdx] >= 100)
//       { // set new time
//         OCR1A = servoTimerStop[servoTimerIdx];
//         // output end-edge for last servo and start edge for this servo
//         PORTB = 1 << servoTimerIdx;
//       }
//       else 
//         PORTB = 0;
//       break;
      // stay here one cycle more
      timerState--;
    }
    else
    { // stop last servo pulse - and set pin 8 to 1 to mark start of A/D converter period.
      PORTB = 0x80;
      // set next time to 100us
      OCR1A = 1600;
    }
    break;
  case 3:
    // start AD conversion series
    admuxCnt = 0;
    // start new conversion
    ADMUX = ((1<<REFS1) | (1<<REFS0)) + admuxCnt;
    ADCSRA |= 1 << ADSC;    
    break;
  default: // finished - restart
    // wait to allow RHD to trigger new pulse
    // else restart to maintain servo pulses
    if (timerState > 150)
      timerState = 0;
    break;
  }
  timerState++;
}


/**
 * ADC interrupt routine */
ISR(ADC_vect)
{ // get AD value
  adValue[admuxCnt] = ADC;
  // set mux to new channel
  admuxCnt++;
  if (admuxCnt <= 7)
  { // start new conversion, if more channels to test
    ADMUX = ((1<<REFS1) | (1<<REFS0)) + admuxCnt;
    // start new conversion
    ADCSRA |= 1 << ADSC;
  }
  else
  {
    sensorCnt++;
    // mark end of A/D converter period
    PORTB = 0x00;
  }
  // if all values are fetched, then admuxCnt remains 8
  // until reset to first channel - and AD started
}

/**
 * Send string - and wait until send
 * \param str is the string to send */
void sendString(const char * str)
{
  uint16_t n = strlen(str);
  usb_serial_write((const uint8_t *) str, n);
//   while (1)
//   { // send one char at a time until no more
//     if (str[i] == '\0')
//       break;
//     serial_write(str[i++]);
//   }
}

void sendStringLn(const char * str)
{
  sendString(str);
  sendString(lfcr);
  usb_serial_flush_output();
}

/**
 * Send status of interface */
void sendStatus(void)
{
//   const int MSL = 20;
//   char s[MSL];
  if (PORTD & (1 << PD6))
    PORTD &= ~(1 << PD6);
  else
    PORTD |=  (1 << PD6);
  if (localEcho)
  {
    snprintf(s5, MSTL, "S %u %u\r\n",
            servoTimerStop[0], servoTimerStop[1] /*, servoTimerStop[2], servoTimerStop[3]*/
              );
    sendString(s5);
  }
  // 0,1 is bumper value - short time constant
  // 2..7 is IR sensor
  // 8 is port C (floor detect c0..c3), all 8 pins returned, rest is NC
  snprintf(s5, MSTL, "G %d %d %d %d %d %d %d %d %d",
           adValue[0], adValue[1], adValue[2], adValue[3], adValue[4], adValue[5], adValue[6], adValue[7],
           PINC
            );
  sendStringLn(s5);
}

//==================================================
//Core functions
//==================================================
//Function: ioinit
//Purpose:      Initialize AVR I/O, UART and Interrupts
//Inputs:       None
//Outputs:      None
void ioinit(void)
{ // SPI setup
  //1 = output, 0 = input
  /* Set MOSI (PB2) and SCK (PB1) and SS (PB0) as output, all others input */
  PORTB = 0x00; // set output to servos to "idle"
  DDRB = 0xff; // set port B as all output - servo pons
//   DDRB = (1 << PB2) | (1<<PB1) | (1 << PB0);
//   PORTB |= (1 << PB3);   //Enable pull-up on MISO pin
//   PORTB |= (1 << PB0); // set SS to 1 (tell slaves to disable)
  // SPI interrupt, SPI as master, set SPI clock rate fck/64
//   SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << SPR1);
//   SPSR = 0; // not double speed (bit 0 writable only)

  //
  DDRD  = (1 << PD3) | (1 << PD6);  //Set Port D bit 3 (TX) and bit 6 (LED) to outputs - all other input
  PORTD = (1 << PD2);  //Enable the pull-up on Rx (not needed, we have an external)
  // returns baudrate divisor
  // as is the same as

  //Init Timer 1 for CTC mode (clear on count in OCRA and make interrupt on OCRA
  TCCR1A = 0; //Set no output pin effect
  TCCR1B = (1 << WGM12) | (1 << CS10);  //Set CTC mode, clk_IO/1 prescaler, clear timer count, when OCRA is reached
  OCR1A = 0xfffe; // initial compare value
  TIMSK1 = (1 << OCIE1A); // interrupt enable on compare A

  //Initialize the ADC for Reads to control motor shut-down in case of over current
  ADMUX = ((1<<REFS1) | (1<<REFS0)) + admuxCnt;    // use internal 2.56V reference. Set the ADC channel to 0.
  ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1); //Enable the ADC, the ADC Interrupt and set the ADC prescaler to 64 (about 0.1ms/conversion)
  ADCSRA |= (1<<ADSC);  //Start the first conversion

  // make the pin C0..C4 an output used to control servo tack direction
  // and pin C7 is debug signal for dxl talk (no encoder read)
  DDRC |= ((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3) | (1 << PINC7));
}



// Basic command interpreter for controlling port pins
int main(void)
{
#define BUF_SIZE 32
  char buf[BUF_SIZE];
  uint8_t n;
  uint16_t loop = 0;
//  uint8_t err[DXL_SERVO_CNT];
//  uint8_t statusTime = 0;
  uint8_t lastSensorCnt = 0;

  // set for 16 MHz clock, and turn on the LED
  CPU_PRESCALE(0);
  LED_CONFIG; // D6 as output (LED)
  LED_ON;     // turn LED on
  //
  // init communication with dynamixel
  ioinit();
  //
  // initialize the USB, and then wait for the host
  // to set configuration.  If the Teensy is powered
  // without a PC connected to the USB port, this
  // will wait forever.
  usb_init();
  while (!usb_configured())
    /* wait */ ;
  // and a bit more
  _delay_ms(1000);
//  InitPWM();
//  DDRD  |= (1 << PD0);
//  DDRB  |= (1 << PB7);
//   OCR0B  = 127;
//   OCR0A  = 127;
  // main loop
  while (1)
  { // wait for the USB user to run
    // which sets DTR to indicate it is ready to receive.
    while (!(usb_serial_get_control() & USB_SERIAL_DTR)) /* wait */ ;

    // discard anything that was received prior.  Sometimes the
    // operating system or other software will send a modem
    // "AT command", which can still be buffered.
    usb_serial_flush_input();
    n = 0;
    // and then listen for commands and process them
    while (1)
    {
      uint8_t m;
//      statusTime = 1;
      loop++;
      if (lastSensorCnt != sensorCnt)
      {
        lastSensorCnt = sensorCnt;
        if (pushStatus >= 1)
        {
          if (pushStatus == 2)
            // just once
            pushStatus = 0;
          sendStatus();
          timerState = 0;
        }
      }
//       if (timerState > 40)
//         timerState = 0;
      // get number of available chars
      m = usb_serial_available();
      if (m > 0)
      { // something on the USB channel and servo command buffer is clear
        buf[n] = usb_serial_getchar();
        if (localEcho)
           usb_serial_putchar(buf[n]);
        buf[n + 1] = '\0';
        if (n > 0 && (buf[n] == '\n' || buf[n] == '\r'))
        { // there is a command terminated with either
          if (localEcho)
            send_str(PSTR("\r\n"));   //\f
          parse_and_execute_command(buf, n);
          if (localEcho)
            send_str(PSTR(">"));
          // clear command buffer
          n = 0;
          buf[n] = '\0';
        }
        if (buf[n] >= ' ' && buf[n] <= '~')
          n++;
        if (n >= BUF_SIZE)
          // input error - restart
          n = 0;
      }
    }
  }
}

// Send a string to the USB serial port.  The string must be in
// flash memory, using PSTR
//
void send_str(const char *s)
{
//   int n;
//   const char * p1 = s;
//   while (*p1)
//     p1++;
//   n = p1 - s;
//   usb_serial_write(s, n);
//   usb_serial_flush_output();
  char c;
  while (1)
  {
    c = pgm_read_byte(s++);
    if (!c)
      break;
    usb_serial_putchar(c);
  }
}

/** Receive a string from the USB serial port.  The string is stored
// in the buffer and this function will not exceed the buffer size.
// A carriage return or newline completes the string, and is not
// stored into the buffer.
\param buffer is buffer for the received characters
\param size is size of buffer.
\return value is the number of characters received, or 255 if
// the virtual serial connection was closed while waiting.
*/
uint8_t recv_str(char *buf, uint8_t size)
{
  int16_t r;
  uint8_t count=0;

  while (count < size)
  {
    r = usb_serial_getchar();
    if (r != -1)
    {
      if (r == '\r' || r == '\n')
        return count;
      if (r >= ' ' && r <= '~')
      { // save character in buffer
        *buf++ = r;
        if (localEcho)
          // do we want an echo of what we send?
          usb_serial_putchar(r);
        count++;
      }
    }
    else
    {
      if (!usb_configured() ||
          !(usb_serial_get_control() & USB_SERIAL_DTR))
      { // user no longer connected
        return 255;
      }
      // just a normal timeout, keep waiting
    }
  }
  return count;
}

// parse a user command and execute it, or print an error message
//
void parse_and_execute_command(const char *buf, uint8_t num)
{
  uint8_t port = 0, pin = 0;
  uint16_t val;
  char isOK = 0;
  const char * help = PSTR("\n"
    "Green robot steering USB interface, " REV_ID "\r\n"
    "Control Shell\r\n"
    "  F6=1   Write 1 to port pin F6\r\n"
    "  D6=1   Write 1 Port D pin 6 (D6 is LED pin)\r\n"
    "  B4=0   Get status for switches (pin B4 not used)\r\n"
    "  i=1    Interactive - use local echo of all commands\r\n"
    "  j=a    set timing char - is returned as part of status\r\n"
    "  K=L,R  set left and right servo to these values [16000..32000] (1..2 ms) (decimal)\r\n"
//    "  P=P,T  set Pan and Tilt camera servo to these values [16000..32000] (1..2 ms) (decimal)\r\n"
    "  s[=1]    Send status (1= push status, 0= stop)\r\n"
    "  reset  (no action)\r\n"
    "  help   This help text\r\n"
    );
//   if (num < 3)
//   { // too short
//     send_str(PSTR("unrecognized format, 3 chars min req'd\r\n"));
//     return;
//   }
//  send_str(PSTR("OK1\n"));
  // first character is the port letter
  if ((buf[0] == 'k' || buf[0] == 'K') && buf[1] == '=')
  {
    char * p1 = (char*)&buf[2];
    uint16_t wl, wr;
    wl = strtol(p1, &p1, 0);
    isOK = (*p1++ == ',');
    if (isOK)
    { // valid request - set request
      wr = strtol(p1, &p1, 0);
      servoTimerStop[0] = wl;
      servoTimerStop[1] = wr;
    }
  }
  else if ((buf[0] == 'p' || buf[0] == 'P') && buf[1] == '=')
  { // unused - planned for pan-tilt servos
    char * p1 = (char*)&buf[2];
    uint16_t wl, wr;
    wl = strtol(p1, &p1, 0);
    isOK = (*p1++ == ',');
    if (isOK)
    { // valid request - set request
      wr = strtol(p1, &p1, 0);
      servoTimerStop[2] = wl;
      servoTimerStop[3] = wr;
    }
    isOK = 1;
  }
  else if (buf[0] >= 'A' && buf[0] <= 'F')
  {
    port = buf[0] - 'A';
  }
  else if (buf[0] >= 'a' && buf[0] <= 'f')
  {
    port = buf[0] - 'a';
  }
  else if (buf[0] == 'i' && buf[1] == '=')
  {
    localEcho = buf[2] == '1';
    isOK = 1;
  }
  else if (buf[0] == 'j' && buf[1] == '=')
  {
    timingChar = buf[2];
    isOK = 1;
  }
  else if (buf[0] == 's')
  {
    if (buf[1] == '=')
    {
      pushStatus = strtol(&buf[2], NULL, 0);
    }
    if (pushStatus == 0)
      sendStatus();
    isOK = 1;
  }
  else if (buf[0] == 'h' || buf[0] == 'H')
  {
    send_str(help);
    isOK = 1;
  }
  else if (strstr(buf, "reset") != 0)
  {
//     getServoStatus();
//     sendServoStatus();
    isOK = 1;
  }
  else
  {
    snprintf(s5, MSTL, "Err: got %d chars: %s\n", num, buf);
//     send_str(PSTR("Unknown command "));
//     sendString(buf);
//     usb_serial_putchar(buf[0]);
//     send_str(PSTR("\"\r\n"));
    // send_str(help);
    isOK = 1;
  }
  // second character is the pin number
  if (isOK == 0)
  { // not handled port command (set or get)
    if (buf[1] >= '0' && buf[1] <= '7')
    {
      pin = buf[1] - '0';
    }
    else
    {
      send_str(PSTR("Unknown pin \""));
      usb_serial_putchar(buf[0]);
      send_str(PSTR("\", must be 0 to 7\r\n"));
      isOK = 2;
    }
    // if the third character is a question mark, read the pin
    if (isOK==0 && buf[2] == '?')
    { // make the pin an input
      *(uint8_t *)(0x21 + port * 3) &= ~(1 << pin);
      // read the pin
      val = *(uint8_t *)(0x20 + port * 3) & (1 << pin);
      usb_serial_putchar(val ? '1' : '0');
      send_str(PSTR("\r\n"));
      isOK = 2;
  //    return;
    }
    else
    { // not a read
      // if the third character is an equals sign, write the pin
      if (num >= 4 && buf[2] == '=')
      {
        if (buf[3] == '0')
        { // make the pin an output
          *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
          // drive it low
          *(uint8_t *)(0x22 + port * 3) &= ~(1 << pin);
          isOK = 1;
  //        return;
        }
        else if (buf[3] == '1')
        { // make the pin an output
          *(uint8_t *)(0x21 + port * 3) |= (1 << pin);
          // drive it high
          *(uint8_t *)(0x22 + port * 3) |= (1 << pin);
          isOK = 1;
  //        return;
        }
        else
        {
          send_str(PSTR("Unknown value \""));
          usb_serial_putchar(buf[3]);
          send_str(PSTR("\", must be 0 or 1\r\n"));
          isOK = 2;
        }
      }
    }
  }
  // otherwise, error message
  if (isOK == 0)
  {
    send_str(PSTR("Unknown '"));
    usb_serial_putchar(buf[0]);
    send_str(PSTR("' command, must have ? or =\r\n"));
  }
//   else
//     send_str(PSTR("OK2\n"));
}

