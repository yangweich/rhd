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
#define REV "$Rev: 280 $"
#define REV_ID "$Id: dxlctrl.c 280 2013-10-20 14:30:48Z jcan $"

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
#include "dynamixel.h"
#include "dxl_hal.h"

#define LED_CONFIG  (DDRD  |= (1 << 6))
#define LED_ON      (PORTD |= (1 << 6))
#define LED_OFF     (PORTD &= ~(1 << 6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

const char * lfcr = "\n\r";
void send_str(const char *s);
uint8_t recv_str(char *buf, uint8_t size);
void parse_and_execute_command(const char *buf, uint8_t num);
// void InitPWM(void);
// void SetPWMOutput0(uint8_t duty);
// void SetPWMOutput1(uint8_t duty);
// void SetPWMOutput(uint8_t duty, uint8_t port);
int localEcho = 1;
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
#define ENC_SUM_VALUES 1
uint8_t  encLadd = 0, encRadd = 0;
uint16_t encL, encLsum, encLx4 = 1, encLparErr = 0;
uint16_t encR, encRsum, encRx4 = 2, encRparErr = 0;
uint16_t encT, encTsum, encTx4 = 3, encTparErr = 0;
uint8_t encL_MAG, encL_PAR;
uint8_t encR_MAG, encR_PAR;
uint8_t encT_MAG, encT_PAR;
uint8_t par1, par2, par3;

// timer state
uint8_t timerState = 0;
//uint8_t encCycleRead = 0;
//uint8_t encReadCnt, encDataCnt;
uint16_t timerStateCnt = 0;
// timer A clock (CLK_IO / prescaler)
#define TIMER_A_CLOCK (16/1)
// encoder read time in us * CLK_IO frequency in MHz
#define ENCODER_READ_TIME_US_N 390
#define ENCODER_READ_CNT_N (ENCODER_READ_TIME_US_N * TIMER_A_CLOCK)
//#define CYCLE_TIME_US 10000
int16_t remaining_cycleTime;
char s2[4] = "ab\0"; // debug
//int8_t  encReadCycles;
char s3[4] = "12\0"; // debug
int spiCnt = 0;
//uint16_t timer1Cnta, timer1Cntb, timer1Cntc, timer1Cntd, timer1Cnte;
int16_t timer1Cnt = 0;
#define MSTL 100
char s5[MSTL];
uint8_t lastStatusCnt = 0, lastStatusSend = 0;
uint8_t statusAll = 0;
//
// dynamixel control and status
uint8_t dxlTalkOK = 0;
uint16_t space4dxlTalk = 32, space4dxlTalkCnt; // space for about 12 characters (tx+rx)
uint16_t dxlRxVal, dxlTXVal;
uint8_t dxlID;
uint8_t dxlAdr;
uint8_t dxlWord;
uint8_t dxlRead = 1;
uint8_t dxlCmdTx = 0; // command valid (not send)
uint8_t dxlCmdRx = 0; // reply valid
//
#define DXL_SERVO_CNT 5
#define DXL_REG_CNT   50
int8_t dxlReg[DXL_SERVO_CNT][DXL_REG_CNT];
#define DXL_PRESENT_POS  36
#define DXL_PRESENT_VEL  38
#define DXL_PRESENT_LOAD 40
#define DXL_PRESENT_VOLT 42
#define DXL_PRESENT_TEMP 43
#define DXL_PRESENT_ALARM 20
float dxlBaudRate = 500000.0; // in bit/sec, default is 57600 bit/s, 500kbit/s is fine
int8_t dxlIDList[DXL_SERVO_CNT] = { 1, 2, 3, 4, 5};
// debug value
int8_t dxlBaudrateDivisor;
// request status from this servo
int8_t dxlStatusID = 0;
/// debug values
uint16_t timer1CntReg[DXL_SERVO_CNT];
uint16_t servoErrCnt[DXL_SERVO_CNT] = {0,0,0,0,0};



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
    PORTB |= (1 << PB0); // 0xfb;  // stop SPI, set SS to high
    // filled buffer is valid
    encBufOK[encBufN] = 1;
    encBufN = (encBufN + 1) & 0x3;
    // new buffer is now invalid and empty
    encBufOK[encBufN] = 0;
    encBufCnt = 0;
  }
  spiCnt++;
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
  // do encoder read sycles
  switch (timerState)
  {
  case 1: // time to talk to servos
     OCR1A = 150 * TIMER_A_CLOCK; 
     //gwRxTimeout = 0; // time to transmit/receive from dynamixel
     dxlTalkOK = 1;
     PORTC |= 0x80;
     //timer1Cnta = TCNT2;
     space4dxlTalkCnt = 0;
    break;
  case 2: // talking to servos
     space4dxlTalkCnt++;
     if (space4dxlTalkCnt < 12) //space4dxlTalk)
       // stay here one cycle more
       timerState--;
     //timer1Cntb = TCNT2;
    break;
  case 3: // stop talking to servos
    // end dxl talk time
    dxlTalkOK = 0;
    if (timer1Cnt > 550)
      // use long timeout during startup (approx 250 ms)
      gwRxTimeout = 1;
    PORTC &= ~0x80;
    // start encoder read
//     encReadCnt = 0;
//     encDataCnt = 0;
    encLsum = 0;
    encRsum = 0;
    encTsum = 0;
    //timer1Cntc = TCNT2;
    break;
//   case 4: // encoder read
//     OCR1A = 100 * TIMER_A_CLOCK; // ENCODER_READ_CNT_N; // ~400us sample rate for encoder 2kHz
//     if (encBufCnt == 0)
//     {  // start new read
//       PORTB &= ~(1 << PB0); // 0xfb;  // start SPI, set SS to low
//       PORTB &= ~(1 << PB0); // wait at least 0.5 us
//       PORTB &= ~(1 << PB0); // wait at least 0.5 us
//       SPDR = 0x3C;
//     }
//     if (encReadCnt < ENC_SUM_VALUES)
//       // stay in read mode for (4) reads
//       timerState--;
//     encReadCnt++;
//     //timer1Cntd = TCNT2;
//     break;
  default: // finished - restart
    timerState = 0;
    OCR1A = 100 * TIMER_A_CLOCK; // ~1ms wait after encoder read
    //timer1Cnte = TCNT2;
    break;
  }
  timerState++;
  lastStatusCnt++;
}

/**
 * Decode message from 1..3 AS5045 encoders received in one buffer (encBuf)
 * encoder value is stored in a arw encoder value buffer encXRaw, to
 * allow filtering of stary error values (buffer with 5 values).
 * OCF when 1: offset compensation finished is good
 * COF Cordic overflow - when 1:value is form last good measurement
 * LIN liniarity is bad, when 1: magnet alignmenet out of order
 * MAG when 0 magnets are good and stable, 1 is increase, 2 is decrease 3 is bad.
 * PAR is parity, when 1 parity is OK.
 * conts number of parity errors */
// void encoderMsgDecode(uint8_t * buf)
// { // decode bytes from AS5045 magnetic encoder
//   // left
//   uint8_t par;
//   encL_MAG = ((buf[1] & 0x03) << 2) + (buf[2] >> 6);
//   encL = (((uint16_t)buf[0] & 0x7f) << 5) + (buf[1] >> 3);
//   encLsum += encL;
//   par  = parity_even_bit(encL & 0xff);
//   par += parity_even_bit(encL >> 8) << 1;
//   par += parity_even_bit(encL_MAG) << 2;
//   par1 = par;
//   par = parity_even_bit(par);
//   encL_PAR = ((buf[2] & 0x20) != 0) == par;
//   if (encL_PAR == 0)
//     encLparErr++;
//   // right
//   encR_MAG = (buf[4]>> 3) & 0x0f;
//   encR = (((uint16_t)buf[2] & 0xf) << 8) + buf[3];
//   encRsum += encR;
//   par  = parity_even_bit(encR & 0xff);
//   par += parity_even_bit(encR >> 8) << 1;
//   par += parity_even_bit(encR_MAG) << 2;
//   par2 = par;
//   par = parity_even_bit(par);
//   encR_PAR = ((buf[4] & 0x04) != 0) == par;
//   if (encR_PAR == 0)
//     encRparErr++;
//   // tilt
//   encT_MAG = buf[6] & 0x0f;
//   encT = ((((buf[4] & 0x1) << 8) + buf[5]) << 3) + (buf[6] >> 5);
//   encTsum += encT;
//   par  = parity_even_bit(encT & 0xff);
//   par += parity_even_bit(encT >> 8) << 1;
//   par += parity_even_bit(encT_MAG) << 2;
//   par3 = par;
//   par = parity_even_bit(par);
//   encT_PAR = ((buf[7] & 0x80) != 0) == par;
//   if (encT_PAR == 0)
//     encTparErr++;
//   if (encDataCnt >= ENC_SUM_VALUES - 1)
//   { // deliver read result (as 14 bit)
//     encLx4 = encLsum;
//     encRx4 = encRsum;
//     encTx4 = encTsum;
//   }
//   encDataCnt++;
// }


/**
 * ADC interrupt routine */
ISR(ADC_vect)
{
  // start new conversion
  //ADCSRA = AD_START_CONV;
  sei();
}

/**
 * Send string - and wait until send
 * \param str is the string to send */
void sendString(const char * str)
{
  uint16_t n = strlen(str);
  usb_serial_write((uint8_t *) str, n);
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
void sendStatus(int8_t sendall)
{
  if (dxlReg[0][DXL_PRESENT_VOLT] > 0 || sendall)
  {
    snprintf(s5, MSTL, "Y0 %x %x %x %x %x %x %x ",
             *(uint16_t*)&dxlReg[0][DXL_PRESENT_POS],
             *(uint16_t*)&dxlReg[0][DXL_PRESENT_VEL],
             *(uint16_t*)&dxlReg[0][DXL_PRESENT_LOAD],
             dxlReg[0][DXL_PRESENT_VOLT] & 0xff,
             dxlReg[0][DXL_PRESENT_TEMP] & 0xff,
             dxlReg[0][DXL_PRESENT_ALARM] & 0xff,
             servoErrCnt[0]
            );
    sendStringLn(s5);
  }
  if (dxlReg[1][DXL_PRESENT_VOLT] > 0 || sendall)
  {
    snprintf(s5, MSTL, "Y1 %x %x %x %x %x %x %x",
              *(uint16_t*)&dxlReg[1][DXL_PRESENT_POS],
              *(uint16_t*)&dxlReg[1][DXL_PRESENT_VEL],
              *(uint16_t*)&dxlReg[1][DXL_PRESENT_LOAD],
              dxlReg[1][DXL_PRESENT_VOLT] & 0xff,
              dxlReg[1][DXL_PRESENT_TEMP] & 0xff,
              dxlReg[1][DXL_PRESENT_ALARM] & 0xff,
              servoErrCnt[1]
              );
   sendStringLn(s5);
  }
  if (dxlReg[2][DXL_PRESENT_VOLT] > 0 || sendall)
  {
    snprintf(s5, MSTL, "Y2 %x %x %x %x %x %x %x",
              *(uint16_t*)&dxlReg[2][DXL_PRESENT_POS],
              *(uint16_t*)&dxlReg[2][DXL_PRESENT_VEL],
              *(uint16_t*)&dxlReg[2][DXL_PRESENT_LOAD],
              dxlReg[2][DXL_PRESENT_VOLT] & 0xff,
              dxlReg[2][DXL_PRESENT_TEMP] & 0xff,
              dxlReg[2][DXL_PRESENT_ALARM] & 0xff,
              servoErrCnt[2]
              );
   sendStringLn(s5);
  }
  if (dxlReg[3][DXL_PRESENT_VOLT] > 0 || sendall)
  {
    snprintf(s5, MSTL, "Y3 %x %x %x %x %x %x %x",
              *(uint16_t*)&dxlReg[3][DXL_PRESENT_POS],
              *(uint16_t*)&dxlReg[3][DXL_PRESENT_VEL],
              *(uint16_t*)&dxlReg[3][DXL_PRESENT_LOAD],
              dxlReg[3][DXL_PRESENT_VOLT] & 0xff,
              dxlReg[3][DXL_PRESENT_TEMP] & 0xff,
              dxlReg[3][DXL_PRESENT_ALARM] & 0xff,
              servoErrCnt[3]
              );
   sendStringLn(s5);
  }
  if (dxlReg[4][DXL_PRESENT_VOLT] > 0 || sendall)
  {
    snprintf(s5, MSTL, "Y4 %x %x %x %x %x %x %x",
              *(uint16_t*)&dxlReg[4][DXL_PRESENT_POS],
              *(uint16_t*)&dxlReg[4][DXL_PRESENT_VEL],
              *(uint16_t*)&dxlReg[4][DXL_PRESENT_LOAD],
              dxlReg[4][DXL_PRESENT_VOLT] & 0xff,
              dxlReg[4][DXL_PRESENT_TEMP] & 0xff,
              dxlReg[4][DXL_PRESENT_ALARM] & 0xff,
              servoErrCnt[4]
              );
   sendStringLn(s5);
  }
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
  DDRB = (1 << PB2) | (1<<PB1) | (1 << PB0);
  PORTB |= (1 << PB3);   //Enable pull-up on MISO pin
  PORTB |= (1 << PB0); // set SS to 1 (tell slaves to disable)
  // SPI interrupt, SPI as master, set SPI clock rate fck/64
  SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << SPR1);
  SPSR = 0; // not double speed (bit 0 writable only)

  // serial setup for dynamixel
  DDRD  = (1 << PD3) | (1 << PD6);  //Set Port D bit 3 (TX) and bit 6 (LED) to outputs - all other input
  PORTD = (1 << PD2);  //Enable the pull-up on Rx (not needed, we have an external)
  // returns baudrate divisor
  // as is the same as
  dxlBaudrateDivisor = dxl_initialize(dxlBaudRate);

  //Init Timer 1 for CTC mode
   TCCR1A = 0; //Set no output pin effect
   TCCR1B = (1 << WGM12) | (1 << CS10);  //Set CTC mode, clk_IO/1 prescaler
   OCR1A = 0xfffe; // initial value
   TIMSK1 = (1 << OCIE1A); // on compare A (timer reset)

  //Initialize the ADC for Reads to control motor shut-down in case of over current
//   ADMUX = 0;    //Set the ADC channel to 0.
//   ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Enable the ADC, auto-triggering, the ADC Interrupt and set the ADC prescaler to 64                                                                                                                                    //ADC clock = 250kHz
//   ADCSRA |= (1<<ADSC);  //Start the first conversion

  // make the pin C0..C4 an output used to control servo tack direction
  // and pin C7 is debug signal for dxl talk (no encoder read)
  DDRC |= (1 << PINC7);
  // port F pin 6 (1=RX) pin 7 (1=tx) for half duplex direction control
  DDRF |= ((1 << PF6) | (1 << PF7));
}


// Basic command interpreter for controlling port pins
int main(void)
{
#define BUF_SIZE 32
  char buf[BUF_SIZE];
  uint8_t n;
  uint8_t err[DXL_SERVO_CNT];
  uint8_t statusTime = 0;

  // set for 16 MHz clock, and turn on the LED
  CPU_PRESCALE(0);
  LED_CONFIG; // D6 as output (LED)
  LED_ON;     // turn LED on
  //
  // init communication with dynamixel
  ioinit();
  // get main status from each servo
  for (n = 0; n < DXL_SERVO_CNT; n++)
  { // get EEPROM data
    int i, m;
    dxl_set_txpacket_id(dxlIDList[n]);
    dxl_set_txpacket_instruction(INST_READ);
    dxl_set_txpacket_length(4);
    dxl_set_txpacket_parameter(0, 0);
    dxl_set_txpacket_parameter(1, 19);
    timer1Cnt = 0;
    dxl_hal_clear();
    dxl_txrx_packet();
    err[n] = dxl_get_result();
    timer1CntReg[n] = timer1Cnt;
    if (err[n] == COMM_RXSUCCESS)
    { // seems to be OK
      m = dxl_get_rxpacket_length();
      if (m > DXL_REG_CNT || m < 0)
        m = DXL_REG_CNT;
      for (i = 0; i < m; i++)
        dxlReg[n][i] = dxl_get_rxpacket_parameter(i);
      for (i = m; i < DXL_REG_CNT; i++)
        dxlReg[n][24 + i] = n + 1;
      // get ram data
      dxl_set_txpacket_id(dxlIDList[n]);
      dxl_set_txpacket_instruction(INST_READ);
      dxl_set_txpacket_length(4);
      dxl_set_txpacket_parameter(0, 24);
      dxl_set_txpacket_parameter(1, 49 - 24 + 1);
      dxl_txrx_packet();
      // assume this is OK too
      m = dxl_get_rxpacket_length();
      if (m > DXL_REG_CNT || m < 0)
        m = DXL_REG_CNT;
      for (i = 0; i < m; i++)
        dxlReg[n][24 + i] = dxl_get_rxpacket_parameter(i);
      for (i = m; i < DXL_REG_CNT; i++)
        dxlReg[n][24 + i] = n + 1;
    }
  }
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

    // print a nice welcome message
    send_str(PSTR("Dynamixel MX-28T USB interface, " REV_ID "\r\n"));
    for (n = 0; n < DXL_SERVO_CNT; n++)
    { // send full status
      int i;
      snprintf(s5, MSTL, "Servo %d (ID=%d) rxok: %d (%s timeout: %d) alarm %x", n, dxlIDList[n], err[n],
               dxl_get_result_str(err[n]), timer1CntReg[n], dxlReg[n][DXL_PRESENT_ALARM]);
      sendStringLn(s5);
      if (err[n] == COMM_RXSUCCESS)
      { // send (almost) all register values 
        for (i = 0; i < 47; i++)
        {
          if (i == 19)
            i = 24;
          if (i == 9)
            i++;
          snprintf(s5, BUF_SIZE, "reg %2u %2x\n\r", i, dxlReg[n][i] & 0xff);
          sendString(s5);
        }
      }
    }
    n = 0;
    // and then listen for commands and process them
    while (1)
    {
      uint8_t m;
      char c;
      // get index to potentially filled encoder buffer
//      uint8_t encb = (encBufN - 1) & 0x3;
      // encoder PCI interface
//       if (encBufOK[encb])
//       { // new encoder message is available
//         encoderMsgDecode(encBuf[encb]);
//         encBufOK[encb] = 0; // buffer used
//         if (pushStatus)
//           sendStatus();
//       }
      // send data to servo
      if (dxlTalkOK)
      {
        if (dxlCmdTx == 1)
        { // command waiting a value
          uint16_t got = 0;
          if (dxlWord)
          {
            if (dxlRead)
              got = dxl_read_word(dxlID, dxlAdr);
            else
              dxl_write_word(dxlID, dxlAdr, dxlTXVal);
          }
          else
          {
            if (dxlRead)
              got = dxl_read_byte(dxlID, dxlAdr);
            else
              dxl_write_byte(dxlID, dxlAdr, dxlTXVal);
          }
          if (dxl_get_result() > COMM_RXSUCCESS)
            servoErrCnt[dxlID]++;
          if (0)
          { // debug to putty
            if (dxlWord && dxlRead)
              c = 'R';
            else if (!dxlWord && dxlRead)
              c = 'r';
            else if (dxlWord && !dxlRead)
              c = 'W';
            else
              c = 'w';
            // is send
            if (dxlRead)
              snprintf(s5, MSTL, "dxl %c id %2d, adr %2d=%d ok=%d (1=OK)", c, dxlID, dxlAdr, got, dxl_get_result());
            else
              snprintf(s5, MSTL, "dxl %c id %2d, adr %2d set to %d ok=%d", c, dxlID, dxlAdr, dxlTXVal, dxl_get_result());
            sendStringLn(s5);
          }
          dxlCmdTx = 0;
          dxlCmdRx = 1; // not really needed
          statusTime = 0;
        }
        if (statusTime)
        { // get present status from one of the servos
          int i;
          for (i = 0; i < DXL_SERVO_CNT - 1; i++)
          { // find next active servo
            dxlStatusID++;
            if (dxlStatusID >= DXL_SERVO_CNT)
              dxlStatusID = 0;
            if (dxlReg[dxlStatusID][DXL_PRESENT_VOLT] > 0)
              break;
            // debug
            // break;
            // debug end
          }
          dxl_set_txpacket_id(dxlIDList[dxlStatusID]);
          dxl_set_txpacket_instruction(INST_READ);
          dxl_set_txpacket_length(4);
          dxl_set_txpacket_parameter(0, 36);
          dxl_set_txpacket_parameter(1, 8);
          dxl_txrx_packet();
          if (dxl_get_result() == COMM_RXSUCCESS)
          {
            for (i = 0; i < 8; i++)
              dxlReg[dxlStatusID][i + 36] = dxl_get_rxpacket_parameter(i);
          }
          statusTime = 0;
        }
      }
      else
        statusTime = 1;
      // get number of available chars
      m = usb_serial_available();
      if (m > 0  && dxlCmdTx == 0)
      { // something on the USB channel and servo command buffer is clear
        buf[n] = usb_serial_getchar();
        if (localEcho)
          usb_serial_putchar(buf[n]);
        buf[n + 1] = '\0';
        if (buf[n] == '\n' || buf[n] == '\r')
        { // there is a command terminated with either
//           if (localEcho)
//             send_str(PSTR("\r\n"));   //\f
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
      if (pushStatus && lastStatusCnt > 32)
      {
        sendStatus(statusAll);
        lastStatusCnt = 0;
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
    "Dynamixel USB interface " REV_ID "\r\n"
    "  F6=1   Write 1 to port pin F6\r\n"
    "  D6=1   Write 1 Port D pin 6 (D6 is LED pin)\r\n"
    "  B4=0   Get status for switches (pin B4 not used)\r\n"
    "  i=1    Interactive - use local echo of all commands\r\n"
    "  j=1    Snd status for all servos - also inactive\r\n"
    "  r=a,b  dxl read byte from servo [0,1] a at address (decimal)\r\n"
    "  R=a,b  dxl read word from servo [0,1] a at address (decimal)\r\n"
    "  w=a,b,c  dxl write byte to servo [0,1] a at address b to value c\r\n"
    "  W=a,b,c  dxl write word to servo [0,1] a at address b to value c\r\n"
    "  s[=1]    Send status (1= push status, 0= stop)\r\n"
    "  help   This help text\r\n"
    );
//   if (num < 3)
//   { // too short
//     send_str(PSTR("unrecognized format, 3 chars min req'd\r\n"));
//     return;
//   }
  // first character is the port letter
  if (buf[0] >= 'A' && buf[0] <= 'F')
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
    statusAll = buf[2] == '1';
    isOK = 1;
  }
  else if (buf[0] == 's')
  {
    if (buf[1] == '=')
    {
      if (buf[2] == '1')
        pushStatus = 1;
      else
        pushStatus = 0;
    }
    if (pushStatus == 0)
      sendStatus(1);
    isOK = 1;
  }
  else if (buf[0] == 'h' || buf[0] == 'H')
  {
    send_str(help);
    isOK = 1;
  }
  else if ((buf[0] == 'r' || buf[0] == 'R') && buf[1] == '=')
  { // read operation (byte or word)
    char *p1;
    p1 = (char*)&buf[2];
    uint8_t id = strtol(p1, &p1,0);
    isOK = (*p1++ == ',');
    if (isOK)
    { // valid request - set request
      dxlCmdRx = 0;
      dxlRead = 1;
      dxlCmdTx = 0;
      if (id >= 0 && id < DXL_SERVO_CNT)
        dxlID = dxlIDList[id];
      else
        dxlID = 0;
      dxlAdr = strtol(p1, &p1, 0);
      dxlWord = buf[0] == 'R';
      dxlRxVal = 0;
      // request is complete
      dxlCmdTx = 1;
    }
  }
  else if ((buf[0] == 'w' || buf[0] == 'W') && buf[1] == '=')
  { // write operation (byte or word)
    char *p1;
    p1 = (char*)&buf[2];
    uint8_t id = strtol(p1, &p1,0);
    isOK = (*p1++ == ',');
    if (isOK)
    { // valid request - set request
      dxlCmdRx = 0;
      dxlRead = 0;
      dxlCmdTx = 0;
      if (id >= 0 && id < DXL_SERVO_CNT)
        dxlID = dxlIDList[id];
      else
        dxlID = 0;
      dxlAdr = strtol(p1, &p1, 0);
      p1++;
      dxlTXVal = strtol(p1,&p1, 0);
      dxlWord = buf[0] == 'W';
      dxlRxVal = 0;
      // request is complete - flag implement
      dxlCmdTx = 1;
    }
  }
  else
  {
    send_str(PSTR("Unknown command \""));
    usb_serial_putchar(buf[0]);
    send_str(PSTR("\"\r\n"));
    send_str(help);
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
}

