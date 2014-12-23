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
#define REV "$Rev: 473 $"
#define REV_ID "$Id: steeruc.c 473 2014-04-14 17:57:02Z jcan $"

/**
 * Controller for steering servo and magnetic encoders
 * for DTU field robot */

// makes the interrupt routine enable further (nested) interrupts / chr
//#define ISR_NOBLOCK

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
int localEcho = 0;
uint8_t timingChar = 'a';
uint8_t pushStatus = 0; // 1 = send status automatically to USB-hostwhen available
uint8_t pushCnt = 0;

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
uint8_t enableMagEnc = 1; // enable magnetic encoder communication
uint8_t enableDynamixel = 1; // enable communication with dynamixel
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
uint8_t encReadCnt, encDataCnt;
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
uint16_t timer1Cnt = 0;
#define MSTL 190
char s5[MSTL];

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
#define DXL_SERVO_CNT 2
#define DXL_REG_CNT   50
int8_t dxlReg[DXL_SERVO_CNT][DXL_REG_CNT];
#define DXL_PRESENT_POS  36
#define DXL_PRESENT_VEL  38
#define DXL_PRESENT_LOAD 40
#define DXL_PRESENT_VOLT 42
#define DXL_PRESENT_TEMP 43
#define DXL_PRESENT_LED 17
#define DXL_PRESENT_ALARM 18
float dxlBaudRate = 500000.0; // in bit/sec, default is 57600 bit/s
int8_t dxlIDList[DXL_SERVO_CNT] = {1, 2};
//int8_t dxlIDList[DXL_SERVO_CNT] = {1, 1};
uint8_t err[DXL_SERVO_CNT];
uint8_t errCom[DXL_SERVO_CNT] = {0,0};
// debug value
int8_t dxlBaudrateDivisor;
// request status from this servo
int8_t dxlStatusID = 0;
/// debug values
uint16_t timer1CntReg[DXL_SERVO_CNT];
uint16_t servoErrCnt[DXL_SERVO_CNT] = {0,0};
/**
 * value to set for wheel servos */
uint16_t dxlWheelLeft = 2048;
uint16_t dxlWheelRight = 2048;
/**
 * flag to set new values 0=is set 1=new values to set */
uint8_t  dxlWheelUpd = 0;
uint8_t admuxCnt = 2;
uint16_t adValue[8];
uint8_t sensorCnt;


/**
 * SPI data transfer complete interrupt (8 bit) */
ISR(SPI_STC_vect)
{ // save data from AS5045 magnetic encoder
//  PORTD |= 0x40;
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
//  PORTD &= ~0x40;
}

///
/** timer 1 (16 bit) interrupt
 * this is used to split time into encoder read and communication with servos
 * state 1 and 2 is reserved time for servo communication,
 * state 3 and 4 is encoder read states */
ISR(TIMER1_COMPA_vect)
{
  // increase timer for simplex receive timeout
  //gwCountNum +=OCR1A;
//  sei();
  // debug timer
  timer1Cnt++;
  // do encoder read sycles
  switch (timerState)
  {
  case 1: // time to talk to servos
     OCR1A = 100 * TIMER_A_CLOCK; // (gfByteTransTime_us * TIMER_A_CLOCK); // wait on character
     //gwRxTimeout = 0; // time to transmit/receive from dynamixel
     dxlTalkOK = enableDynamixel;
     PORTC |= 0x80;
     //timer1Cnta = TCNT2;
     space4dxlTalkCnt = 0;
    break;
  case 2: // stop talking to servos
    if (enableDynamixel)
    {
      space4dxlTalkCnt++;
      if (space4dxlTalkCnt < 23) // 15:~3ms 25:~5ms, 50:~ 7.5ms//space4dxlTalk)
        // stay here one cycle more
        timerState--;
      //timer1Cntb = TCNT2;
    }
    break;
  case 3: // encoder read start
    // end dxl talk time and talk to encoder
    dxlTalkOK = 0;
    if (timer1Cnt > 550)
      // use long timeout durig startup (approx 250 ms)
      gwRxTimeout = 1;
    PORTC &= ~0x80;
    // start encoder read
    encReadCnt = 0;
    encDataCnt = 0;
    break;
  case 4: // encoder read
    OCR1A = ENCODER_READ_CNT_N; // ~400us sample rate for encoder 2kHz
    if (enableMagEnc)
    { // encoder is not disabled, so start communication
      // one read reads all encoders (daisy-chained)
      // reads 4 times to get better values
      if (encBufCnt == 0)
      {  // start new read
        PORTB &= ~(1 << PB0); // 0xfb;  // start SPI, set SS to low
        PORTB &= ~(1 << PB0); // wait at least 0.5 us
        PORTB &= ~(1 << PB0); // wait at least 0.5 us
        SPDR = 0x3C;
      }
      if (encReadCnt < ENC_SUM_VALUES)
        // stay in read mode for (4) reads
        timerState--;
      encReadCnt++;
    }
    //timer1Cntd = TCNT2;
    pushCnt++;
    break;
  case 5:
    // start AD conversion series
    admuxCnt = 0;
    // start new conversion
    ADMUX = ((1<<REFS1) | (1<<REFS0)) + admuxCnt;
    ADCSRA |= 1 << ADSC;    
    break;
  default: // finished - restart
    timerState = 0;
    OCR1A = 100 * TIMER_A_CLOCK; // ~1ms wait after encoder read
    //timer1Cnte = TCNT2;
    break;
  }
  timerState++;
}

/**
 * Decode message from 1..3 AS5045 encoders received in one buffer (encBuf)
 * encoder value is stored in a raw encoder value buffer encXRaw, to
 * allow filtering of stary error values (buffer with 5 values).
 * OCF when 1: offset compensation finished is good
 * COF Cordic overflow - when 1:value is form last good measurement
 * LIN linearity is bad, when 1: magnet alignment out of order
 * MAG when 0 magnets are good and stable, 1 is increase, 2 is decrease 3 is bad.
 * PAR is parity, when 1 parity is OK.
 * conts number of parity errors */
void encoderMsgDecode(uint8_t * buf)
{ // decode bytes from AS5045 magnetic encoder
  // left
  uint8_t par;
  encL_MAG = ((buf[1] & 0x07) << 2) + ((buf[2] >> 6) & 0x03);
  encL = (((uint16_t)buf[0] & 0x7f) << 5) + ((buf[1] >> 3) & 0x1f);
  encLsum += encL;
  par  = parity_even_bit(encL & 0xff);
  par += parity_even_bit(encL >> 8) << 1;
  par += parity_even_bit(encL_MAG) << 2;
  par1 = par;
  par = parity_even_bit(par);
  encL_PAR = ((buf[2] & 0x20) != 0) == par;
  if (encL_PAR == 0)
    encLparErr++;
  // right
  encR_MAG = buf[4] >> 3;
  encR = (((uint16_t)buf[2] & 0xf) << 8) + buf[3];
  encRsum += encR;
  par  = parity_even_bit(encR & 0xff);
  par += parity_even_bit(encR >> 8) << 1;
  par += parity_even_bit(encR_MAG) << 2;
  par2 = par;
  par = parity_even_bit(par);
  encR_PAR = ((buf[4] & 0x04) != 0) == par;
  if (encR_PAR == 0)
    encRparErr++;
  // tilt
  encT_MAG = buf[6] & 0x1f;
  encT = ((((buf[4] & 0x1) << 8) + buf[5]) << 3) + ((buf[6] >> 5) & 0x07);
  encTsum += encT;
  par  = parity_even_bit(encT & 0xff);
  par += parity_even_bit(encT >> 8) << 1;
  par += parity_even_bit(encT_MAG) << 2;
  par3 = par;
  par = parity_even_bit(par);
  encT_PAR = ((buf[7] & 0x80) != 0) == par;
  if (encT_PAR == 0)
    encTparErr++;
  if (encDataCnt >= ENC_SUM_VALUES - 1)
  { // deliver read result (as 14 bit)
    encLx4 = encLsum;
    encLsum = 0;
    encRx4 = encRsum;
    encRsum = 0;
    encTx4 = encTsum;
    encTsum = 0;
  }
  encDataCnt++;
}


/**
 * ADC interrupt routine */
ISR(ADC_vect)
{
  adValue[admuxCnt] = ADC;
  // set mux to new channel
  admuxCnt++;
  if (admuxCnt <= 1)
  { // start new conversion, if more channels to test
    ADMUX = ((1<<REFS1) | (1<<REFS0)) + admuxCnt;
    // start new conversion
    ADCSRA |= 1 << ADSC;
  }
  else
  {
    sensorCnt++;
  }
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

/**
 * Send string and at the end send a LFCR and flush output to USB host
 * */
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
//   if (PORTD & (1 << PD6))
//     PORTD &= ~(1 << PD6);
//   else
//     PORTD |=  (1 << PD6);
  uint8_t n = 0;
//  uint8_t bufIdx = (encBufN - 1) & 0x3;
  if (enableMagEnc)
  {
    snprintf(s5, MSTL, "E%3x %3x %3x "/*, %3x,%3x,%3x"*/, encLx4, encRx4, encTx4 /*, encL, encR, encT*/);
    n = strlen(s5);
    // magnet status
    snprintf(&s5[n], MSTL-n, "M%2x %2x %2x ", encL_MAG & 0x0f, encR_MAG & 0x0f, encT_MAG & 0x0f);
    n += strlen(&s5[n]);
    // parity error for encoders
    snprintf(&s5[n], MSTL-n, "P%x %x %x ", encLparErr, encRparErr, encTparErr);//, encDataCnt, encReadCnt, spiCnt, timer1Cnt);
    n += strlen(&s5[n]);
    // raw encoder buffer
//     snprintf(&s5[n], MSTL-n, "R%2x %2x %2x %2x %2x %2x %2x %2x", encBuf[bufIdx][0], encBuf[bufIdx][1],
//                                                          encBuf[bufIdx][2], encBuf[bufIdx][3],
//                                                          encBuf[bufIdx][4], encBuf[bufIdx][5],
//                                                          encBuf[bufIdx][6], encBuf[bufIdx][7]);
//     n += strlen(&s5[n]);

  }
  // emergency switch pushed and battery voltage
  snprintf(&s5[n], MSTL-n, "N%x %x\n\r", !(PINE & 0x01), adValue[0]);
  n += strlen(&s5[n]);
  if (enableDynamixel)
  { // dynamixel error counts
    snprintf(&s5[n], MSTL-n, "Y0 %x %x %x %x %x %x %x ",
             *(uint16_t*)&dxlReg[0][DXL_PRESENT_POS],
             *(uint16_t*)&dxlReg[0][DXL_PRESENT_VEL],
             *(uint16_t*)&dxlReg[0][DXL_PRESENT_LOAD],
             dxlReg[0][DXL_PRESENT_VOLT] & 0xff,
             dxlReg[0][DXL_PRESENT_TEMP] & 0xff,
             dxlReg[0][DXL_PRESENT_LED] & 0xff,
             servoErrCnt[0]
            );
    n += strlen(&s5[n]);
    snprintf(&s5[n], MSTL-n, "Y1 %x %x %x %x %x %x %x ",
              *(uint16_t*)&dxlReg[1][DXL_PRESENT_POS],
              *(uint16_t*)&dxlReg[1][DXL_PRESENT_VEL],
              *(uint16_t*)&dxlReg[1][DXL_PRESENT_LOAD],
              dxlReg[1][DXL_PRESENT_VOLT] & 0xff,
              dxlReg[1][DXL_PRESENT_TEMP] & 0xff,
              dxlReg[1][DXL_PRESENT_LED] & 0xff,
              servoErrCnt[1]
              );
    n += strlen(&s5[n]);
    // and debug error counts
    snprintf(&s5[n], MSTL-n, "F%x %x %x %x %x %x %x\n\r", dxlErrSuccess,
                          dxlErrTimeout, dxlErrCurrupt, dxlRxTimeoutCnt,
                          dxlRxTimeoutCntMax, dxlBusUseCnt, dxlBusAlreadyInUseCnt);
    n += strlen(&s5[n]);
  }
  if (n > MSTL)
    n = MSTL;
  usb_serial_write((uint8_t *) s5, n);
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

  //Set Port D bit 3 (TX) and debug bit 5, 6 (LED) and 7 to outputs - all other input
  DDRD  = (1 << PD3) | (1 << PD5) | (1 << PD6) | (1 << PD7);
  PORTD = (1 << PD2);  //Enable the pull-up on Rx (not needed, we have an external)
  // returns baudrate divisor
  // as is the same as
  dxlBaudrateDivisor = dxl_initialize(dxlBaudRate);

  //Init Timer 1 for CTC mode
   TCCR1A = 0; //Set no output pin effect
   TCCR1B = (1 << WGM12) | (1 << CS10);  //Set CTC mode, clk_IO/1 prescaler
   OCR1A = 0xfffe; // initial value
   TIMSK1 = (1 << OCIE1A); // on compare A (timer reset)

  //Initialize the ADC for read of battery voltage
  ADMUX = ((1<<REFS1) | (1<<REFS0)) + admuxCnt;    // use internal 2.56V reference. Set the ADC channel to 0.
  ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1); //Enable the ADC, the ADC Interrupt and set the ADC prescaler to 64 (about 0.1ms/conversion)
  ADCSRA |= (1<<ADSC);  //Start the first conversion
  //Initialize the ADC for Reads to control motor shut-down in case of over current
//   ADMUX = 0;    //Set the ADC channel to 0.
//   ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Enable the ADC, auto-triggering, the ADC Interrupt and set the ADC prescaler to 64                                                                                                                                    //ADC clock = 250kHz
//   ADCSRA |= (1<<ADSC);  //Start the first conversion

  // make the pin C0..C3 an output used to control servo talk direction
  // and pin C7 is debug signal for dxl talk (no encoder read)
//  DDRC |= ((1 << PINC0) || (1 << PINC1) || (1 << PINC2) || (1 << PINC3) || (1 << PINC7));
  DDRC = 0x8f;
}


void getServoStatus(void)
{
  int n;
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
    if (dxlReg[n][18] > 0)
    { // reset alarm
      dxl_set_txpacket_id(dxlIDList[n]);
      dxl_set_txpacket_instruction(INST_WRITE);
      dxl_set_txpacket_length(4);
      dxl_set_txpacket_parameter(0, 18);
      dxl_set_txpacket_parameter(1, 0);
      dxl_txrx_packet();
    }
  }
}

void getServoStatus2(int servo, int from, int cnt)
{
  int n = servo;
  int i, m = 0;
  dxl_set_txpacket_id(dxlIDList[n]);
  dxl_set_txpacket_instruction(INST_READ);
  dxl_set_txpacket_length(4);
  dxl_set_txpacket_parameter(0, from);
  dxl_set_txpacket_parameter(1, cnt);
  timer1Cnt = 0;
  dxl_hal_clear();
  dxl_txrx_packet();
  err[n] = dxl_get_result();
  if (err[n] == COMM_RXSUCCESS)
  { // seems to be OK
    m = dxl_get_rxpacket_length();
    for (i = 0; i < cnt; i++)
      dxlReg[n][i + from] = dxl_get_rxpacket_parameter(i);
  }
//   snprintf(s5, MSTL, "GS %d %d %d m=%d\n", servo, from, cnt, m);
//   sendString(s5);
}
/////////////////////////////////////////////////////

void sendServoStatus(void)
{
  int n;
  send_str(PSTR("Field robot steering, " REV_ID "\r\n"));
  for (n = 0; n < DXL_SERVO_CNT; n++)
  { // send full status
    int i;
    snprintf(s5, MSTL, "Servo %d rxok: %d (%s timeout: %d) alarm 0x%x", n, err[n],
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
        snprintf(s5, MSTL, "reg %2u %2x\n\r", i, dxlReg[n][i] & 0xff);
        sendString(s5);
      }
    }
  }
}

// Basic command interpreter for controlling port pins
int main(void)
{
#define BUF_SIZE 32
  char buf[BUF_SIZE];
  //char s12[12];
  uint8_t n;
  uint16_t loop = 0;
//  uint8_t err[DXL_SERVO_CNT];
  uint8_t statusTime = 0;

  // set for 16 MHz clock, and turn on the LED
  CPU_PRESCALE(0);
  LED_CONFIG; // D6 as output (LED)
  LED_ON;     // turn LED on
  //
  // init communication with dynamixel
  ioinit();
  // get main status from each servo
  getServoStatus();
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
    while (!(usb_serial_get_control() & USB_SERIAL_DTR))
      /* wait */ ;

    // discard anything that was received prior.  Sometimes the
    // operating system or other software will send a modem
    // "AT command", which can still be buffered.
    usb_serial_flush_input();

    // print a nice welcome message
    sendServoStatus();
    n = 0;
    // and then listen for commands and process them
    while (1)
    {
      uint8_t m;
      //char c;
      // get index to potentially filled encoder buffer
      uint8_t encb = (encBufN - 1) & 0x3;
      // encoder PCI interface
      if (encBufOK[encb])
      { // new encoder message is available
        encoderMsgDecode(encBuf[encb]);
        encBufOK[encb] = 0; // buffer used
//         if (encReadCnt == 4 && pushStatus && pushCnt > pushStatus)
//         {
//           sendStatus();
//           pushCnt = 0;
//         }
      }
      if (pushCnt > pushStatus && pushStatus)
      { // time to send a status message
        sendStatus();
        pushCnt = 0;
      }
      // send data to servo
      if (dxlTalkOK)
      { // talk once only in one cycle
        dxlTalkOK = 0;
        if (dxlWheelUpd)
        { // new steering wheel position
          PORTD |= 0x40;
          dxl_write_word(dxlIDList[0], 30, dxlWheelLeft);
          dxl_write_word(dxlIDList[1], 30, dxlWheelRight);
          PORTD &= ~0x40;
          dxlWheelUpd = 0;
        }
        else if (dxlCmdTx == 1)
        { // command waiting a value
          uint16_t got = 0;
          send_str(PSTR("cmd start\n\r"));
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
          dxlCmdTx = 0;
          dxlCmdRx = 1; // not really needed
          statusTime = 0;
          send_str(PSTR("cmd end\n\r"));

        }
        else if (statusTime)
        { // get present status from one of the servos
          if ((loop & 0xf) == 0)
          { // get alert bits
            getServoStatus2(0, DXL_PRESENT_LED, 2);
            getServoStatus2(1, DXL_PRESENT_LED, 2);
          }
          else
          { // ask for current status from register 36 and 8 registers
            dxlStatusID = ! dxlStatusID;
            getServoStatus2(dxlStatusID, 36, 8);
          }
          statusTime = 0;
        }
      }
      else
        statusTime = 1;
      loop++;
      // get number of available chars
      m = usb_serial_available();
      // debug - send m as a character
//       if (localEcho && m > 0)
//       {
//         snprintf(s12,12,"m=%d n=%d\n",m,n);
//         sendString(s12);
//       }
      // debug end
      while (m > 0  && dxlCmdTx == 0)
      { // something on the USB channel and servo command buffer is clear
        buf[n] = usb_serial_getchar();
        if (localEcho)
           usb_serial_putchar(buf[n]);
        buf[n + 1] = '\0';
        if (buf[n] == '\n' || buf[n] == '\r')
        { // there is a command terminated with either
          if (localEcho)
            send_str(PSTR("\r\n"));   //\f
          parse_and_execute_command(buf, n);
          if (localEcho)
            send_str(PSTR(">"));
          // clear command buffer
          n = 0;
          buf[n] = '\0';
          break;
        }
        else
          if (buf[n] >= ' ' && buf[n] <= '~')
            n++;
        if (n >= BUF_SIZE)
          // input error - restart
          n = 0;
        m--;
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
    "Field robot steering USB interface, " REV_ID "\r\n"
    "Control Shell\n\r\n"
    "  F6=1   Write 1 to port pin F6\r\n"
    "  D6=1   Write 1 Port D pin 6 (D6 is LED pin)\r\n"
    "  B4=0   Get status for switches (pin B4 not used)\r\n"
    "  i=1    Interactive - use local echo of all commands\r\n"
    "  j=a    set timing char - is returned as part of status\r\n"
    "  m=0    disable magnetic encoder communication (default is 1)\r\n"
    "  x=0    disable dynamixel communication (default is 1)\r\n"
    "  r=a,b  dxl read byte from servo a [0..255]at address (decimal)\r\n"
    "  R=a,b  dxl read word from servo a [0..255] at address (decimal)\r\n"
    "  K=L,R  set left and right servo to these values [0..4095] (decimal)\r\n"
    "  w=a,b,c  dxl write byte to servo a [0..255] at address b to value c\r\n"
    "  W=a,b,c  dxl write word to servo a [0..255] at address b to value c\r\n"
    "  s[=1]    Send status (1= push status, 0= stop)\r\n"
    "  reset  Get main status for all servos - are they alive\r\n"
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
      isOK = (wr < 4096 && wr < 4096);
      if (isOK && dxlWheelUpd == 0)
      {
        dxlWheelLeft = wl;
        dxlWheelRight = wr;
        dxlWheelUpd = 1;
      }
    }
//     snprintf(s5, MSTL, "K got %d,%d ok=%d s=%s\n",
//                       wl, wr, dxlWheelUpd, buf);
//     sendString(s5);
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
      pushStatus = strtol(&buf[2], NULL, 0);
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
    getServoStatus();
    sendServoStatus();
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
//      if (id >= 0 && id < DXL_SERVO_CNT)
        dxlID = id; // dxlIDList[id];
//      else
//        dxlID = 0;
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
//      if (id >= 0 && id < DXL_SERVO_CNT)
        dxlID = id; // dxlIDList[id];
//      else
//        dxlID = 0;
      dxlAdr = strtol(p1, &p1, 0);
      p1++;
      dxlTXVal = strtol(p1,&p1, 0);
      dxlWord = buf[0] == 'W';
      dxlRxVal = 0;
      // request is complete - flag implement
      dxlCmdTx = 1;
    }
  }
  else if ((buf[0] == 'm' || buf[0] == 'M') && buf[1] == '=')
  {
    char *p1;
    p1 = (char*)&buf[2];
    enableMagEnc = strtol(p1, &p1,0);
    isOK = 1;
  }
  else if ((buf[0] == 'x' || buf[0] == 'X') && buf[1] == '=')
  {
    char *p1;
    p1 = (char*)&buf[2];
    enableDynamixel = strtol(p1, &p1,0);
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

