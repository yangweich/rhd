/** \file globalfunc.c
 *  \ingroup core
 *  \brief Globally used functions

 *  Global functions for serial/socket RX/TX and set serial etc..
 *
 *  \author Anders Billesø Beck
 *  $Rev: 912 $
 *  $Date: 2010-04-12 11:14:16 +0200 (Mon, 12 Apr 2010) $
 */
 /***************************************************************************
 *                  Copyright 2010 Anders Billesø Beck                     *
 *                       anders.beck@get2net.dk                            *
 *                                                                         *
 ***************************************************************************/
/************************** Version control information ***************************/
 #define VERSION            "$Rev: 912 $:"
 #define DATE               "$Date: 2010-04-12 11:14:16 +0200 (Mon, 12 Apr 2010) $:"
/**********************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <sys/types.h>
#include <sys/socket.h>


#include "globalfunc.h"


/** \brief Make sure that all is written to socket
 * 
 * \param[in] int fd
 * \param[in] int buf
 * \param[in] size_t txLen
 * 
 * \returns ssize_t txLen
 */
ssize_t secureWrite(int fd, const void *buf, ssize_t txLen) {

  int i, acc = 0;
  while (acc < txLen) {
    i = write(fd, buf+acc, txLen - acc);
    if (i <= 0) return i;
    acc += i;
  }
  return (ssize_t)acc;
}


/** \brief Make sure that all is written any file
 * 
 * \param[in] int fd
 * \param[in] int buf
 * \param[in] size_t txLen
 * 
 * \returns ssize_t txLen
 */
ssize_t secureSend(int fd, const void *buf, ssize_t txLen) {

  int i, acc = 0;
  while (acc < txLen) {
    i = send(fd, buf+acc, txLen - acc, MSG_NOSIGNAL); //Ignore SIGPIPE
    if (i <= 0) return i;
    acc += i;
  }
  return (ssize_t)acc;
}

/** \brief Make sure that all is read from socket
 *
 * \param[in] int fd
 * \param[in] int buf
 * \param[in] size_t txLen
 *
 * \returns ssize_t txLen
 */
ssize_t secureRecv(int fd, void *buf, ssize_t txLen) {

  int i, acc = 0;
  while (acc < txLen) {
    i = recv(fd, buf+acc, txLen - acc, MSG_NOSIGNAL);
    if (i <= 0) return i;
    acc += i;
  }
  return (ssize_t)acc;
}

/** \brief Make sure that all is read any file
 * 
 * \param[in] int fd
 * \param[in] int buf
 * \param[in] size_t txLen
 * 
 * \returns ssize_t txLen
 */
ssize_t secureRead(int fd, void *buf, ssize_t txLen) {

  int i, acc = 0;
  while (acc < txLen) {
    i = read(fd, buf+acc, txLen - acc);
    if (i <= 0) return i;
    acc += i;
  }
  return (ssize_t)acc;
}

/** \brief Setup serial port
 * 
 * \param[in] int fd
 * \param[in] int baud
 * 
 * \returns success
 */
int set_serial(int fd, int baud) {
  struct termios tios;

  if (tcgetattr(fd, &tios))  return -1;

  tios.c_iflag =
    IGNPAR;                     /* ignore framing errors (read as 0) */
  tios.c_oflag = 0;
  tios.c_cflag = (CS8
                  | CREAD       /* enable receiver ? */
                  | CLOCAL      /* ignore modem control lines */
                  );
  tios.c_lflag = 0;

//you can easily add more baud options if you need them
  if(baud == 115200)  {
    cfsetospeed (&tios, B115200);
    cfsetispeed (&tios, B115200);
  } else if(baud == 57600) {
    cfsetospeed (&tios, B57600);
    cfsetispeed (&tios, B57600);
  } else if(baud == 38400) {
    cfsetospeed (&tios, B38400);
    cfsetispeed (&tios, B38400);
  } else if(baud == 19200) {
    cfsetospeed (&tios, B19200);
    cfsetispeed (&tios, B19200);
  } else if(baud == 9600) {
    cfsetospeed (&tios, B9600);
    cfsetispeed (&tios, B9600);
  } else if(baud == 4800) {
    cfsetospeed (&tios, B4800);
    cfsetispeed (&tios, B4800);
  } else if(baud == 2400) {
    cfsetospeed (&tios, B2400);
    cfsetispeed (&tios, B2400);
  } else if(baud == 1200) {
    cfsetospeed (&tios, B1200);
    cfsetispeed (&tios, B1200);
  } else if(baud == 300) {
    cfsetospeed (&tios, B300);
    cfsetispeed (&tios, B300);
  } else return -1;

  if (tcsetattr(fd, TCSANOW, &tios))  return -1;

// request low-latency from kernel
    struct serial_struct ss;
    ioctl(fd, TIOCGSERIAL, &ss);
    ss.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &ss);

  return 0;
}

