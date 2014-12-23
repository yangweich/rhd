/**
 * test if interface to teensy steering up */
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <poll.h>
#include <math.h>
#include <stdint.h>

#include <globalfunc.h>

#define PLUGINNAME "teensy"

int ttyDev;
FILE * freLog;
#define MxBL 200
char rxBuf[MxBL];
char * p2 = rxBuf;
pthread_t controlio_thread;


///////////////////////////////////

int init()
{
  int result = 0;
  char * serialDev = "/dev/ttyACM0";
  ttyDev = open(serialDev, O_RDWR /*| O_NONBLOCK*/);
  result = ttyDev != -1;
  if (result == 0)
    fprintf(stderr,"   FSteer: Can't open device 2 (motor): %s\n", serialDev);
  // initialize rx buffer to empty
  p2 = rxBuf;
  *p2 = '\0';
  //
  return result;
}

///////////////////////////

int send2(int dev, char * msg)
{
  int n = write(dev, msg, strlen(msg));
  return n;
}

///////////////////////////////////////////////////////////

int pollDeviceRx(int fd, int msTimeout)
{
  int dataAvailable = 0;
  //
  if (fd == -1)
  { // no device - debugging mode
    usleep(50000);
  }
  else
  { // test for data
    int err = 0;
    struct pollfd pollStatus;
    int revents;
    //
    pollStatus.fd = fd;
    pollStatus.revents = 0;
    pollStatus.events = POLLIN  |  /*  0x0001  There is data to read */
                        POLLPRI;   /*  0x0002  There is urgent data to read */
                                  /* POLLOUT 0x0004  Writing now will not block */
    //
    err = poll(&pollStatus, 1, msTimeout);
    if (err < 0)
    { // not a valid call (may be debugger interrrupted)
      perror(PLUGINNAME ":pollDeviceRx (poll)");
    }
    else if (err > 0)
    { // at least one connection has data (or status change)
      revents = pollStatus.revents;
      if (((revents & POLLIN) != 0) ||
          ((revents & POLLPRI) != 0))
        dataAvailable = 1;
    }
  }
  return dataAvailable;
}

////////////////////////////////////////////////

int getNewLine(int timeoutms, int maxWaitCnt)
{
  int l = 0;
  int n = 0;
  char * p1 = rxBuf;
  char * p3;
  // wait for reply - up to 500ms
  // there may be stuff already,
  // remove used part, and append
  n = strlen(p2);
  if (n > 0)
  { // partial message received
    if (n > 0 && n < MxBL - 1)
      memmove(p1, p2, n);
    else
      n = 0;
    p1 = &rxBuf[n];
  }
  // zero terminate - for debug
  *p1 = '\0';
  p2 = rxBuf;
  // see if we already has the next line
  p3 = strchr(p2, '\n');
  // get some data if no new line end
  while ((l < maxWaitCnt) && (p3 == NULL))
  { // more data needed
    int dataOK;
    int m;
    l++; // loop counter
    // wait for data
    dataOK = pollDeviceRx(ttyDev, timeoutms);
    if (dataOK)
    { // there is data
      m = read(ttyDev, p1, MxBL - n - 1);
      if (m > 0)
      { // zero terminate and look for new-line
        p1[m] = '\0';
        p3 = strchr((char*)p1, '\n');
        if (p3 == NULL)
          p3 = strchr((char*)p1, '\r');
        n += m;
        p1 = &rxBuf[n];
      }
      else if (m < 0)
      { // poll error - device is removed?
        printf(PLUGINNAME ": Read error from device - device is lost\n");
        break;
      }
    }
  }
  if (p3 != NULL)
  { // terminate and advance p2 to first byte in next message
    *p3++ = '\0';
    while (*p3 < ' ' && *p3 > '\0')
      p3++;
    p2 = p3;
    //printf("# l=%d i=%d (%x) %s\n", strlen(rxBuf), l, (unsigned int)p3, rxBuf);
  }
//  printf("getData: got %d bytes in %d pools (timeout=%dms maxLoop=%d)\n",
//            n, l, timeoutms, maxWaitCnt);
  return strlen(rxBuf);
}


///RS232  control thread - for steering
void * controlio_task_1(void * not_used)
{ // run in thread (steer)
  int n;
  struct timeval logtime;
  //
  fprintf(stderr, "   FSteer: rx_task 1 running\n");
  // wait for RHD to init variables
  //usleep(300000);
  // set initial values of variables
  //Mark thread as running
  while (1)
  { // maintain interface
    // connection is open get next status message
    n = getNewLine(20, 22);
    if (n > 2)
    { // new data available)
      char * p1;

      //float speed;
      // debug
      //printf(PLUGINNAME ": %d get from device: %s\n", m, rxBuf);
      // debug end
      p1 = rxBuf;
      while (*p1 < ' ')
        p1++;
      // most likely a usable status message
      gettimeofday(&logtime, NULL);
      if (freLog != NULL)
      {
        fprintf(freLog, "%lu.%06lu if1 got %s\n", logtime.tv_sec, logtime.tv_usec, p1);
        fflush(freLog);
      }
      fprintf(stdout, "%lu.%06lu if1 got %s\n", logtime.tv_sec, logtime.tv_usec, p1);
    }
  }
  fprintf(stderr,PLUGINNAME ": closing bus device\n");
  close(ttyDev);
  fprintf(stderr,PLUGINNAME ": Shutting down thread\n");
  pthread_exit(0);
  return NULL;
}


///////////////////////////

///////////////////////////

int main(int argc, char * argv[])
{
  int ok;
  //
  freLog = fopen("fre.log", "w");
  struct timeval logtime;
  ok = init();
  printf("init returned %d (1=OK)\n", ok);
  if (ok)
  { // start receive thread
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&controlio_thread, &attr, controlio_task_1, 0))
    {
      perror(PLUGINNAME ": Can't start controlio 1 receive thread (steering)");
      ok = 0;
    }
  }
  if (ok)
  {
    int i;
    float a = 0;
    char s[100];
    float da = M_PI/2/200.0;
    for (i = 0; i < 100000000; i++)
    {
      if ( i % 500 == 0)
      {
        int ws = (int)(sin(a)*1024) + 2048;
  //      int wc = (int)(cos(a)*1024) + 2048;
        int m;
        a += da;
        if (a > 2*M_PI)
        a=0.0;
        snprintf(s,100,"K=%d,%d\n", ws, ws);
        m = write(ttyDev, s, strlen(s));
        if (freLog != NULL)
        {
          gettimeofday(&logtime, NULL);
          fprintf(freLog, "%lu.%06lu %d if1 send %d char %s",
                      logtime.tv_sec,
                      logtime.tv_usec, i, m,
                      s);
          fprintf(stdout, "%lu.%06lu %d if1 send %d char %s",
                      logtime.tv_sec,
                      logtime.tv_usec, i, m,
                      s);
        }
      }
      //printf("sending s\n");
      if (1)
      {
        write(ttyDev, "s\n", 2);
        gettimeofday(&logtime, NULL);
        if (freLog != NULL)
        {
          fprintf(freLog, "%lu.%06lu %d if1 send s\n",
                    logtime.tv_sec, logtime.tv_usec, i);
          fflush(freLog);
        }
        fprintf(stdout, "%lu.%06lu %d if1 send s\n",
                   logtime.tv_sec, logtime.tv_usec, i);
      }
      usleep(8000);
    }
    close(ttyDev);
  }
  fclose(freLog);
  return 0;
}
