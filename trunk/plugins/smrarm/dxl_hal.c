//##########################################################
//##                      dxl_hal.h		                  ##
//##     low level Servo control utility functions        ##
//##                                   Version 2012.06.01 ##
//##########################################################

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <inttypes.h>
#include <poll.h>

#include "dxl_hal.h"

//intitialize values.
int	gSocket_fd	= -1;
uint64_t	glStartTime	= 0;
uint64_t	gfRcvWaitTime	= 0;
uint64_t	gfByteTransTime	= 0;

char	gDeviceName[20];

int dxl_hal_open(int deviceIndex, float baudrate)
{
	struct termios newtio;
	struct serial_struct serinfo;
	char dev_name[100] = {0, };

	sprintf(dev_name, "/dev/ttyUSB%d", deviceIndex);

	strcpy(gDeviceName, dev_name);
	memset(&newtio, 0, sizeof(newtio));
	dxl_hal_close();
	
	if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY)) < 0) {     // | O_NONBLOCK                        *******************
		fprintf(stderr, "device open error: %s\n", dev_name);
		goto DXL_HAL_OPEN_ERROR;
	}

	newtio.c_cflag		= B38400|CS8|CLOCAL|CREAD;
	newtio.c_iflag		= IGNPAR;
	newtio.c_oflag		= 0;
	newtio.c_lflag		= 0;
	newtio.c_cc[VTIME]	= 0;	// time-out value (TIME * 0.1) 0 : disable
	newtio.c_cc[VMIN]	= 0;	// MIN is the minimum number of characters read will return

	tcflush(gSocket_fd, TCIFLUSH);
	tcsetattr(gSocket_fd, TCSANOW, &newtio);
	
	if(gSocket_fd == -1)
		return 0;
	
	if(ioctl(gSocket_fd, TIOCGSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot get serial info\n");
		return 0;
	}
	
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
	if(ioctl(gSocket_fd, TIOCSSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot set serial info\n");
		return 0;
	}
	
	dxl_hal_close();
	
	gfByteTransTime = ((uint64_t)1000 * (uint64_t)1000000 * (uint64_t)12)/ baudrate;//1000000 = microsec
	
	strcpy(gDeviceName, dev_name);
	memset(&newtio, 0, sizeof(newtio));
	dxl_hal_close();
	
	if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY)) < 0) {     // | O_NONBLOCK                        *******************
		fprintf(stderr, "device open error: %s\n", dev_name);
		goto DXL_HAL_OPEN_ERROR;
	}

	newtio.c_cflag		= B38400|CS8|CLOCAL|CREAD;
	newtio.c_iflag		= IGNPAR;
	newtio.c_oflag		= 0;
	newtio.c_lflag		= 0;
	newtio.c_cc[VTIME]	= 0;	//time-out value(TIME * 0.1) 0 : disable
	newtio.c_cc[VMIN]	= 0;	// MIN is the minimum number of characters read will return

	tcflush(gSocket_fd, TCIFLUSH);
	tcsetattr(gSocket_fd, TCSANOW, &newtio);
	
	return 1;

DXL_HAL_OPEN_ERROR:
	dxl_hal_close();
	return 0;
}

void dxl_hal_close()
{
	if(gSocket_fd != -1)
		close(gSocket_fd);
	gSocket_fd = -1;
}

int dxl_hal_set_baud( float baudrate )
{
	struct serial_struct serinfo;
	
	if(gSocket_fd == -1)
		return 0;
	
	if(ioctl(gSocket_fd, TIOCGSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot get serial info\n");
		return 0;
	}
	
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
	if(ioctl(gSocket_fd, TIOCSSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot set serial info\n");
		return 0;
	}
	
	//dxl_hal_close();
	//dxl_hal_open(gDeviceName, baudrate);
	
	gfByteTransTime = ((uint64_t)1000 * (uint64_t)1000000 * (uint64_t)12)/ baudrate;
	return 1;
}

void dxl_hal_clear(void)
{
	tcflush(gSocket_fd, TCIFLUSH);
}

int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
	return write(gSocket_fd, pPacket, numPacket);
}

int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
	memset(pPacket, 0, numPacket);
	return read(gSocket_fd, pPacket, numPacket);
}

inline uint64_t CurrentMicrosec()
{
    struct timeval t;
    gettimeofday(&t, 0);
    uint64_t d = t.tv_sec*1000000 + t.tv_usec;
    return d;
}

void dxl_hal_set_timeout( int NumRcvByte )
{
	glStartTime = CurrentMicrosec();
	//printf("time start %llu",glStartTime);
	gfRcvWaitTime = (uint64_t)(gfByteTransTime*NumRcvByte);//+ 150.0f);
	//printf("gf = %llu        num = %d",gfByteTransTime,NumRcvByte);
	//printf("time wait %llu",gfRcvWaitTime);
}
extern uint64_t gfEndTime;

int dxl_hal_timeout(void)
{
	uint64_t time;

	time = CurrentMicrosec() - glStartTime;
	gfEndTime=time;
	if(time > gfRcvWaitTime){
		return 1;
	}
	else if(time < 0)
		glStartTime = CurrentMicrosec();
		
	return 0;
}
//This is to ease the cpu usag e
int dxl_hal_poll()
{
    struct pollfd s;
    s.fd = gSocket_fd;
    s.events = POLLIN | POLLPRI;

    const int timeout = 10;
    return poll(&s, 1, timeout);
}
