// MD25 example c code for the Raspberry pi.
//
// Drives both motors untill an encoder count of over 0x2000
// is reached and stops the motors. Displays the current decoder value.
//
// By James Henderson, 2012

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

long readEncoderValues(void);								// Reads encoder data for both motors and displays to the screen
void resetEncoders(void);									// Resets the encoders to 0
void driveMotors(void);										// Drive the motors forwards
void stopMotors(void);										// Stop the motors

int fd;														// File descrition
char *fileName = "/dev/i2c-1";								// Name of the port we will be using
int  address = 0xb0 >> 1;										// Address of MD25 shifted one bit
unsigned char buf[10];										// Buffer for data being read/ written on the i2c bus

int main(int argc, char **argv)
{
	printf("**** MD25 test program ****\n");
		
	if ((fd = open(fileName, O_RDWR)) < 0) {					// Open port for reading and writing
		printf("Failed to open i2c port\n");
		exit(1);
	}

	if (ioctl(fd, I2C_SLAVE, address) < 0) {					// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}
	
	buf[0] = 13;												// This is the register we wish to read software version from
	
	if ((write(fd, buf, 1)) != 1) {								// Send regiter to read software from from
		printf("Error writing to i2c slave (1)\n");
		exit(1);
	}
	
	if (read(fd, buf, 1) != 1) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else {
		printf("Software version: %u\n", buf[0]);
	}
	
	resetEncoders();											// Reset the encoder values to 0
	
	while(readEncoderValues() < 0x2000) {						// Check the value of encoder 1 and stop after it has traveled a set distance
		 driveMotors();
		 usleep(200000);										// This sleep just gives us a bit of time to read what was printed to the screen in driveMortors()
	}
	
	stopMotors();
	return 0;
}

void resetEncoders(void) {
	buf[0] = 16;												// Command register
	buf[1] = 32;												// command to set decoders back to zero
	
	if ((write(fd, buf, 2)) != 2) {
		printf("Error writing to i2c slave\n");
		exit(1);
	}
}

long readEncoderValues (void) {
	
	long encoder1, encoder2;
	
	buf[0] = 2;													// register for start of encoder values
	
	if ((write(fd, buf, 1)) != 1) {
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buf, 8) != 8) {								// Read back 8 bytes for the encoder values into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else {
		encoder1 = (buf[0] <<24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];	// Put encoder values together
		encoder2 = (buf[4] <<24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
		printf("Encoder 1: %08lX   Encoder 2: %08lX\n",encoder1, encoder2);
	}
	return encoder1;	
}

void driveMotors(void){
	buf[0] = 0;													// Register to set speed of motor 1
	buf[1] = 200;												// speed to be set
	
	if ((write(fd, buf, 2)) != 2) {
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	buf[0] = 1;													// motor 2 speed
	buf[1] = 200;												
	
	if ((write(fd, buf, 2)) != 2) {
		printf("Error writing to i2c slave\n");
		exit(1);
	}
}

void stopMotors(void){
	buf[0] = 0;													
	buf[1] = 128;												// A speed of 128 stops the motor
	
	if ((write(fd, buf, 2)) != 2) {
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	buf[0] = 1;													
	buf[1] = 128;												
	
	if ((write(fd, buf, 2)) != 2) {
		printf("Error writing to i2c slave\n");
		exit(1);
	}
}
