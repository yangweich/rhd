
//Port D Pin Assignments
#define RX		0
#define TX		1
#define	M1_P	2
#define M1_N	3
#define M2_P	4
#define M2_N	7

//Port B Pin Assignments
#define MOSI	3
#define MISO	4
#define SCK		5

//*******************************************************
//						Macros
//*******************************************************
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))


//*******************************************************
//					General Definitions
//*******************************************************
#define MYUBRR 16	//Used to set the AVR Baud Rate TO 115200 (External 16MHz Oscillator)
#define MAX_COMMAND_LENGTH	10

#define SPEED_UNIT	114
#define AVERAGE	16

#define CURRENT_THRESHOLD	150 //This is compared against the 10 bit ADC value and corresponds to roughly 1.5A on the Current Sense pin
		//from the Motor Controller

#include <stdint.h>
//void set_direction(const uint8_t motor, const char direcion);
void sendString(const char * str);
//uint16_t read_sense(char motor);
