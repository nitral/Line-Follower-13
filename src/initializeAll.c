#include <stdio.h>
#include <avr/io.h>

#include "../inc/lineFollowerMain.h"
#include "../inc/adci.h"
#include "../inc/moveAlgorithm.h"

void initializeAll(int sensorArray[TOTALSENSORS])
{
	int i;

	// Initialize Sensor Array. Make Bot Think it doesn't detect any line.
	for(i = 0 ; i < TOTALSENSORS ; i++)
		sensorArray[i] = HIGH;
	
	initializeADC();
	initializePWM();

	// Assigns PORTD Pins as Output Pins
	
	DDRD = 0xFF;
	
	// Assigns PORTA Pins as Input Pins
	
	DDRA = 0x00;

	// Initializes PORTD
	PORTD = 0x00;
	PORTD |= 0x08;
	PORTD |= 0x02;
}
