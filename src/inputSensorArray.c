#include <stdio.h>

#include "../inc/lineFollowerMain.h"
#include "../inc/adci.h"

void inputSensorArray(int sensorArray[TOTALSENSORS])
{

// LINE is to the LEFT of Bot
/*
	sensorArray[0] = LOW;
	sensorArray[1] = LOW;
	sensorArray[2] = HIGH;
	sensorArray[3] = HIGH;
	sensorArray[4] = HIGH;
	sensorArray[5] = HIGH;
	sensorArray[6] = HIGH;
	sensorArray[7] = HIGH;
*/

// LINE is to the RIGHT of Bot
/*
 	sensorArray[0] = HIGH;
	sensorArray[1] = HIGH;
	sensorArray[2] = HIGH;
	sensorArray[3] = HIGH;
	sensorArray[4] = HIGH;
	sensorArray[5] = HIGH;
	sensorArray[6] = LOW;
	sensorArray[7] = LOW;
*/

// Only LEFT for some time. Then, only RIGHT for some time and back
/*
	static int phase = 0;
	static int counter = 0;
	
	if(phase == 0)
	{
		sensorArray[0] = LOW;
		sensorArray[1] = LOW;
		sensorArray[2] = HIGH;
		sensorArray[3] = HIGH;
		sensorArray[4] = HIGH;
		sensorArray[5] = HIGH;
		sensorArray[6] = HIGH;
		sensorArray[7] = HIGH;
		counter++;
	}
	else if(phase == 1)
	{
		sensorArray[0] = HIGH;
		sensorArray[1] = HIGH;
		sensorArray[2] = HIGH;
		sensorArray[3] = HIGH;
		sensorArray[4] = HIGH;
		sensorArray[5] = HIGH;
		sensorArray[6] = LOW;
		sensorArray[7] = LOW;
		counter++;
	}
	
	if(counter >= 1000)
	{
		phase = (phase + 1)  % 2;
		counter = 0;
	}
*/
		
// Sinusoidal Track
/*
	int i;
	static int current = 0, phase = 1, counter = 0;
	
	for(i = 0 ; i < TOTALSENSORS ; i++)
		sensorArray[i] = HIGH;
		
	if(phase == 1)
	{
		if(current < TOTALSENSORS)
		{
			sensorArray[current] = LOW;
			if(counter >= 63)
			{
				current++;
				counter = 0;
			}
			counter++;
		}
		else
		{
			phase = 2;
			current--;
			counter = 63;
		}
	}
	
	if(phase == 2)
	{
	
		if(current == 0)
		{
			phase = 1;
		}
		
		if(current > 0)
		{
			
			if(counter >= 63)
			{
				current--;
				counter = 0;
			}
			
			counter++;
			
			sensorArray[current] = LOW;
		}
	}
*/

// To Test Reaction when No Line is Found
/*
	int i;
	for(i = 0 ; i < TOTALSENSORS ; i++)
		sensorArray[i] = LOW;
*/
		
// Reading Input From ADC
/*
	int i;
	
	for(i = 0 ; i < TOTALSENSORS ; i++)
	{
		if(readADC(i) >= THRESH)
			sensorArray[i] = HIGH;
		else if(readADC(i) < THRESH)
			sensorArray[i] = LOW;
	
	}
*/

// Directly Get Input from Array. Use ONLY if Comparator is used.

	int pinAStatus = PINA;
	int i;
	for(i = 0 ; i < TOTALSENSORS ; i++)
		sensorArray[i] = (pinAStatus >> (i)) & ~(~0 << 1);


// Print Sensor Array
/*
 	printf("SENSOR ARRAY: ");
	for(i = 0 ; i < TOTALSENSORS ; i++)
		printf("%d ", sensorArray[i]);
	printf("| ");
*/

}

// Function to Initialize ADC
void initializeADC(void)
{
/*

// AVcc with external capacitor at AREF	
	ADMUX = (1<<REFS0);	
	
// Enable ADC and set Prescaler division factor as 16
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);	

*/
}

// Function to Read ADC Value for a Channel
unsigned int readADC(unsigned char channel)
{
/*

// channel must be b/w 0 to (TOTALSENSORS - 1)
	channel = channel & 0b00000110;	
			
// selecting channel	
	ADMUX |= channel;		

// start conversion	
	ADCSRA |= (1<<ADSC);	
			
// waiting for ADIF, conversion complete	
	while(!(ADCSRA & (1<<ADIF)));	
			
// clearing of ADIF, it is done by writing 1 to it	
	ADCSRA |= (1<<ADIF);
	
	return (ADC);

*/
}