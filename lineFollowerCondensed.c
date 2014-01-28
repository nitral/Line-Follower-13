/* ********************************************
Line Follower 2013 | Condensed Source Code File
******************************************** */

// Library Header Files

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>

/* ****************************
Header Files for Line Follower.
**************************** */

// Main Header File for Line Follower

#define TRUE 1
#define FALSE 0

#define HIGH 1
#define LOW 0

#define TOTALSENSORS 7

void initializeAll(int sensorArray[TOTALSENSORS]);

void inputSensorArray(int sensorArray[TOTALSENSORS]);

int isRunningFeasible(void);

void normalizeSensorArray(int sensorArray[TOTALSENSORS]);

void moveLineFollower(int sensorArray[TOTALSENSORS]);

void initializeADC(void);

void initializePWM(void);

// Header File for ADC Conversion of Microcontroller Input

#define THRESH 512

void initializeADC(void);

unsigned int readADC(unsigned char channel);

// Header File for Normalization of Sensor Array Algorithm

int isArrayEmpty(int sensorArray[TOTALSENSORS]);

int isArrayFull(int sensorArray[TOTALSENSORS]);

// Header For Line Follower Motion Algorithm

#define Kp 1.5
#define Ki 0.75
#define Kd 0.6

#define TOP 0x00FF
#define MAX 255
#define BOTTOM 0

#define VOLTAGESOURCE 11
#define MOTORSCALEDOWN 1.4

#define IDEALERROR 0

void getMotorVoltage(int sensorArray[TOTALSENSORS], double motorVoltage[2]);

void getOCRValue(int OCRValue[2], double motorVoltage[2]);

double getPIDControllerValue(int sensorArray[TOTALSENSORS]);

int getError(int sensorArray[TOTALSENSORS]);

double getProportional(int error);

double getIntegral(int error);

double getDerivate(int error);

double getControlValue(double P, double I, double D);

double normalizePIDControllerValue(double PIDControllerValue);

void driveMotors(double motorVoltage[2]);

void startPWM(int OCRValue[2]);

/* ***********************************
Source Code For Functions Begins Here.
*********************************** */


int main()
{
	int sensorArray[TOTALSENSORS];

	initializeAll(sensorArray);
	
	while(TRUE)
	{
		// For Clean Simulator
		//system("clear");					// Clears Screen | For Unix
		
		inputSensorArray(sensorArray);
		
		if(isRunningFeasible())
		{
			normalizeSensorArray(sensorArray);
			moveLineFollower(sensorArray);
		}
		else
			break;
		
		_delay_ms(100);						// Gives a Delay of 100 milliseconds between 2 successive Iterations to reduce stress on Microcontroller and let Bot run PWM sucessfully.
		// For Clean Simulator
		//usleep(500000);					// Adds a Delay | Argument in Microseconds
	}

	return 0;
}

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

// Check if Running Bot is Feasible. If Yes, Return TRUE. Else, return FALSE
int isRunningFeasible(void)
{
	return TRUE;
}


void normalizeSensorArray(int sensorArray[TOTALSENSORS])
{

	// Stores Previous Line Signal. If New Line Signal is Empty, then restore previous Line Signal as current one.
	static int previousSensorArray[TOTALSENSORS] = {0, 0, 0, 1, 0, 0, 0};
	int i;
	
	if(isArrayEmpty(sensorArray))										// Check if Robot DOES NOT detect Line. If TRUE, restore previous known Line Position.
	{
		for(i = 0 ; i < TOTALSENSORS ; i++)
		{
			sensorArray[i] = previousSensorArray[i];
		}
	}
	else if(isArrayFull(sensorArray))									// Check if Robot detects a line throughout. If TRUE, restore previous known Line Position.
	{
		
		for(i = 0 ; i < TOTALSENSORS ; i++)
		{
			sensorArray[i] = previousSensorArray[i];
		}
	}
	else																// If Line is distinguished properly, Remember it.
	{
		for(i = 0 ; i < TOTALSENSORS ; i++)
		{
			previousSensorArray[i] = sensorArray[i];
		}
	}
	
}

int isArrayEmpty(int sensorArray[TOTALSENSORS])
{
	int i;
	
	for(i = 0 ; i < TOTALSENSORS ; i++)
	{
		if(sensorArray[i] == LOW)
				return FALSE;
	}
	
	return TRUE;
}

int isArrayFull(int sensorArray[TOTALSENSORS])
{
	int i;
	
	for(i = 0 ; i < TOTALSENSORS ; i++)
	{
		if(sensorArray[i] == HIGH)
			return FALSE;
	}
	
	return TRUE;
}

void moveLineFollower(int sensorArray[TOTALSENSORS])
{
	int OCRValue[2];
	double motorVoltage[2];

	getMotorVoltage(sensorArray, motorVoltage);					// Get Voltages to be Fed to Motors
	getOCRValue(OCRValue, motorVoltage);						// Get OCR Values for PWM of Motors

	// To Test OCR Value Output
	//printf("OCR Value Left = %3d | OCR Value Right = %3d | ", OCRValue[0], OCRValue[1]);
	//printf("Motor Voltage Left = %3lf | Motor Voltage Right = %3lf\n", motorVoltage[0], motorVoltage[1]);

	driveMotors(motorVoltage);									// Get Motors to Run in Correct Direction as Calculated in motorVoltage.
	startPWM(OCRValue);											// Get Motors to Run with Calculated Motor Voltages.
}

// Gets PID Controller Value. PID Controller Value has 3 Parts (Proportional, Integral and Derivate). Returns The Controller Value.
double getPIDControllerValue(int sensorArray[TOTALSENSORS])
{
	int error;
	double P, I, D, controlValue;

	error = getError(sensorArray);

	P = getProportional(error);
	I = getIntegral(error);
	D = getDerivate(error);

	controlValue = getControlValue(P, I, D);

	return controlValue;
}

// Gets Error/Deviation of Bot from Ideal Position. Uses Weigths on different sensors to calculate Error/Deviation.
// Works only for ODD Number of Sensors eg: 7.
int getError(int sensorArray[TOTALSENSORS])
{
	int i, error;

	for(i = 0, error = 0 ; i < TOTALSENSORS ; i++)
	{
			if(sensorArray[i] == LOW)
				error += i - ((int)(TOTALSENSORS / 2)) - IDEALERROR;
			else
				continue;
	}

	return error;
}

double getProportional(int error)
{
	return (Kp * error);
}

double getIntegral(int error)
{
	static double integral = 0;
	static int previousError = 0;
	
	// Nullifies the Integral Part of PID when the accumulated Integral is no longer needed. Else, calculates Integral.
	if (error == 0 || (integral / abs(integral)) != (previousError / abs(previousError)))
	{
		previousError = error;
		integral = 0;
		return integral;
	}
	else
	{
		integral = Ki * (integral + error);
		previousError = error;
	}
	
	return integral;
}

double getDerivate(int error)
{
	static int previousError = 0;
	double derivate;

	derivate = (error - previousError);
	derivate *= Kd;

	previousError = error;

	return derivate;
}

double getControlValue(double P, double I, double D)
{
	double controlValue;

	controlValue = P + I + D;

	return controlValue;
}

// Normalizes PID Controller Value to lie within a Specified Range.
// Removes Error due to Abnormally Large PID Controller Values.
double normalizePIDControllerValue(double PIDControllerValue)
{
	// Formula to Normalize PID Controller Value to approximately lie between -(2 x VOLTAGESOURCE) and (2 x VOLTAGESOURCE)
	PIDControllerValue = (PIDControllerValue / ((3 * Kp) + Kd)) * (2 * VOLTAGESOURCE);
	return PIDControllerValue;
}

void getMotorVoltage(int sensorArray[TOTALSENSORS], double motorVoltage[2])
{

	double PIDControllerValue;
	
	PIDControllerValue = getPIDControllerValue(sensorArray);
	PIDControllerValue = normalizePIDControllerValue(PIDControllerValue);

	// Assigns Motor Voltages based on PID Controller Value.
	if(PIDControllerValue > 0)										// If PIDControllerValue is +ve, Line should be LEFT of Bot.
	{
		motorVoltage[0] = VOLTAGESOURCE;
		motorVoltage[1] = VOLTAGESOURCE - PIDControllerValue;
	}
	else if(PIDControllerValue < 0)									// If PIDControllerValue is -ve, Line should be RIGHT of Bot.
	{
		motorVoltage[0] = VOLTAGESOURCE + PIDControllerValue;
		motorVoltage[1] = VOLTAGESOURCE;
	}
	else															// If PIDControllerValue is 0, LINE is at IDEALPOSITION.
	{
		motorVoltage[0] = VOLTAGESOURCE;
		motorVoltage[1] = VOLTAGESOURCE;
	}

	// If No Error. Run Bot Straight Ahead at FULL SPEED.
	/*if(!getError(sensorArray))
	{
		motorVoltage[0] = VOLTAGESOURCE;
		motorVoltage[1] = VOLTAGESOURCE;
	}*/
	
	// Special Case for 90 Degrees Turn.
	if(sensorArray[0] == LOW && sensorArray[1] == LOW)				// If 90 Degree Turn is to LEFT of Bot.
	{
		motorVoltage[0] = -VOLTAGESOURCE;
		motorVoltage[1] = VOLTAGESOURCE;
	}
	else if(sensorArray[5] == LOW && sensorArray[6] == LOW)			// If 90 Degree Turn is to RIGHT of Bot.
	{
		motorVoltage[0] = VOLTAGESOURCE;
		motorVoltage[1] = -VOLTAGESOURCE;
	}
	
	// To Test ADC. Motor Stops on Corresponding Side on Sensor Array Finding Line.
	/*	
	if(sensorArray[0] == LOW || sensorArray[1] == LOW || sensorArray[2] == LOW)
	motorVoltage[0] = 0;
	
	if(sensorArray[3] == LOW)
	{
		motorVoltage[0] = 0;
		motorVoltage[1] = 0;
	}
	
	if(sensorArray[4] == LOW || sensorArray[5] == LOW || sensorArray[6] == LOW)
		motorVoltage[1] = 0;
	*/	
	
	// If Calculated Motor Voltage is Out of Range, Make it Maximum Possbile while retaining Direction of Motor Motion.
	if(motorVoltage[0] > VOLTAGESOURCE || motorVoltage[0] < -VOLTAGESOURCE)
		motorVoltage[0] = (motorVoltage[0] / fabs(motorVoltage[0])) * VOLTAGESOURCE;

	if(motorVoltage[1] > VOLTAGESOURCE || motorVoltage[1] < -VOLTAGESOURCE)
		motorVoltage[1] = (motorVoltage[1] / fabs(motorVoltage[1])) * VOLTAGESOURCE;

	// Scale Down Motor Voltages by dividing with a factor of MOTORSCALEDOWN
	motorVoltage[0] /= MOTORSCALEDOWN;
	motorVoltage[1] /= MOTORSCALEDOWN;
}

void getOCRValue(int OCRValue[2], double motorVoltage[2])
{
	double dutyCycle;
	int i;

	// Calculate OCRValues for PWM for both Motors based on their calculated Motor Voltages
	for(i = 0 ; i < 2 ; i++)
	{
		dutyCycle = motorVoltage[i] / VOLTAGESOURCE;						 // dutyCycle is ON_TIME by (ON_TIME + OFF_TIME) of Motor.
		OCRValue[i] = dutyCycle * TOP;
		OCRValue[i] = abs(OCRValue[i]);
	}

	// If Calculated OCR Value is Out of Range, Make it Maximum Possbile.
	if(OCRValue[0] > TOP)
		OCRValue[0] = TOP;
	
	if(OCRValue[1] > TOP)
		OCRValue[1] = TOP;
}

// Drives Motors by Assigning Bits to PORTD
void driveMotors(double motorVoltage[2])
{

	if(motorVoltage[0] > 0)								// PORTD Configuration for Left Motor FORWARD
	{
		PORTD &= 0xFB;
		PORTD |= 0x08;
	}
	else if(motorVoltage[0] < 0)						// PORTD Configuration for Left Motor BACKWARD
	{
		PORTD &= 0xF7;
		PORTD |= 0x04;
	}
	else												// PORTD Configuration for Left Motor STOP
		PORTD &= 0xF3;

	if(motorVoltage[1] > 0)								// PORTD Configuration for Right Motor FORWARD
	{
		PORTD &= 0xFE;
		PORTD |= 0x02;
	}
	else if(motorVoltage[1] < 0)						// PORTD Configuration for Right Motor BACKWAWRD
	{
		PORTD &= 0xFD;
		PORTD |= 0x01;
	}
	else												// PORTD Configuration for Right Motor STOP
		PORTD &= 0xFC;

}

// Initializes PWM for use
void initializePWM(void)
{

	ICR1 = TOP;											// TOP Number to which Counter Counts.
	DDRD |= 1<<PD5;										// PD5 Pin of PORTD is now Enabler Pin. Does PWM.
	DDRD |= 1<<PD4;										// PD4 Pin of PORTD is now Enabler Pin. Does PWM.
	TCCR1A |= 1<<COM1A1;								// Sets Compare Output Mode for Phase Correct PWM.
	TCCR1A |= 1<<WGM11;									// Assigning Bits 1 to WGM11 and WGM13 makes PWM Mode as PWM Phase Correct.
	TCCR1B |= 1<<WGM13;
	TCCR1A |= 1<<COM1B1;								// Sets Compare Output Mode for Phase Correct PWM.
	TCCR1B |= 1<<CS10;									// We use Internal Clock. Assigning 1 to CS10 Bit does not Prescale this Clock.

}

// Assigns OCR Values to PWM Output Pins to start PWM Generation
void startPWM(int OCRValue[2])
{

	OCR1B = OCRValue[0];								// Assigns OCR Value to Output Compare Register of Left Motor.
	OCR1A = OCRValue[1];								// Assigns OCR Value to Output Compare Register of Right Motor.

}