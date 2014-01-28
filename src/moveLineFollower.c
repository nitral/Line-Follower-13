#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "../inc/lineFollowerMain.h"
#include "../inc/moveAlgorithm.h"

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