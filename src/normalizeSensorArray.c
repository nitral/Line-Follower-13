#include <stdio.h>

#include "../inc/lineFollowerMain.h"
#include "../inc/normalizeAlgorithm.h"

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