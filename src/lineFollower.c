#include <stdio.h>
#include <stdlib.h>							// For system() Function and abs() Function
#include <unistd.h>							// For usleep() Function

#include "../inc/lineFollowerMain.h"
#include "../inc/moveAlgorithm.h"
#include "../inc/adci.h"
#include "../normalizeAlgorithm.h"

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