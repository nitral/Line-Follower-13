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