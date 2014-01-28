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