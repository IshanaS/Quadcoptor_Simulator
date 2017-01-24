/*
Receives voltage of motor and gives the motor speed as the output.
*/

#ifndef MotorSoftware_H
#define MotorSoftware_H

#include "Motor.h"
#include "Battery.h"

#define dvar double
#define minVolt 0                               //minimum voltage when ON
#define maxVolt 5                               //minimum voltage when ON
#define stepVolt maxVolt*0.05                    //step change in motor voltage
// initialSpeed = sqrt(0.5*9.81*0.25/0.0000000125)
#define initialSpeed 400

class MotorSoftware:public Battery
{
public:
    MotorSoftware();							//default constructor
	MotorSoftware(int);							//Constructor overloading
	~MotorSoftware();                           //de-attach all the motors
	unsigned int flyReady() const;              //Check if the rotor is ready to fly. Motor conditions
    void setVoltage(dvar,dvar,dvar,dvar,dvar);	//receive the desired movement and calculate voltage
    dvar* getAllSpeed();

private:
    Motor* mMotor[4];                            //Motor class objects
    dvar baseVolt;                              //base thrust speed
	int maxMotors;
};

#endif
