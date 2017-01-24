#include <iostream>
#include "Motor.h"
using namespace std;
#define debugMode false         // preprocessor directives for Debug Mode

Motor::Motor():iMotorLocation(0)
{//default constructor
	nextPtr = 0;
}

Motor::Motor(NODE_TYPE pos):iMotorLocation(pos)
{
    nextPtr=0;
    iMotorSpeed = 0;
}

Motor::~Motor()
{
    if (debugMode==true)
        cout << "Motor " << iMotorLocation << " is detached";
}


NODE_TYPE Motor::getVoltIn() const
{
    return ivoltIn;
}

void Motor::setSpeed(NODE_TYPE volt)
{
    ivoltIn=volt;
    NODE_TYPE s = SpeedManipulateFunction(volt);
    if (s > maxSpeed)
        iMotorSpeed = maxSpeed;
    else if (s < minSpeed)
        iMotorSpeed = minSpeed;
    else
        iMotorSpeed = s;
}

NODE_TYPE Motor::getSpeed() const
{
    return iMotorSpeed;
}

Motor* Motor::getMotorPtr()
{
    return nextPtr;
}

void Motor::setMotorPtr(Motor *n)
{
    nextPtr=n;
}
