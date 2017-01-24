#include <iostream>
#include "MotorSoftware.h"
using namespace std;

#define debugMode false // preprocessor directives for Debug Mode

MotorSoftware::MotorSoftware()
{
    maxMotors = 4;
    for (int i = 0; i < maxMotors; i++)
    {
        mMotor[i] = new Motor(i + 1);
        mMotor[i]->setSpeed(initialSpeed);
        if (i)
        {
            mMotor[i - 1] ->setMotorPtr(mMotor[i]);
        }
    }
}
MotorSoftware::MotorSoftware(int n)
{
    maxMotors = n;
    for (int i = 0; i < maxMotors; i++)
    {
        mMotor[i] = new Motor(i + 1);
        mMotor[i]->setSpeed(initialSpeed);
        if (i)
        {
            mMotor[i - 1]->setMotorPtr(mMotor[i]);
        }
    }
}

MotorSoftware::~MotorSoftware()
{
    for(int i=maxMotors-1;i>0;i--)
    {
        delete mMotor[i];
        mMotor[i - 1]->setMotorPtr(0);
    }
    delete mMotor[0];
}

//gets the desired movement and set the motor current and speed as per the input
void MotorSoftware::setVoltage(dvar dThrottle,dvar dFront, dvar dBack,dvar dLeft, dvar dRight)
{
    baseVolt = 0.8*maxVolt*dThrottle;
    for (int i = 0; i < 4; ++i) {
        mMotor[i]->setSpeed(baseVolt);
    }

    if (1)//flyReady()==1)
    {
        if (dFront>0)
        {
            mMotor[0]->setSpeed(mMotor[0]->getVoltIn() - stepVolt);
            //mMotor[1]->setSpeed(mMotor[1]->getVoltIn() - stepVolt);
            mMotor[2]->setSpeed(mMotor[2]->getVoltIn() + stepVolt);
            //mMotor[3]->setSpeed(mMotor[3]->getVoltIn() + stepVolt);
        }
        if (dBack>0)
        {
            mMotor[0]->setSpeed(mMotor[0]->getVoltIn() + stepVolt);
            //mMotor[1]->setSpeed(mMotor[1]->getVoltIn() + stepVolt);
            mMotor[2]->setSpeed(mMotor[2]->getVoltIn() - stepVolt);
            //mMotor[3]->setSpeed(mMotor[3]->getVoltIn() - stepVolt);
        }
        if (dLeft>0)
        {
            //mMotor[0]->setSpeed(mMotor[0]->getVoltIn() - stepVolt);
            mMotor[1]->setSpeed(mMotor[1]->getVoltIn() - stepVolt);
            //mMotor[2]->setSpeed(mMotor[2]->getVoltIn() + stepVolt);
            mMotor[3]->setSpeed(mMotor[3]->getVoltIn() + stepVolt);
        }
        if (dRight>0)
        {
            //mMotor[0]->setSpeed(mMotor[0]->getVoltIn() + stepVolt);
            mMotor[1]->setSpeed(mMotor[1]->getVoltIn() + stepVolt);
            //mMotor[2]->setSpeed(mMotor[2]->getVoltIn() - stepVolt);
            mMotor[3]->setSpeed(mMotor[3]->getVoltIn() + stepVolt);
        }
        for (int i = 0; i < 4; ++i)
        {
            if (mMotor[i]->getSpeed() > maxSpeed)
                mMotor[i]->setSpeed(maxVolt);
            else if (mMotor[i]->getSpeed() < minSpeed)
                mMotor[i]->setSpeed(minVolt);
        }
        setCharge((mMotor[0]->getSpeed()+mMotor[1]->getSpeed()+mMotor[2]->getSpeed()+mMotor[3]->getSpeed())/500);
        if (debugMode==true)
        {
            for (int i = 0; i < 4; ++i)
            {
                cout << "Motor " << i+1 << ": " << mMotor[i]->getSpeed() << endl;
            }
            cout << "dFront = " << dFront << endl;
            cout << "dBack  = " << dBack  << endl;
            cout << "dLeft  = " << dLeft  << endl;
            cout << "dRight = " << dRight << endl;
        }
    }
    else
    {
        std::cout << "Not ready to fly";
    }
}

unsigned int MotorSoftware::flyReady() const
{
    return getCondition();

}

dvar* MotorSoftware::getAllSpeed()
{
    static dvar speeds[4];
    for(int i=0;i<4;i++)
    {
        speeds[i] = mMotor[i]->getSpeed();
    }
    return speeds;

}

