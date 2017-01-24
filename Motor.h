/*
Receives voltage of motor and gives the motor speed as the output.
*/

#ifndef MOTOR_H
#define MOTOR_H

#define NODE_TYPE double
#define SpeedManipulateFunction(volt) (20*volt)	//volt-speed curve of the motor
#define maxSpeed 100
#define minSpeed 0		//minimum speed when motor is ON

class Motor
{
public:
	Motor();                        //default constructor
	Motor(NODE_TYPE);               //constructor overload
    ~Motor();                       //Destructor when fault in Motor
    NODE_TYPE getVoltIn() const;    //gives the current input voltage on demand
    void setSpeed(NODE_TYPE);       //sets the speed of the motor based on voltage
    NODE_TYPE getSpeed() const;     //gives the rotation velocity of the motor
	Motor* getMotorPtr();			//returns address of next element
	void setMotorPtr(Motor *n);     //sets address of next element
private:
    NODE_TYPE ivoltIn;              //voltage input into the motor
    NODE_TYPE iMotorSpeed;          //motor output speed
    const NODE_TYPE iMotorLocation;	//Motor location wrt quadcopter
    Motor* nextPtr;
};

#endif
