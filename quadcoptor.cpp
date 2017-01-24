#include "quadcoptor.h"
#include <cmath>
#define pi 3.142
using namespace std;

#define debugMode true // preprocessor directives for Debug Mode

quadcoptor::quadcoptor(double mass, double ix, double ArmLength, state_type in_state):dMass(mass), dIx(ix), dIy(ix), dArmLength(ArmLength)
{
    x = in_state;
    inp = {{0.0,0.0,0.0,0.0}};
    dIz = dIx+dIy;
    updateSys();
}

//states:
//orientations/positions: 0 - x, 1 - y, 2 - z, 3 - phi, 4 - nu, 5 - psi
//their derivatives: 6 - vx, 7 - vy, 8 - vz, 9 - p, 10 - q, 11 - r

void quadcoptor::operator()(state_type &x, state_type &dxdt, double t)
{

    //position velocities directly updated

    dxdt[0] = x[6];

    dxdt[1] = x[7];

    dxdt[2] = x[8];

    //orientation rates

    dxdt[3] = x[9] + sin(x[3])*tan(x[4])*x[10] + cos(x[3])*tan(x[4])*x[11];

    dxdt[4] = cos(x[3])*x[10]+sin(x[3])*x[11];

    dxdt[5] = sin(x[3])*x[10]/cos(x[4]) + cos(x[3])*x[11]/cos(x[4]);

    //updation of the derivatives



    //translation accelerations

    dxdt[6] = (dDragCoeff*abs(x[6])*x[6]-(cos(x[5])*sin(x[4])*cos(x[3])+sin(x[5])*sin(x[3]))*sysActuation[0])/dMass;

    dxdt[7] = (dDragCoeff*abs(x[7])*x[7]-(sin(x[5])*sin(x[4])*cos(x[3])+sin(x[5])*cos(x[3]))*sysActuation[0])/dMass;

    dxdt[8] = -g + (dDragCoeff*abs(x[8])*x[8]+(cos(x[4])*cos(x[3])*sysActuation[0]))/dMass;

    if (debugMode == true)
    {
        cout << "Acceleration (ax,ay,az) = " << dxdt[6] << ", " << dxdt[7] << ", " << dxdt[8] << endl;
    }

    //rotational accelerations

    dxdt[9] = (dIy-dIy)*x[10]*x[11]/dIx + sysActuation[1]/dIx;

    dxdt[10] = (dIz-dIx)*x[9]*x[11]/dIy + sysActuation[2]/dIy;

    dxdt[11] = (dIx-dIy)*x[9]*x[10]/dIz + sysActuation[3]/dIz;

    if ((x[3])>pi/4 || (x[4])>pi/4 || (x[5])>pi/4){
        x[3] = min(x[3],pi/4);
        x[4] = min(x[4],pi/4);
        x[5] = min(x[5],pi/4);
    }
    if ((x[3])<-pi/4 || (x[4])<-pi/4 || (x[5])<-pi/4){
        x[3] = max(x[3],-pi/4);
        x[4] = max(x[4],-pi/4);
        x[5] = max(x[5],-pi/4);
    }

}

//update system
void quadcoptor::updateSys(){
    //update state - auto updates - due to odeint

    //update input - will depend on the final qt code - after vedant's part is finished

    //update other derived quantities
    this->updateSysActuation();
}

//update the actuation values with the new input & state
void quadcoptor::updateSysActuation(){

    //get rotorForces using the input
    rotorForces[0] = dThrustCoeff*inp[0]*inp[0];
    rotorForces[1] = dThrustCoeff*inp[1]*inp[1];
    rotorForces[2] = dThrustCoeff*inp[2]*inp[2];
    rotorForces[3] = dThrustCoeff*inp[3]*inp[3];
    rotorTorques[0] = dTorqueCoeff*inp[0]*inp[0];
    rotorTorques[1] = dTorqueCoeff*inp[1]*inp[1];
    rotorTorques[2] = dTorqueCoeff*inp[2]*inp[2];
    rotorTorques[3] = dTorqueCoeff*inp[3]*inp[3];

    //updates the net thrust
    sysActuation[0] = rotorForces[0]+rotorForces[1]+rotorForces[2]+rotorForces[3];

    //updates the rotational actuation
    sysActuation[1] = dArmLength*(rotorForces[1]-rotorForces[3]);
    sysActuation[2] = dArmLength*(rotorForces[0]-rotorForces[2]);
    sysActuation[3] = (-rotorTorques[0] + rotorTorques[1] -rotorTorques[2] + rotorTorques[3]);
    //cout << " " << sysActuation[0] << " " << sysActuation[1] << " " << sysActuation[2] << " " << sysActuation[3] <<endl;
}
