#ifndef quadcoptor_hpp
#define quadcoptor_hpp

#include <stdio.h>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <algorithm>
namespace odeint = boost::numeric::odeint;
using namespace std;
using namespace boost::numeric::odeint;

typedef vector<double> state_type;
typedef std::array<double,4> inp_type;

class quadcoptor{
public:
    vector<double> x;
    inp_type u;
    quadcoptor(double dMass, double dIx, double dArmLength, state_type in_state);
    void operator()(state_type &x, state_type &dxdt, double t);
    void updateSys();
    void setInp(inp_type f){inp = f;}
    //void RkfODE((state_type &x, state_type &dxdt, double t);

private:
    //parameters assigned upon initialization
    double dMass, dIx, dIy, dIz, dArmLength;

    //const parameters
    const double dThrustCoeff = 0.0000000125;
    const double dTorqueCoeff = 0.25*dThrustCoeff;
    const double dDragCoeff = -0.05;
    const double g = 9.81;

    //force & torque of the rotors
    std::array<double,4> rotorForces;
    std::array<double,4> rotorTorques;

    //actuation components
    std::array<double,4> sysActuation;

    //inputs
    inp_type inp;

    //private functions
    void updateSysActuation();

};

#endif /* quadcoptor_hpp */

