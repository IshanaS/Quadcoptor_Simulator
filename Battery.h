#ifndef BATTERY_H
#define BATTERY_H

#define dvar double
#define dDischarge 0.2
#define maxCharge 100
#define minCharge 10

class Battery
{
public:
    Battery();
    dvar getCharge() const;
    int getCondition() const;
    void setCharge(dvar);
    ~Battery();				//Destructor when fault in Motor
private:
    dvar dCharge;
    int cCondition;
};

#endif
