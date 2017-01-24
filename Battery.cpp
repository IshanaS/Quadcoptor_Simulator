#include "Battery.h"

Battery::Battery()
{
    cCondition=1;
    dCharge=maxCharge;
}

Battery::~Battery() {}

dvar Battery::getCharge() const
{
    return dCharge;
}

int Battery::getCondition() const
{
    if(dCharge<=minCharge)
        return 0;
    else
        return 1;
}

void Battery::setCharge(dvar dDrainStep)
{
    dCharge-=(dDrainStep*dDischarge);
}