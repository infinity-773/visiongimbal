#ifndef SUPER_STRUCTURE_H_
#define SUPER_STRUCTURE_H_

#include "Serial.h"
#include "TargetContainer.h"

struct axisData
{
    double yaw;
    double pitch;

    axisData(double yawValue = 0.0, double pitchValue = 0.0)
        : yaw(yawValue), pitch(pitchValue)
    {}
};

struct OutputData
{
    axisData axis;
    bool fire = false;
    bool useABSposition = true;
};



#endif // SUPER_STRUCTURE_H_