/*
 * SDC_EncPositionAdapter.cpp
 *
 * Created: 6/1/2019 3:36:09 PM
 *  Author: Alexey
 */

#include <Arduino.h>

#include "SDC_EncPositionAdapter.h"

SDC_EncPosAdapter::SDC_EncPosAdapter(double Kp, double Ki, volatile long *scopeEncPos, volatile long *motorEncPos, double motorToScope, SDC_Motor *motor)
    :   scopeEncPos_(scopeEncPos), motorEncPos_(motorEncPos), motorToScope_(motorToScope), motor_(motor),
        pid_(&input_, &output_, &setpoint_, Kp, Ki, 0, DIRECT)
{
}

// call once in setup()
void SDC_EncPosAdapter::Setup()
{
}

// call periodically in loop()
// returns true if safe to do some long job
bool SDC_EncPosAdapter::Run()
{
    return true;
}
