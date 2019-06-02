/*
 * SDC_EncPositionAdapter.h
 *
 * Created: 6/1/2019 3:35:46 PM
 *  Author: Alexey
 */


#ifndef SDC_ENCPOSITIONADAPTER_H_
#define SDC_ENCPOSITIONADAPTER_H_

#include "PID_v1.h"
#include "SDC_Motor.h"

class SDC_EncPosAdapter
{
public:
    SDC_EncPosAdapter(double Kp, double Ki, volatile long *scopeEncPos, volatile long *motorEncPos, double motorToScope, SDC_Motor *motor);

    // call once in setup()
    void Setup();

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run();

private:
    volatile long *scopeEncPos_, *motorEncPos_;
    double motorToScope_;
    SDC_Motor *motor_;

    PID pid_;
    double setpoint_, input_, output_;
};



#endif /* SDC_ENCPOSITIONADAPTER_H_ */