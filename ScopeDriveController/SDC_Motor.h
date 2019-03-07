/*
 * SDC_Motor.h
 *
 * Created: 1/20/2019 6:22:28 PM
 *  Author: Alexey
 */ 


#ifndef SDC_MOTOR_H_
#define SDC_MOTOR_H_

#include "PID_v1.h"

///////////////////////////////////////////////////////////////////////////////////////
// motor class
class SDC_Motor
{
public:
    SDC_Motor(double rpm, uint8_t dirPin, uint8_t speedPin);

    // call once in setup()
    void Setup();

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run();

    bool Start (double speed,       // start speed, in encoder units/ms
                long *upos,         // starting position, in encoder units
                long *ts);          // starting timestamp, in ms (returned by millis())

    bool GetPos(long *upos, long *ts, long *setpoint);
    bool SetNextPos(long upos, long ts);
    void Stop();

private:
    double max_speed_;  // units/ms
    uint8_t dirPin_, speedPin_; // pins
    double voltage_;

    bool running_;
    long upos_;
    long ts_;
    double speed_;      // units/ms
    PID pid_;
    double setpoint_, input_, output_;

    void DoGetPos(long *upos, long *ts);
};

#endif /* SDC_MOTOR_H_ */