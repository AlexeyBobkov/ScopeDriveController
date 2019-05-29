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
    // motion law
    class MotionType
    {
    public:
        virtual bool    CanMove(const SDC_Motor *m) const   = 0;    // can the motor move?
        virtual void    MotorStarted(SDC_Motor *m)          = 0;    // action on motor started
        virtual void    MotorStopped(SDC_Motor *m)          = 0;    // action on motor stopped
    };

    SDC_Motor(double rpm, uint8_t dirPin, uint8_t speedPin, MotionType *mt, volatile long *encPos);

    // call once in setup()
    void Setup();

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run();

    bool Start (double speed,       // start speed, in encoder units/ms
                long *upos,         // starting position, in encoder units
                long *ts);          // starting timestamp, in ms (returned by millis())

    bool IsRunning() const {return running_;}
    bool GetPos(long *upos, long *ts, long *setpoint);
    bool SetNextPos(long upos, long ts);
    void Stop();

private:
    double max_speed_;  // units/ms
    uint8_t dirPin_, speedPin_; // pins
    MotionType *mt_;
    volatile long *encPos_;

    bool running_;
    long upos_;
    long ts_;
    double speed_;      // units/ms
    PID pid_;
    double setpoint_, input_, output_;

    void DoGetPos(long *upos, long *ts);
};

#endif /* SDC_MOTOR_H_ */