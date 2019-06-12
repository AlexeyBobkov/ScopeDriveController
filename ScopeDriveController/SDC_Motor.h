/*
 * SDC_Motor.h
 *
 * Created: 1/20/2019 6:22:28 PM
 *  Author: Alexey
 */ 


#ifndef SDC_MOTOR_H_
#define SDC_MOTOR_H_

#include "PID_v1.h"

// motion law
class SDC_MotorItf;
class SDC_MotionType
{
public:
    virtual bool    CanMove(const SDC_MotorItf *m) const                = 0;    // can the motor move?
    virtual void    MotorStarted(SDC_MotorItf *m)                       = 0;    // action on motor started
    virtual void    MotorStopped(SDC_MotorItf *m, bool byStopCommand)   = 0;    // action on motor stopped
};

class SDC_MotorItf
{
public:
    struct Ref
    {
        long upos_;     // position in encoder units
        long ts_;       // timestamp in milliseconds
        Ref() {}
        Ref(long upos, long ts) : upos_(upos), ts_(ts) {}
    };

    virtual ~SDC_MotorItf() {}

    virtual bool IsRunning() const = 0;
    virtual bool GetPos(Ref *ref, long *setpoint = 0) const = 0;
    virtual double GetMaxSpeed() const = 0;

    virtual bool Start (double speed,                       // start speed, in encoder units/ms
                        SDC_MotionType *mt,                 // motion callback
                        Ref *ref = NULL) = 0;               // starting position and timestamp
    virtual bool SetSpeed(double speed, Ref *ref = NULL);   // speed is in encoder units/ms
    virtual bool SetNextPos(long upos, long ts, Ref *ref = NULL) = 0;
    virtual void Stop() = 0;
};


///////////////////////////////////////////////////////////////////////////////////////
// motor class
class SDC_Motor : public SDC_MotorItf
{
public:
    struct Options
    {
        double maxSpeed_;  // units/ms
        double Kp_, Ki_;
        Options() {}
        Options(double max_speed, double Kp, double Ki)
            : maxSpeed_(max_speed), Kp_(Kp), Ki_(Ki) {}
    };

    SDC_Motor(const Options &options, uint8_t dirPin, uint8_t speedPin, volatile long *encPos);

    // call once in setup()
    void Setup();

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run();

    // SDC_MotorItf
    bool IsRunning() const {return running_;}
    bool GetPos(Ref *ref, long *setpoint) const;
    double GetMaxSpeed() const {return maxSpeed_;}
    bool Start (double speed, SDC_MotionType *mt, Ref *ref);
    bool SetSpeed(double speed, Ref *ref);
    bool SetNextPos(long upos, long ts, Ref *ref);
    void Stop();

private:
    double maxSpeed_;  // units/ms
    uint8_t dirPin_, speedPin_; // pins
    SDC_MotionType *mt_;
    volatile long *encPos_;

    bool running_;
    long upos_;
    long ts_;
    double speed_;      // units/ms
    PID pid_;
    double setpoint_, input_, output_;

    void DoStop();
    void DoGetPos(long *upos, long *ts);
};

#endif /* SDC_MOTOR_H_ */