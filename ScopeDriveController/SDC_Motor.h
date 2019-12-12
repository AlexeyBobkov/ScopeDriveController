/*
 * SDC_Motor.h
 *
 * Created: 1/20/2019 6:22:28 PM
 *  Author: Alexey
 */ 


#ifndef SDC_MOTOR_H_
#define SDC_MOTOR_H_

#define TEST_SLOW_PWM
#define USE_SLOW_PWM

#define USE_EXPONENTIAL_APPROXIMATION   // if not defined, linear approximation is used

#include "PID_v1.h"

// motion law
class SDC_MotorItf;
class SDC_MotionType
{
public:
    virtual ~SDC_MotionType() {}
    virtual bool    CanMove(const SDC_MotorItf *m) const                = 0;    // can the motor move?
    virtual void    MotorStarted(SDC_MotorItf *m)                       = 0;    // action on motor started
    virtual void    MotorStopped(SDC_MotorItf *m, bool byStopCommand)   = 0;    // action on motor stopped
};

class SDC_MotorItf
{
public:
    struct Ref
    {
        double upos_;   // position in encoder units
        long ts_;       // timestamp in milliseconds
        Ref() {}
        Ref(double upos, long ts) : upos_(upos), ts_(ts) {}
    };

    virtual ~SDC_MotorItf() {}

    virtual bool IsRunning() const = 0;
    virtual bool GetPhysicalPos(Ref *ref, double *setpoint = NULL, double *dbgParam = NULL) const = 0;
    virtual bool GetLogicalPos(Ref *ref) const = 0;
    virtual bool GetDeviation(Ref *ref) const = 0;
    virtual double GetSpeed() const = 0;
    virtual double GetMaxSpeed() const = 0;

    virtual bool Start (double speed,                       // start speed, in encoder units/ms
                        SDC_MotionType *mt,                 // motion callback
                        Ref *ref = NULL) = 0;               // starting position and timestamp
    virtual bool SetSpeed(double speed, Ref *ref = NULL);   // speed is in encoder units/ms
    virtual bool SetNextPos(double upos, long ts, bool reset, Ref *ref = NULL) = 0;
    virtual void Stop() = 0;
};


///////////////////////////////////////////////////////////////////////////////////////
// motor class
class SDC_Motor : public SDC_MotorItf
{
public:
    // profile for low frequency PWM
    struct PWMProfile
    {
        uint8_t value_;     // required value
        uint8_t magnitude_; // PWM magnitude
        int16_t period_;    // PWM (and PID) period
        PWMProfile() {}
        PWMProfile(uint8_t v, uint8_t m, int16_t p) : value_(v), magnitude_(m), period_(p) {}
    };

    struct Options
    {
        double maxSpeed_;  // units/ms
        double Kp_, Ki_, Kd_;

        // PWM profiles
        PWMProfile loProfile_, hiProfile_;

        Options() {}
        Options(double max_speed, double Kp, double Ki, double Kd, const PWMProfile &lp, const PWMProfile &hp)
            : maxSpeed_(max_speed), Kp_(Kp), Ki_(Ki), Kd_(Kd), loProfile_(lp), hiProfile_(hp) {}
    };

    SDC_Motor(const Options &options, uint8_t dirPin, uint8_t speedPin, volatile long *encPos);

    // call once in setup()
    void Setup();

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run();

    // SDC_MotorItf
    bool IsRunning() const {return running_;}
    bool GetPhysicalPos(Ref *ref, double *setpoint, double *dbgParam) const;
    bool GetLogicalPos(Ref *ref) const;
    bool GetDeviation(Ref *ref) const;
    double GetMaxSpeed() const {return maxSpeed_;}
    double GetSpeed() const {return speed_;}
    bool Start (double speed, SDC_MotionType *mt, Ref *ref);
    bool SetSpeed(double speed, Ref *ref);
    bool SetNextPos(double upos, long ts, bool reset, Ref *ref);
    void Stop();

#ifdef TEST_SLOW_PWM
    bool StartSlowPWM(int val, long period, double dutyCycle, SDC_MotionType *mt, Ref *ref = NULL);
#endif

private:
    // encapsulated approximation model
    class PWMApproximation
    {
        PWMProfile loProfile_;
        double magnitudeCoeff_, periodCoeff_;
    public:
        PWMApproximation(const PWMProfile &lp, const PWMProfile &hp);
        void MakeApproximation(int absSp, uint8_t *magnitude, double *period);
    };

    double maxSpeed_;  // units/ms
    uint8_t dirPin_, speedPin_; // pins
    volatile long *encPos_;

    // PWM profiles
    PWMProfile loProfile_, hiProfile_;
    PWMApproximation pwmApprox_;

    SDC_MotionType *mt_;
    bool running_;
    double upos_;
    long ts_;
    double speed_;      // units/ms
    PID pid_;

    enum PWM_STATE
    {
        PWM_CONST,
        PWM_LOW,
        PWM_HIGH
    };

    long tsPWMStart_;
    long hiPeriod_;
    PWM_STATE pwmState_;
    int val_;

    void SetVal(int val);
    void SetVal(uint8_t val, bool positive);

#ifdef TEST_SLOW_PWM
    long loPeriod_;
    int testPWMVal_;
#endif
    
    double setpoint_, input_, output_;

    void DoStop();
    void DoGetPos(double *upos, long *ts);
};

#endif /* SDC_MOTOR_H_ */