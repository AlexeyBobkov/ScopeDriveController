/*
 * SDC_Motor.h
 *
 * Created: 1/20/2019 6:22:28 PM
 *  Author: Alexey
 */ 


#ifndef SDC_MOTOR_H_
#define SDC_MOTOR_H_

//#define TEST_SLOW_PWM
#define USE_SLOW_PWM

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

// flags for SetSpeed and SetNextPos
#define FLG_BOOST_SPEED     1

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
    virtual bool SetNextPos(double upos, long ts, int flags, Ref *ref = NULL) = 0;
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

    enum ApproximationType
    {
        LINEAR,
        EXPONENTIAL
    };

    struct Options
    {
        long encRes_;
        double maxSpeedRPM_;  // RPM
        double Kp_, KiF_, Kd_;

        // PWM profiles
        ApproximationType approximationType_;
        PWMProfile loProfile_, hiProfile_;

        Options() {}
        Options(long encRes, double maxSpeedRPM, double Kp, double KiF, double Kd, ApproximationType approximationType, const PWMProfile &lp, const PWMProfile &hp)
            : encRes_(encRes), maxSpeedRPM_(maxSpeedRPM), Kp_(Kp), KiF_(KiF), Kd_(Kd), approximationType_(approximationType), loProfile_(lp), hiProfile_(hp) {}
    };

    SDC_Motor(const Options &options, uint8_t dirPin, uint8_t speedPin, volatile long *encPos);

    const Options& GetOptions() const {return options_;}
    bool SetOptions(const Options &options) {return SetOptionsInternal(options, true);}

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
    double GetMaxSpeed() const  {return options_.maxSpeedRPM_*options_.encRes_/60000.0;}
    double GetSpeed() const {return speed_;}
    bool Start (double speed, SDC_MotionType *mt, Ref *ref);
    bool SetSpeed(double speed, Ref *ref);
    bool SetNextPos(double upos, long ts, int flags, Ref *ref);
    void Stop();

#ifdef TEST_SLOW_PWM
    bool StartSlowPWM(int val, long period, double dutyCycle, SDC_MotionType *mt, Ref *ref = NULL);
#endif

private:
    // encapsulated approximation model
    class PWMApproximation
    {
        ApproximationType approximationType_;
        PWMProfile loProfile_;
        double magnitudeCoeff_, periodCoeff_;
    public:
        PWMApproximation() : approximationType_(LINEAR), loProfile_(0, 0, 100), magnitudeCoeff_(1), periodCoeff_(0) {}
        //PWMApproximation(ApproximationType approximationType, const PWMProfile &lp, const PWMProfile &hp) {Init(approximationType, lp, hp);}
        bool Init(ApproximationType approximationType, const PWMProfile &lp, const PWMProfile &hp, bool rejectInvalid);

        void MakeApproximation(int absSp, uint8_t *magnitude, double *period);
    };

    uint8_t dirPin_, speedPin_; // pins
    volatile long *encPos_;

    Options options_;
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

    void SetVal(int val);

#ifdef TEST_SLOW_PWM
    long loPeriod_;
    int testPWMVal_;
#endif
    
    double setpoint_, input_, output_;

    static bool ValidateOptions(const Options &options);
    bool SetOptionsInternal(const Options &options, bool rejectInvalid);
    void DoStop();
    void DoGetPos(double *upos, long *ts);
};

#endif /* SDC_MOTOR_H_ */