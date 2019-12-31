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

class SDC_MotorAdapter : public SDC_MotorItf, private SDC_MotionType
{
public:
    struct Options
    {
        long encRes_;
        double scopeToMotor_;
        double deviationSpeedFactor_, KiF_, KdF_, KpFast2F_, KpFast3F_;
        double diff2_, diff3_;
        long pidPollPeriod_;    // ms
        long adjustPidTmo_;     // ms
        long speedSmoothTime_;  // ms
        Options() {}
        Options(long encRes, double scopeToMotor, double deviationSpeedFactor, double KiF, double KdF, double KpFast2F, double KpFast3F,
                double diff2, double diff3, long pidPollPeriod, long adjustPIDTmo, long speedSmoothTime)
                    :   encRes_(encRes), scopeToMotor_(scopeToMotor), deviationSpeedFactor_(deviationSpeedFactor), KiF_(KiF), KdF_(KdF),
                        KpFast2F_(KpFast2F), KpFast3F_(KpFast3F), diff2_(diff2), diff3_(diff3), pidPollPeriod_(pidPollPeriod), adjustPidTmo_(adjustPIDTmo), speedSmoothTime_(speedSmoothTime) {}
    };

    enum SpeedMode
    {
        STOP,
        REGULAR,
        FAST2,
        FAST3
    };

    SDC_MotorAdapter(const Options &options, volatile long *scopeEncPos, SDC_MotorItf *motor);
    void Init(const Options &options);
    const Options& GetOptions() const {return options_;}

    // call once in setup()
    void Setup();

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run();

    // SDC_MotorItf
    bool IsRunning() const;
    bool GetPhysicalPos(Ref *ref, double *setpoint, double *dbgParam) const;
    bool GetLogicalPos(Ref *ref) const;
    bool GetDeviation(Ref *ref) const;
    double GetMaxSpeed() const {return motor_->GetMaxSpeed() / options_.scopeToMotor_;}
    double GetSpeed() const {return speed_;}
    bool Start (double speed, SDC_MotionType *mt, Ref *ref);
    bool SetSpeed(double speed, Ref *ref);
    bool SetNextPos(double upos, long ts, bool reset, Ref *ref);
    void Stop();

    // MotionType
    bool CanMove(const SDC_MotorItf*) const                 {return mt_ ? mt_->CanMove(this) : true;}
    void MotorStarted(SDC_MotorItf*)                        {if(mt_) mt_->MotorStarted(this); running_ = true;}
    void MotorStopped(SDC_MotorItf*, bool byStopCommand)    {if(mt_) mt_->MotorStopped(this, byStopCommand); DoStop();}

private:
    Options options_;
    volatile long *scopeEncPos_;
    SDC_MotorItf *motor_;
    double Kp_, Ki_, Kd_, KpFast2_, KpFast3_;

    double speedSmooth_;
    bool running_;
    SDC_MotionType *mt_;
    double refScopePos_;
    long ts_;
    double speed_;      // units/ms
    double setpoint_, input_, output_;
    long lastAdjustPID_;
    SpeedMode speedMode_;
    PID pid_;

    void UpdateSpeed(double speed);
    void ReInitializePID(SpeedMode newMode, double speed, double lastError, double Kp, double Ki, double Kd);
    void AdjustPID(double diff, long ts);
    void DoStop();
};



#endif /* SDC_ENCPOSITIONADAPTER_H_ */