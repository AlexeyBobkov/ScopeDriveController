/*
 * SDC_EncPositionAdapter.h
 *
 * Created: 6/1/2019 3:35:46 PM
 *  Author: Alexey
 */


#ifndef SDC_ENCPOSITIONADAPTER_H_
#define SDC_ENCPOSITIONADAPTER_H_

#include <PID_v1.h>
#include "SDC_Motor.h"

class SDC_MotorAdapter : public SDC_MotorItf, private SDC_MotionType
{
public:
    struct Options
    {
        double scopeToMotor_;
        double Kp_, Ki_, KpFast2_, KpFast3_;
        double diff1_, diff2_, diff3_;
        Options() {}
        Options(double scopeToMotor, double Kp, double Ki, double KpFast2, double KpFast3, double diff1, double diff2, double diff3)
            : scopeToMotor_(scopeToMotor), Kp_(Kp), Ki_(Ki), KpFast2_(KpFast2), KpFast3_(KpFast3), diff1_(diff1), diff2_(diff2), diff3_(diff3) {}
    };

    enum SpeedMode
    {
        REGULAR,
        FAST2,
        FAST3
    };

    SDC_MotorAdapter(const Options &options, long encRes, volatile long *scopeEncPos, SDC_MotorItf *motor);

    // call once in setup()
    void Setup();

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run();

    // SDC_MotorItf
    bool IsRunning() const;
    bool GetPhysicalPos(Ref *ref, long *setpoint, long *dbgParam) const;
    bool GetLogicalPos(Ref *ref) const;
    bool GetDeviation(Ref *ref) const;
    double GetMaxSpeed() const {return motor_->GetMaxSpeed() / options_.scopeToMotor_;}
    double GetSpeed() const {return speed_;}
    bool Start (double speed, SDC_MotionType *mt, Ref *ref);
    bool SetSpeed(double speed, Ref *ref);
    bool SetNextPos(long upos, long ts, bool reset, Ref *ref);
    void Stop();

    // MotionType
    bool CanMove(const SDC_MotorItf*) const                 {return mt_ ? mt_->CanMove(this) : true;}
    void MotorStarted(SDC_MotorItf*)                        {if(mt_) mt_->MotorStarted(this); running_ = true;}
    void MotorStopped(SDC_MotorItf*, bool byStopCommand)    {if(mt_) mt_->MotorStopped(this, byStopCommand); running_ = false;}

private:
    Options options_;
    double normalSpeed_;
    volatile long *scopeEncPos_;
    SDC_MotorItf *motor_;

    double maxSpeed_;
    bool running_;
    SDC_MotionType *mt_;
    long refScopePos_;
    long ts_;
    double speed_;      // units/ms
    double setpoint_, input_, output_;
    long lastAdjustPID_;
    SpeedMode speedMode_;
    double diff1_, diff2_, diff3_;
    PID pid_;

    void UpdateSpeed(double speed);
    void SetMaxOutputLimits();
    void AdjustPID();
};



#endif /* SDC_ENCPOSITIONADAPTER_H_ */