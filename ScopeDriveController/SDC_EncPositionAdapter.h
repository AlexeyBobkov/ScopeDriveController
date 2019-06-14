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
        double scopeToMotor_;
        double Kp_, Ki_;
        Options() {}
        Options(double scopeToMotor, double Kp, double Ki) : scopeToMotor_(scopeToMotor), Kp_(Kp), Ki_(Ki) {}
    };

    SDC_MotorAdapter(const Options &options, volatile long *scopeEncPos, volatile long *motorEncPos, SDC_MotorItf *motor);

    // call once in setup()
    void Setup();

    // call periodically in loop()
    // returns true if safe to do some long job
    bool Run();

    // SDC_MotorItf
    bool IsRunning() const;
    bool GetPos(Ref *ref, long *setpoint, long *dbgParam) const;
    double GetMaxSpeed() const {return motor_->GetMaxSpeed() / options_.scopeToMotor_;}
    bool Start (double speed, SDC_MotionType *mt, Ref *ref);
    bool SetSpeed(double speed, Ref *ref);
    bool SetNextPos(long upos, long ts, Ref *ref);
    void Stop();

    // MotionType
    bool CanMove(const SDC_MotorItf*) const                 {return mt_ ? mt_->CanMove(this) : true;}
    void MotorStarted(SDC_MotorItf*)                        {if(mt_) mt_->MotorStarted(this); running_ = true;}
    void MotorStopped(SDC_MotorItf*, bool byStopCommand)    {if(mt_) mt_->MotorStopped(this, byStopCommand); running_ = false;}

private:
    Options options_;
    volatile long *scopeEncPos_, *motorEncPos_;
    SDC_MotorItf *motor_;

    double maxSpeed_;
    bool running_;
    SDC_MotionType *mt_;
    long refScopePos_;
    long refMotorPos_;
    long ts_;
    double speed_;      // units/ms
    double setpoint_, input_, output_;
    long lastAdjustPID_;
    bool regularAdjustPID_;
    PID pid_;

    void DoGetPos(long *spos, long *mpos, long *ts);
    void UpdateSpeed(double speed);
    void ReInitializePID(double speed);
    void AdjustPID();
};



#endif /* SDC_ENCPOSITIONADAPTER_H_ */