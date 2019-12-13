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
        double deviationSpeedFactor_, Ki_, KpFast2_, KpFast3_;
        double diff2_, diff3_;
        Options() {}
        Options(double scopeToMotor, double deviationSpeedFactor, double Ki, double KpFast2, double KpFast3, double diff2, double diff3)
            : scopeToMotor_(scopeToMotor), deviationSpeedFactor_(deviationSpeedFactor), Ki_(Ki), KpFast2_(KpFast2), KpFast3_(KpFast3), diff2_(diff2), diff3_(diff3) {}
    };

    enum SpeedMode
    {
        STOP,
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

    bool SetDevSpeedAndSetTunings(double f);

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
    double normalSpeed_;
    SDC_MotorItf *motor_;
    double A_, Kp_, Ki_;

    double maxSpeed_;
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
    void ReInitializePID(SpeedMode newMode, double speed, double Kp, double Ki);
    void AdjustPID(double diff, long ts);
    void DoStop();
};



#endif /* SDC_ENCPOSITIONADAPTER_H_ */