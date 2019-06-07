/*
 * SDC_EncPositionAdapter.cpp
 *
 * Created: 6/1/2019 3:36:09 PM
 *  Author: Alexey
 */

#include <Arduino.h>
#include <float.h>

#include "SDC_EncPositionAdapter.h"

const long ADJUST_PID_TMO = 1000;

SDC_MotorAdapter::SDC_MotorAdapter(const Options &options, volatile long *scopeEncPos, volatile long *motorEncPos, SDC_MotorItf *motor)
    :   options_(options), scopeEncPos_(scopeEncPos), motorEncPos_(motorEncPos), motor_(motor), running_(false), output_(0), lastAdjustPID_(0),
        pid_(&input_, &output_, &setpoint_, options.Kp_, options.Ki_, 0.0, P_ON_E, DIRECT, true)
{
    pid_.SetOutputLimits(-8,10);
}

// call once in setup()
void SDC_MotorAdapter::Setup()
{
    lastAdjustPID_ = millis() - ADJUST_PID_TMO;
}

void SDC_MotorAdapter::AdjustPID()
{
    lastAdjustPID_ = millis();
    if(speed_ == 0)
        pid_.SetMode(MANUAL);
    else
    {
        pid_.SetMode(AUTOMATIC);

        double sampleTime = 1/((speed_ > 0) ? speed_ : -speed_);
        double diff = refScopePos_ + speed_*(millis() - ts_) - *scopeEncPos_;

        if(diff < 0)
            diff = -diff;
        if(diff > 25)
        {
            // very far from setpoint
            pid_.SetSampleTime(100);
            //pid_.SetTunings(options_.Kp_*18, 0, 0);
            pid_.SetTunings(options_.Kp_*sampleTime/150, 0, 0);
            pid_.SetOutputLimits(-DBL_MAX, DBL_MAX);
        }
        else if(diff > 200000/sampleTime)
        {
            // closer to setpoint
            pid_.SetSampleTime(200);
            //pid_.SetTunings(options_.Kp_*9, 0, 0);
            pid_.SetTunings(options_.Kp_*sampleTime/300, 0, 0);
            pid_.SetOutputLimits(-100,100);
        }
        else if(diff > 100000/sampleTime)
        {
            // even closer to setpoint
            pid_.SetSampleTime(300);
            //pid_.SetTunings(options_.Kp_*3, 0, 0);
            pid_.SetTunings(options_.Kp_*sampleTime/1000, 0, 0);
            pid_.SetOutputLimits(-8,10);
        }
        else if(sampleTime >= 100)
        {
            // regular use case
            pid_.SetSampleTime(int(sampleTime/3));
            pid_.SetTunings(options_.Kp_, options_.Ki_/3, 0);
            pid_.SetOutputLimits(-8,10);
        }
        else
        {
            // very fast speed
            double ratio = 100/sampleTime;
            pid_.SetSampleTime(100);
            pid_.SetTunings(options_.Kp_/ratio/10, 0, 0);
            pid_.SetOutputLimits(-0.025,0.025);
        }
    }
}

// call periodically in loop()
// returns true if safe to do some long job
bool SDC_MotorAdapter::Run()
{
    if(running_ && motor_->IsRunning())
    {
        long ts = millis();
        if(ts - lastAdjustPID_ > ADJUST_PID_TMO)
            AdjustPID();

        long spos = *scopeEncPos_;
        if(speed_ > 0)
        {
            setpoint_ = refScopePos_ + speed_*(ts - ts_);
            input_ = spos;
        }
        else
        {
            setpoint_ = -refScopePos_ - speed_*(ts - ts_);
            input_ = -spos;
        }
        pid_.Compute();
        motor_->SetSpeed(speed_ * options_.scopeToMotor_ * (1 + output_));
    }
    return true;
}

bool SDC_MotorAdapter::IsRunning() const
{
    return running_ && motor_->IsRunning();
}

bool SDC_MotorAdapter::GetPos(Ref *ref, long *setpoint) const
{
    if(ref)
        *ref = Ref(*scopeEncPos_, millis());
    if(setpoint)
        //*setpoint = setpoint_;
        *setpoint = output_ * 1000;
    return motor_->IsRunning();
}

bool SDC_MotorAdapter::Start (double speed, MotionType *mt, Ref *ref)
{
    Ref r;
    bool ok = motor_->Start(speed * options_.scopeToMotor_ * (1 + output_), mt, &r);
    if(ok)
    {
        running_ = true;
        refScopePos_ = *scopeEncPos_;
        refMotorPos_ = r.upos_;
        ts_ = r.ts_;
        speed_ = speed;
        AdjustPID();
        if(ref)
            *ref = Ref(refScopePos_, ts_);
    }
    return ok;
}

bool SDC_MotorAdapter::SetSpeed(double speed, Ref *ref)
{
    if(!running_ || !motor_->IsRunning())
        return false;
    if(speed == speed_)
        return true;

    DoGetPos(&refScopePos_, &refMotorPos_, &ts_);
    speed_ = speed;
    AdjustPID();
    if(ref)
        *ref = Ref(refScopePos_, ts_);
    return motor_->SetSpeed(speed * options_.scopeToMotor_ * (1 + output_), NULL);
}

bool SDC_MotorAdapter::SetNextPos(long upos, long ts, Ref *ref)
{
    if(!running_ || !motor_->IsRunning())
        return false;

    long sposCurr, mposCurr, tsCurr;
    DoGetPos(&sposCurr, &mposCurr, &tsCurr);
    if((ts > tsCurr ? ts - tsCurr : tsCurr - ts) > 10)   // ignore if timestamps are closer than 10 ms, due to bad accuracy
    {
        refScopePos_ = sposCurr;
        refMotorPos_ = mposCurr;
        ts_ = tsCurr;
        speed_ = (upos - refScopePos_)/(ts - ts_);
        AdjustPID();
        if(ref)
            *ref = Ref(refScopePos_, ts_);
        return motor_->SetSpeed(speed_ * options_.scopeToMotor_ * (1 + output_), NULL);
    }
    return false;
}

void SDC_MotorAdapter::Stop()
{
    if(running_ && motor_->IsRunning())
    {
        DoGetPos(&refScopePos_, &refMotorPos_, &ts_);
        motor_->Stop();
        running_ = false;
        speed_ = 0;
        AdjustPID();
    }
}

void SDC_MotorAdapter::DoGetPos(long *spos, long *mpos, long *ts)
{
    *ts = millis();
    *spos = *scopeEncPos_;
    *mpos = *motorEncPos_;
}
