/*
 * SDC_EncPositionAdapter.cpp
 *
 * Created: 6/1/2019 3:36:09 PM
 *  Author: Alexey
 */

#include <Arduino.h>
#include <float.h>

#include "SDC_EncPositionAdapter.h"

const long ADJUST_PID_TMO_REGULAR = 3000;
const long ADJUST_PID_TMO_RETRACK = 1000;

SDC_MotorAdapter::SDC_MotorAdapter(const Options &options, volatile long *scopeEncPos, volatile long *motorEncPos, SDC_MotorItf *motor)
    :   options_(options), scopeEncPos_(scopeEncPos), motorEncPos_(motorEncPos), motor_(motor),
        maxSpeed_(motor_->GetMaxSpeed() / options_.scopeToMotor_), running_(false), mt_(NULL), output_(0), lastAdjustPID_(0), regularTmo_(true), maxOut_(DBL_MAX - 1),
        pid_(&input_, &output_, &setpoint_, options.Kp_, options.Ki_, 0.0, DIRECT)
{
    pid_.SetSampleTime(100);
    //pid_.SetOutputLimits(-8,10);
}

// call once in setup()
void SDC_MotorAdapter::Setup()
{
    lastAdjustPID_ = millis() - ADJUST_PID_TMO_REGULAR;
}

void SDC_MotorAdapter::UpdateSpeed(double speed)
{
    if(speed > maxSpeed_)
        speed = maxSpeed_;
    else if(speed < -maxSpeed_)
        speed = -maxSpeed_;
    speed_ = speed;
    AdjustPID();
}

void SDC_MotorAdapter::AdjustPID()
{
    lastAdjustPID_ = millis();
    if(speed_ == 0)
    {
        maxOut_ = DBL_MAX - 1;
        pid_.SetMode(MANUAL);
    }
    else
    {
        if(pid_.GetMode() == MANUAL)
            output_ = 0;
        pid_.SetMode(AUTOMATIC);

        maxOut_ = maxSpeed_/(speed_ > 0 ? speed_ : -speed_);
        pid_.SetOutputLimits(-maxOut_ - 1, maxOut_ - 1);

        double sampleTime = 1/((speed_ > 0) ? speed_ : -speed_);
        double diff = refScopePos_ + speed_*(millis() - ts_) - *scopeEncPos_;
        if(diff < 0)
            diff = -diff;

        const double diff1 = 25;
        const double diff2 = 14;//sampleTime > 14000 ? 200000/sampleTime : 14;
        const double diff3 = 6;//sampleTime > 14000 ? 100000/sampleTime : 7;

        if(diff > diff1)
        {
            // very far from setpoint
            regularTmo_ = false;
            pid_.SetSampleTime(100);
            pid_.SetTunings(options_.Kp_*18, 0, 0);
            //pid_.SetTunings(options_.Kp_*sampleTime/150, 0, 0);
        }
        else if(diff > diff2)
        {
            // closer to setpoint
            regularTmo_ = false;
            pid_.SetSampleTime(200);
            pid_.SetTunings(options_.Kp_*9, 0, 0);
            //pid_.SetTunings(options_.Kp_*sampleTime/300, 0, 0);
        }
        else if(diff > diff3)
        {
            // even closer to setpoint
            regularTmo_ = false;
            pid_.SetSampleTime(300);
            pid_.SetTunings(options_.Kp_*3, 0, 0);
            //pid_.SetTunings(options_.Kp_*sampleTime/1000, 0, 0);
        }
        else
        {
            // regular use case
            regularTmo_ = true;

            if(sampleTime >= 6000)
                pid_.SetSampleTime(2000);               // no more than 2000
            else if(sampleTime >= 300)
                pid_.SetSampleTime(int(sampleTime/3));
            else
                pid_.SetSampleTime(100);                // no less than 100
            pid_.SetTunings(options_.Kp_, options_.Ki_, 0);
        }
    }
}

// call periodically in loop()
// returns true if safe to do some long job
bool SDC_MotorAdapter::Run()
{
    if(running_ && speed_ != 0)
    {
        long ts = millis();
        long spos = *scopeEncPos_;

        setpoint_ = round(refScopePos_ + speed_*(ts - ts_))/(1000*speed_);
        input_ = spos/(1000*speed_);

        if(ts - lastAdjustPID_ > regularTmo_ ? ADJUST_PID_TMO_REGULAR : ADJUST_PID_TMO_RETRACK)
            AdjustPID();

        pid_.Compute();
        motor_->SetSpeed(speed_ * options_.scopeToMotor_ * (1 + output_));
    }
    return true;
}

bool SDC_MotorAdapter::IsRunning() const
{
    return running_;
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

bool SDC_MotorAdapter::Start (double speed, SDC_MotionType *mt, Ref *ref)
{
    if(running_ || motor_->IsRunning())
        return false;
    mt_ = mt;

    Ref r;
    bool ok = motor_->Start(speed * options_.scopeToMotor_ /* * (1 + output_)*/, this, &r);
    if(ok)
    {
        refScopePos_ = *scopeEncPos_;
        refMotorPos_ = r.upos_;
        ts_ = r.ts_;
        UpdateSpeed(speed);
        if(ref)
            *ref = Ref(refScopePos_, ts_);
    }
    return ok;
}

bool SDC_MotorAdapter::SetSpeed(double speed, Ref *ref)
{
    if(!running_)
        return false;
    if(speed == speed_)
        return true;

    DoGetPos(&refScopePos_, &refMotorPos_, &ts_);
    UpdateSpeed(speed);
    if(ref)
        *ref = Ref(refScopePos_, ts_);
    return motor_->SetSpeed(speed * options_.scopeToMotor_ /* * (1 + output_)*/, NULL);
}

bool SDC_MotorAdapter::SetNextPos(long upos, long ts, Ref *ref)
{
    if(!running_)
        return false;

    long sposCurr, mposCurr, tsCurr;
    DoGetPos(&sposCurr, &mposCurr, &tsCurr);
    if((ts > tsCurr ? ts - tsCurr : tsCurr - ts) > 10)   // ignore if timestamps that are closer than 10 ms, due to bad accuracy
    {
        refScopePos_ = sposCurr;
        refMotorPos_ = mposCurr;
        ts_ = tsCurr;
        UpdateSpeed((upos - refScopePos_)/(ts - ts_));
        if(ref)
            *ref = Ref(refScopePos_, ts_);
        return motor_->SetSpeed(speed_ * options_.scopeToMotor_ /* * (1 + output_)*/, NULL);
    }
    return false;
}

void SDC_MotorAdapter::Stop()
{
    if(running_)
    {
        DoGetPos(&refScopePos_, &refMotorPos_, &ts_);
        motor_->Stop();
        mt_ = NULL;
        UpdateSpeed(0);
    }
}

void SDC_MotorAdapter::DoGetPos(long *spos, long *mpos, long *ts)
{
    *ts = millis();
    *spos = *scopeEncPos_;
    *mpos = *motorEncPos_;
}
