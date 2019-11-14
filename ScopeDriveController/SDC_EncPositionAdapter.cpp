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
    :   options_(options), scopeEncPos_(scopeEncPos), motorEncPos_(motorEncPos), motor_(motor),
        maxSpeed_(motor_->GetMaxSpeed() / options_.scopeToMotor_), running_(false), mt_(NULL), output_(0), lastAdjustPID_(0), regularAdjustPID_(true),
        pid_(&input_, &output_, &setpoint_, 1.0, 0.0, 0.0, DIRECT)
{
    double A = 1000.0/options_.scopeToMotor_;
    options_.Kp_ *= 1/A;
    options_.Ki_ *= (options_.Kp_*options_.Kp_) * A /4;

    pid_.SetTunings(options_.Kp_, options_.Ki_, 0);
    pid_.SetSampleTime(100);
    SetMaxOutputLimits();
}

// call once in setup()
void SDC_MotorAdapter::Setup()
{
    lastAdjustPID_ = millis() - ADJUST_PID_TMO;
}

void SDC_MotorAdapter::ReInitializePID(double speed)
{
    pid_.SetMode(MANUAL);
    output_ = speed * options_.scopeToMotor_;
    pid_.SetMode(AUTOMATIC);
}

void SDC_MotorAdapter::SetMaxOutputLimits()
{
    pid_.SetOutputLimits(-maxSpeed_*options_.scopeToMotor_, maxSpeed_*options_.scopeToMotor_);
}

void SDC_MotorAdapter::UpdateSpeed(double speed)
{
    if(speed > maxSpeed_)
        speed = maxSpeed_;
    else if(speed < -maxSpeed_)
        speed = -maxSpeed_;
    speed_ = speed;
    if(running_)
        AdjustPID();
    else
        pid_.SetMode(MANUAL);
}

void SDC_MotorAdapter::AdjustPID()
{
    long ts = millis();
    double diff = refScopePos_ + speed_*(ts - ts_) - *scopeEncPos_;
    if(diff < 0)
        diff = -diff;
    AdjustPID(diff);
}

const double diff1 = 10, diff2 = 6, diff3 = 3;
void SDC_MotorAdapter::AdjustPID(double diff)
{
    long ts = millis();
    lastAdjustPID_ = ts;

    //if(pid_.GetMode() == MANUAL)
    //    output_ = 0;
    //pid_.SetMode(AUTOMATIC);

    if(diff > diff1)
    {
        //SetMaxOutputLimits();
        regularAdjustPID_ = false;
        pid_.SetSampleTime(100);
        pid_.SetTunings(options_.Kp_*2, 0, 0);
    }
    else if(diff > diff2)
    {
        //SetMaxOutputLimits();
        regularAdjustPID_ = false;
        pid_.SetSampleTime(100);
        pid_.SetTunings(options_.Kp_*1.4, 0, 0);
    }
    else
    {
        // regular use case

        if(!regularAdjustPID_)
        {
            ReInitializePID(speed_);
            regularAdjustPID_ = true;
        }

        double sampleTime = (speed_ == 0) ? DBL_MAX : 1/((speed_ > 0) ? speed_ : -speed_);
        if(sampleTime >= 6000)
            pid_.SetSampleTime(2000);               // no more than 2000
        else if(sampleTime >= 300)
            pid_.SetSampleTime(int(sampleTime/3));
        else
            pid_.SetSampleTime(100);                // no less than 100
        //pid_.SetTunings(options_.Kp_/2, options_.Ki_/8, 0);
        pid_.SetTunings(options_.Kp_, options_.Ki_, 0);
    }
}

// call periodically in loop()
// returns true if safe to do some long job
bool SDC_MotorAdapter::Run()
{
    if(running_)
    {
        long ts = millis();
        setpoint_ = round(refScopePos_ + speed_*(ts - ts_));
        input_ = *scopeEncPos_;

        double diff = refScopePos_ + speed_*(ts - ts_) - *scopeEncPos_;
        if(diff < 0)
            diff = -diff;

        if(ts - lastAdjustPID_ > ADJUST_PID_TMO)
            AdjustPID(diff);

        /*
        double motorSpeed;
        if(diff <= diff3)
        {
            motorSpeed = speed_ * options_.scopeToMotor_;
            pid_.SetOutputLimits(motorSpeed * 0.5, motorSpeed * 2);
        }
        else if(diff <= diff2)
            SetMaxOutputLimits();
        */

        pid_.Compute();

        /*
        if(diff <= diff3)
        {
            double low = motorSpeed * 0.9;
            if(output_ < low)
                output_ = low;
            else
            {
                double high = motorSpeed * 1.11;
                if(output_ > high)
                    output_ = high;
            }
        }
        */
        motor_->SetSpeed(output_);
    }
    return true;
}

bool SDC_MotorAdapter::IsRunning() const
{
    return running_;
}

bool SDC_MotorAdapter::GetPos(Ref *ref, long *setpoint, long *dbgParam) const
{
    if(ref)
        *ref = Ref(*scopeEncPos_, millis());
    if(setpoint)
        *setpoint = setpoint_;
    if(dbgParam)
        *dbgParam = output_ * 1000;
    return motor_->IsRunning();
}

bool SDC_MotorAdapter::Start (double speed, SDC_MotionType *mt, Ref *ref)
{
    if(running_ || motor_->IsRunning())
        return false;
    mt_ = mt;

    Ref r;
    bool ok = motor_->Start(speed * options_.scopeToMotor_, this, &r);
    if(ok && running_)
    {
        refScopePos_ = *scopeEncPos_;
        refMotorPos_ = r.upos_;
        ts_ = r.ts_;
        UpdateSpeed(speed);
        if(ref)
            *ref = Ref(refScopePos_, ts_);
        ReInitializePID(speed_);
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
    ReInitializePID(speed);
    return motor_->SetSpeed(speed * options_.scopeToMotor_, NULL);
}

bool SDC_MotorAdapter::SetNextPos(long upos, long ts, bool reset, Ref *ref)
{
    if(!running_)
        return false;

    long sposCurr, mposCurr, tsCurr;
    DoGetPos(&sposCurr, &mposCurr, &tsCurr);
    if(ref)
        *ref = Ref(sposCurr, tsCurr);
    if((ts > tsCurr ? ts - tsCurr : tsCurr - ts) > 10)   // ignore if timestamps are closer than 10 ms, due to bad accuracy
    {
        refScopePos_ = sposCurr;
        refMotorPos_ = mposCurr;
        ts_ = tsCurr;
        UpdateSpeed(double(upos - refScopePos_)/double(ts - ts_));
        ReInitializePID(speed_);
        return motor_->SetSpeed(speed_ * options_.scopeToMotor_, NULL);
    }
    return false;
}

void SDC_MotorAdapter::Stop()
{
    if(running_)
    {
        motor_->Stop();
        UpdateSpeed(0);
        mt_ = NULL;
    }
}

void SDC_MotorAdapter::DoGetPos(long *spos, long *mpos, long *ts)
{
    *ts = millis();
    *spos = *scopeEncPos_;
    *mpos = *motorEncPos_;
}
