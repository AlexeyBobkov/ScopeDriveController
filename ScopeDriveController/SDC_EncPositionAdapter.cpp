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

SDC_MotorAdapter::SDC_MotorAdapter(const Options &options, volatile long *scopeEncPos, SDC_MotorItf *motor)
    :   options_(options), scopeEncPos_(scopeEncPos), motor_(motor),
        maxSpeed_(motor_->GetMaxSpeed() / options_.scopeToMotor_), running_(false), mt_(NULL), output_(0), lastAdjustPID_(0), regularAdjustPID_(true),
        pid_(&input_, &output_, &setpoint_, 1.0, 0.0, 0.0, DIRECT)
{
    double A = 1000.0/options_.scopeToMotor_;
    options_.Kp_ *= 1/A;
    options_.KpFast_ *= 1/A;
    options_.Ki_ *= (options_.Kp_*options_.Kp_) * A / 4;

    pid_.SetTunings(options_.Kp_, options_.Ki_, 0);
    pid_.SetSampleTime(300);
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
    lastAdjustPID_ = ts;

    double diff = refScopePos_ + speed_*(ts - ts_) - *scopeEncPos_;
    if(diff < 0)
        diff = -diff;

    if(diff > options_.diff1_)
    {
        // fast movement

        SetMaxOutputLimits();
        regularAdjustPID_ = false;
        pid_.SetTunings(options_.KpFast_, 0, 0);
    }
    else
    {
        // regular use case

        if(!regularAdjustPID_)
        {
            ReInitializePID(speed_);
            regularAdjustPID_ = true;
        }

        if(diff > options_.diff2_)
            SetMaxOutputLimits();
        else
        {
            // If close to setpoint, no backward movement!
            //if(speed_ > 0)
            //    pid_.SetOutputLimits(0, maxSpeed_*options_.scopeToMotor_);
            //else
            //    pid_.SetOutputLimits(-maxSpeed_*options_.scopeToMotor_, 0);
            double motorSpeed = speed_*options_.scopeToMotor_;
            if(speed_ > 0)
                pid_.SetOutputLimits(motorSpeed*0.5, motorSpeed*1.5);
            else
                pid_.SetOutputLimits(motorSpeed*1.5, motorSpeed*0.5);
        }
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

        if(ts - lastAdjustPID_ > ADJUST_PID_TMO)
            AdjustPID();

        if(pid_.Compute())
        {
            double curr = motor_->GetSpeed();
            double next = curr + (output_ - curr)*0.07;
            //motor_->SetSpeed(output_);
            motor_->SetSpeed(next);
        }
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

    long sposCurr, tsCurr;
    DoGetPos(&sposCurr, &tsCurr);

    //refScopePos_ = sposCurr;              // <-- Incorrect!
    refScopePos_ += speed_*(tsCurr - ts_);  // <-- Correct! To keep the PID state, we must use current LOGICAL position as the next reference point.
    ts_ = tsCurr;

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

    long sposCurr, tsCurr;
    DoGetPos(&sposCurr, &tsCurr);
    if(ref)
        *ref = Ref(sposCurr, tsCurr);
    if((ts > tsCurr ? ts - tsCurr : tsCurr - ts) > 10)   // ignore if timestamps are closer than 10 ms, due to bad accuracy
    {
        //refScopePos_ = sposCurr;              // <-- Incorrect!
        refScopePos_ += speed_*(tsCurr - ts_);  // <-- Correct! To keep the PID state, we must use current LOGICAL position as the next reference point.
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

void SDC_MotorAdapter::DoGetPos(long *spos, long *ts)
{
    *ts = millis();
    *spos = *scopeEncPos_;
}
