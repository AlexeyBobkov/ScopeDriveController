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
        maxSpeed_(motor_->GetMaxSpeed() / options_.scopeToMotor_), running_(false), mt_(NULL), output_(0), lastAdjustPID_(0), speedMode_(REGULAR),
        pid_(&input_, &output_, &setpoint_, 1.0, 0.0, 0.0, DIRECT)
{
    double A = 1000.0/options_.scopeToMotor_;
    options_.Kp_ *= 1/A;
    options_.KpFast2_ *= 1/A;
    options_.KpFast3_ *= 1/A;
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

    if(diff > options_.diff3_)
    {
        // very fast movement
        if(speedMode_ != FAST3)
        {
            speedMode_ = FAST3;
            SetMaxOutputLimits();
            pid_.SetTunings(options_.KpFast3_, 0, 0);
        }
    }
    else if(diff > options_.diff2_)
    {
        // fast movement
        if(speedMode_ != FAST2)
        {
            speedMode_ = FAST2;
            pid_.SetOutputLimits(-maxSpeed_*options_.scopeToMotor_/4, maxSpeed_*options_.scopeToMotor_/4);
            pid_.SetTunings(options_.KpFast2_, 0, 0);
        }
    }
    else
    {
        // regular use case
        if(speedMode_ != REGULAR)
        {
            speedMode_ = REGULAR;

            // re-initialize PID
            pid_.SetMode(MANUAL);
            output_ = speed_ * options_.scopeToMotor_;
            pid_.SetMode(AUTOMATIC);
            motor_->SetSpeed(output_);
        }

        if(diff > options_.diff1_)
            SetMaxOutputLimits();
        else
        {
            double motorSpeed = speed_*options_.scopeToMotor_;
            if(speed_ > 0)
                pid_.SetOutputLimits(motorSpeed*0.5, motorSpeed*2);
            else
                pid_.SetOutputLimits(motorSpeed*2, motorSpeed*0.5);
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

        if(speedMode_ != REGULAR || ts - lastAdjustPID_ > ADJUST_PID_TMO)
            AdjustPID();

        if(pid_.Compute())
        {
            if(speedMode_ != REGULAR)
                motor_->SetSpeed(output_);
            else
            {
                double curr = motor_->GetSpeed();
                motor_->SetSpeed(curr + (output_ - curr)*0.07);
            }
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

        if(ref)
            *ref = Ref(refScopePos_, ts_);
        output_ = speed * options_.scopeToMotor_;
        pid_.SetMode(AUTOMATIC);
        speedMode_ = REGULAR;
        UpdateSpeed(speed);
    }
    return ok;
}

bool SDC_MotorAdapter::SetSpeed(double speed, Ref *ref)
{
    if(!running_)
        return false;
    if(speed == speed_)
        return true;

    long tsCurr = millis();
    refScopePos_ += speed_*(tsCurr - ts_);  // To keep the PID state, we must use current LOGICAL position as the next reference point.
    ts_ = tsCurr;

    if(ref)
        *ref = Ref(refScopePos_, ts_);
    UpdateSpeed(speed);
    return motor_->SetSpeed(speed * options_.scopeToMotor_, NULL);
}

bool SDC_MotorAdapter::SetNextPos(long upos, long ts, bool reset, Ref *ref)
{
    if(!running_)
        return false;

    long tsCurr = millis();
    if((ts > tsCurr ? ts - tsCurr : tsCurr - ts) > 10)   // ignore if timestamps are closer than 10 ms, due to bad accuracy
    {
        refScopePos_ += speed_*(tsCurr - ts_);  // To keep the PID state, we must use current LOGICAL position as the next reference point.
        ts_ = tsCurr;

        if(ref)
            *ref = Ref(refScopePos_, ts_);
        UpdateSpeed(double(upos - refScopePos_)/double(ts - ts_));
        return motor_->SetSpeed(speed_ * options_.scopeToMotor_, NULL);
    }
    else if(ref)
        *ref = Ref(*scopeEncPos_, tsCurr);
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
