/*
 * SDC_EncPositionAdapter.cpp
 *
 * Created: 6/1/2019 3:36:09 PM
 *  Author: Alexey
 */

#include <Arduino.h>
#include <float.h>

#include "SDC_Configuration.h"
#include "SDC_EncPositionAdapter.h"

const long ADJUST_PID_TMO = 1000;

SDC_MotorAdapter::SDC_MotorAdapter(const Options &options, long encRes, volatile long *scopeEncPos, SDC_MotorItf *motor)
    :   options_(options), normalSpeed_(encRes/(24.0*60.0*60.0*1000.0)), scopeEncPos_(scopeEncPos), motor_(motor),
        maxSpeed_(motor_->GetMaxSpeed() / options_.scopeToMotor_), running_(false), mt_(NULL), output_(0), lastAdjustPID_(0), speedMode_(STOP),
        diff1_(0), diff2_(0), diff3_(0), pid_(&input_, &output_, &setpoint_, 1.0, 0.0, 0.0, DIRECT)
{
    double A = 1000.0/options_.scopeToMotor_;
    options_.Kp_ *= 1/A;
    options_.KpFast2_ *= 1/A;
    options_.KpFast3_ *= 1/A;
    options_.Ki_ *= (options_.Kp_*options_.Kp_) * A / 4;

    maxMotorSpeedDeviation_ = options_.scopeToMotor_*normalSpeed_/3;
    if(maxMotorSpeedDeviation_ < options_.Kp_)
        maxMotorSpeedDeviation_ = options_.Kp_;

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
    {
        double s = speed > 0 ? speed : -speed;

        diff1_ = options_.diff1_*s/normalSpeed_;
        if(diff1_ < 2)
            diff1_ = 2;

        diff2_ = options_.diff2_*s/normalSpeed_;
        if(diff2_ < 3)
            diff2_ = 3;

        diff3_ = options_.diff3_*s/normalSpeed_;
        if(diff3_ < 7)
            diff3_ = 7;
    }
}

void SDC_MotorAdapter::AdjustPID(double diff, long ts)
{
    lastAdjustPID_ = ts;
    if(diff > diff3_)
    {
        // very fast movement
        if(speedMode_ != FAST3)
        {
            speedMode_ = FAST3;
            SetMaxOutputLimits();
            pid_.SetTunings(options_.KpFast3_, 0, 0);
        }
    }
    else if(diff > diff2_)
    {
        // fast movement
        if(speedMode_ != FAST2)
        {
            speedMode_ = FAST2;
            pid_.SetOutputLimits(-maxSpeed_*options_.scopeToMotor_/8, maxSpeed_*options_.scopeToMotor_/8);
            pid_.SetTunings(options_.KpFast2_, 0, 0);
        }
    }
    else
    {
        // regular use case
        if(diff > diff1_)
            SetMaxOutputLimits();
        else
        {
            double motorSpeed = speed_*options_.scopeToMotor_;
            pid_.SetOutputLimits(motorSpeed - maxMotorSpeedDeviation_, motorSpeed + maxMotorSpeedDeviation_);
        }
        if(speedMode_ != REGULAR)
        {
            speedMode_ = REGULAR;

            // re-initialize PID
            pid_.SetMode(MANUAL);
            output_ = speed_ * options_.scopeToMotor_;
            pid_.SetMode(AUTOMATIC);
            motor_->SetSpeed(output_);
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
            AdjustPID(setpoint_ - input_, ts);

        if(pid_.Compute())
        {
            if(speedMode_ != REGULAR)
                motor_->SetSpeed(output_);
            else
            {
                double curr = motor_->GetSpeed();
                motor_->SetSpeed(curr + (output_ - curr)*0.15);
            }
        }
    }
    return true;
}

bool SDC_MotorAdapter::IsRunning() const
{
    return running_;
}

bool SDC_MotorAdapter::GetPhysicalPos(Ref *ref, double *setpoint, double *dbgParam) const
{
    if(ref)
        *ref = Ref(*scopeEncPos_, millis());
    if(setpoint)
        *setpoint = setpoint_;
    if(dbgParam)
        *dbgParam = motor_->GetSpeed() * 1000;
    return motor_->IsRunning();
}

bool SDC_MotorAdapter::GetLogicalPos(Ref *ref) const
{
    if(ref)
    {
        long tsCurr = millis();
        *ref = Ref(running_ ? round(refScopePos_ + speed_*(tsCurr - ts_)) : *scopeEncPos_, tsCurr);
    }
    return running_;
}

bool SDC_MotorAdapter::GetDeviation(Ref *ref) const
{
    if(ref)
    {
        long tsCurr = millis();
        *ref = Ref(running_ ? round(refScopePos_ + speed_*(tsCurr - ts_) - *scopeEncPos_) : 0, tsCurr);
    }
    return running_;
}

bool SDC_MotorAdapter::Start (double speed, SDC_MotionType *mt, Ref *ref)
{
    if(running_ || motor_->IsRunning())
        return false;
    mt_ = mt;

    bool ok = motor_->Start(speed * options_.scopeToMotor_, this, NULL);
    if(!ok || !running_)
    {
        DoStop();
        return false;
    }

    refScopePos_ = *scopeEncPos_;
    ts_ = millis();

    if(ref)
        *ref = Ref(refScopePos_, ts_);
    UpdateSpeed(speed);
    pid_.SetMode(AUTOMATIC);
    AdjustPID(0, ts_);
    return true;
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
    return true;
}

bool SDC_MotorAdapter::SetNextPos(double upos, long ts, bool reset, Ref *ref)
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
        UpdateSpeed((upos - refScopePos_)/double(ts - ts_));
        return true;
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
        if(running_)
            DoStop();
    }
}

void SDC_MotorAdapter::DoStop()
{
    running_ = false;
    UpdateSpeed(0);
    pid_.SetMode(MANUAL);
    speedMode_ = STOP;
    mt_ = NULL;
}
