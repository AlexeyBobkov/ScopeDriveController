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

#define ADJUST_PID_TMO      1000
#define PID_POLL_PERIOD     300
#define SPEED_SMOOTH        0.3

SDC_MotorAdapter::SDC_MotorAdapter(const Options &options, long encRes, volatile long *scopeEncPos, SDC_MotorItf *motor)
    :   options_(options), scopeEncPos_(scopeEncPos), normalSpeed_(encRes/(24.0*60.0*60.0*1000.0)), motor_(motor),
        maxSpeed_(motor_->GetMaxSpeed() / options_.scopeToMotor_), running_(false), mt_(NULL), output_(0), lastAdjustPID_(0), speedMode_(STOP),
        pid_(&input_, &output_, &setpoint_, 1.0, 0.0, 0.0, DIRECT)
{
    A_ = 1000.0/options_.scopeToMotor_;     // coefficient A in equation d(Pos)/dt = A * Vmotor, i.e., A = 1/scopeToMotor  (*1000 because VMotor is calculated in units/ms, not in units/s)

    SetDevSpeedAndSetTunings(options_.deviationSpeedFactor_);
    options_.KpFast2_ *= 1/A_;
    options_.KpFast3_ *= 1/A_;

    pid_.SetSampleTime(PID_POLL_PERIOD);
    pid_.SetOutputLimits(-maxSpeed_*options_.scopeToMotor_, maxSpeed_*options_.scopeToMotor_);
}


// call once in setup()
void SDC_MotorAdapter::Setup()
{
    lastAdjustPID_ = millis() - ADJUST_PID_TMO;
}

bool SDC_MotorAdapter::SetDevSpeedAndSetTunings(double f)
{
    if(running_)
        return false;

    Kp_ = f * normalSpeed_ * options_.scopeToMotor_;
    if(Kp_ > 1/A_)
        Kp_ = 1/A_;
    Ki_ = options_.Ki_ * (Kp_ * Kp_) * A_ / 4;  // optimal Ki = Kp^2*A/4
    pid_.SetTunings(Kp_, Ki_, 0);
    return true;
}

void SDC_MotorAdapter::UpdateSpeed(double speed)
{
    speed_ = speed;
    /*
    if(speed > maxSpeed_)
        speed_ = maxSpeed_;
    else if(speed < -maxSpeed_)
        speed_ = -maxSpeed_;
    else
        speed_ = speed;
    */
}

void SDC_MotorAdapter::ReInitializePID(SpeedMode newMode, double speed, double Kp, double Ki)
{
    if(speedMode_ != newMode)
    {
        speedMode_ = newMode;
        pid_.SetMode(MANUAL);
        output_ = speed;    // set PID integral sum to predefined value
        pid_.SetMode(AUTOMATIC);
        pid_.SetTunings(Kp, Ki, 0);
    }
}

void SDC_MotorAdapter::AdjustPID(double diff, long ts)
{
    lastAdjustPID_ = ts;

    // mode switch hysteresis
    double diff2, diff3;
    switch(speedMode_)
    {
    case REGULAR:
        diff2 = options_.diff2_ + 1;
        diff3 = options_.diff3_ + 1;
        break;

    case FAST2:
        diff2 = options_.diff2_;
        diff3 = options_.diff3_ + 1;
        break;

    default:
        diff2 = options_.diff2_;
        diff3 = options_.diff3_;
        break;
    }

    if(diff < 0)
        diff = -diff;
    if(diff > diff3)
        ReInitializePID(FAST3, 0, options_.KpFast3_, 0);                        // very fast movement
    else if(diff > diff2)
        ReInitializePID(FAST2, 0, options_.KpFast2_, 0);                        // fast movement
    else
        ReInitializePID(REGULAR, speed_ * options_.scopeToMotor_, Kp_, Ki_);    // regular speed
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
                motor_->SetSpeed(curr + (output_ - curr)*SPEED_SMOOTH);
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
    output_ = speed_ * options_.scopeToMotor_;
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

    if(ref)
        *ref = Ref(refScopePos_, ts_);
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
