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

SDC_MotorAdapter::SDC_MotorAdapter(const Options &options, volatile long *scopeEncPos, SDC_MotorItf *motor)
    :   scopeEncPos_(scopeEncPos), motor_(motor), running_(false), mt_(NULL), output_(0), lastAdjustPID_(0), speedMode_(STOP),
        pid_(&input_, &output_, &setpoint_, 1.0, 0.0, 0.0, DIRECT)
{
    Init(options);
}


void SDC_MotorAdapter::Init(const Options &options)
{
    options_        = options;

    speedSmooth_    = double(options_.pidPollPeriod_)/double(options_.speedSmoothTime_);

    double A = 1000.0/options_.scopeToMotor_;     // coefficient A in equation d(Pos)/dt = A * Vmotor, i.e., A = 1/scopeToMotor  (*1000 because VMotor is calculated in units/ms, not in units/s)
    double normalSpeed = options_.encRes_/(24.0*60.0*60.0*1000.0);

    Kp_ = options_.deviationSpeedFactor_ * normalSpeed * options_.scopeToMotor_;
    if(Kp_ > 1/A)
        Kp_ = 1/A;

    Kd_ = options_.KdF_ * options_.deviationSpeedFactor_ * normalSpeed * options_.scopeToMotor_;
    if(Kd_ > 1/A)
        Kd_ = 1/A;

    Ki_ = options_.KiF_ * (Kp_ * Kp_) * A / (4 * (1 + A * Kd_));    // optimal Ki = A*Kp^2/(4*(1 + A*Kd))

    pid_.SetTunings(Kp_, Ki_, Kd_);

    KpFast2_ = options_.KpFast2F_ * 1/A;
    KpFast3_ = options_.KpFast3F_ * 1/A;

    pid_.SetSampleTime(options_.pidPollPeriod_);

    double maxSpeed = motor_->GetMaxSpeed() / options_.scopeToMotor_;
    pid_.SetOutputLimits(-maxSpeed*options_.scopeToMotor_, maxSpeed*options_.scopeToMotor_);
}


// call once in setup()
void SDC_MotorAdapter::Setup()
{
    lastAdjustPID_ = millis() - options_.adjustPidTmo_;
}

void SDC_MotorAdapter::UpdateSpeed(double speed)
{
    speed_ = speed;
}

void SDC_MotorAdapter::ReInitializePID(SpeedMode newMode, double speed, double lastError, double Kp, double Ki, double Kd)
{
    if(speedMode_ != newMode)
    {
        speedMode_ = newMode;
        pid_.SetMode(MANUAL);
        output_ = speed;    // set PID integral sum to predefined value
        input_ = lastError; // set last error to predefined value
        pid_.SetMode(AUTOMATIC);
        pid_.SetTunings(Kp, Ki, Kd);
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
        ReInitializePID(FAST3, 0, 0, KpFast3_, 0, 0);                         // very fast movement
    else if(diff > diff2)
        ReInitializePID(FAST2, 0, 0, KpFast2_, 0, 0);                         // fast movement
    else
        ReInitializePID(REGULAR, speed_ * options_.scopeToMotor_, diff, Kp_, Ki_, Kd_);   // regular speed
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

        if(speedMode_ != REGULAR || ts - lastAdjustPID_ > options_.adjustPidTmo_)
            AdjustPID(setpoint_ - input_, ts);

        if(pid_.Compute())
        {
            if(speedMode_ != REGULAR)
                motor_->SetSpeed(output_);
            else
            {
                double curr = motor_->GetSpeed();
                motor_->SetSpeed(curr + (output_ - curr)*speedSmooth_);
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
    input_ = 0;
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
