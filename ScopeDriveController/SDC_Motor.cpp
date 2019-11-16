/*
 * SDC_Motor.cpp
 *
 * Created: 1/27/2019 10:25:32 AM
 *  Author: Alexey
 */ 

#include <Arduino.h>

#include "SDC_Motor.h"

SDC_Motor::SDC_Motor(const Options &options, uint8_t dirPin, uint8_t speedPin, volatile long *encPos)
    :   maxSpeed_(options.maxSpeed_), dirPin_(dirPin), speedPin_(speedPin), mt_(NULL), encPos_(encPos), running_(false),
        pid_(&input_, &output_, &setpoint_, options.Kp_, options.Ki_, 0.0, P_ON_E, DIRECT)
{
    pid_.SetOutputLimits(-255,255);
    pid_.SetSampleTime(100);
}

void SDC_Motor::Setup()
{
    pinMode(dirPin_, OUTPUT);
    digitalWrite(dirPin_, LOW);
    pinMode(speedPin_, OUTPUT);
    analogWrite(speedPin_, 0);
}

bool SDC_Motor::Run()
{
    if(!running_)
        return true;
    if(mt_ && !mt_->CanMove(this))
    {
        SDC_MotionType *mt = mt_;
        DoStop();
        mt->MotorStopped(this, false);
        return true;
    }

    long uposCurr, tsCurr;
    DoGetPos(&uposCurr, &tsCurr);

    setpoint_ = upos_ + speed_*(tsCurr - ts_);
    input_ = uposCurr;
    pid_.Compute();

    int sp;
    uint8_t direction;
    if(output_ > 0)
    {
        sp = int(output_ + 0.5);
        direction = HIGH;
    }
    else
    {
        sp = int(-output_ + 0.5);
        direction = LOW;
    }
    if(sp > 255)
        sp = 255;
    else if (sp < 0)
        sp = 0;
    digitalWrite(dirPin_, direction);
    analogWrite(speedPin_, sp);

    return true;
}

bool SDC_Motor::GetPhysicalPos(Ref *ref, long *setpoint, long *dbgParam) const
{
    if(ref)
        *ref = Ref(*encPos_, millis());
    if(setpoint)
        *setpoint = (long)setpoint_;
    if(dbgParam)
        *dbgParam = output_;
    return running_;
}

bool SDC_Motor::GetLogicalPos(Ref *ref) const
{
    if(ref)
    {
        long tsCurr = millis();
        *ref = Ref(running_ ? (upos_ + speed_*(tsCurr - ts_)) : *encPos_, tsCurr);
    }
    return running_;
}

bool SDC_Motor::GetDeviation(Ref *ref) const
{
    if(ref)
    {
        long tsCurr = millis();
        *ref = Ref(running_ ? (upos_ + speed_*(tsCurr - ts_) - *encPos_) : 0, tsCurr);
    }
    return running_;
}

bool SDC_Motor::Start (double speed, SDC_MotionType *mt, Ref *ref)
{
    if(running_)
        return false;

    if(mt && !mt->CanMove(this))
        return false;

    running_ = true;
    mt_ = mt;
    if(mt_)
        mt_->MotorStarted(this);

    DoGetPos(&upos_, &ts_);
    if(ref)
        *ref = Ref(upos_, ts_);
    speed_ = speed;

    setpoint_ = input_ = upos_;
    output_ = 0;
    pid_.SetMode(AUTOMATIC);
    return true;
}

bool SDC_Motor::SetSpeed(double speed, Ref *ref)
{
    if(!running_)
        return false;

    if(speed != speed_)
    {
        long uposCurr, tsCurr;
        DoGetPos(&uposCurr, &tsCurr);

        //upos_ = uposCurr;             // <-- Incorrect!
        upos_ += speed_*(tsCurr - ts_); // <-- Correct! To keep the PID state, we must use current LOGICAL position as the next reference point.
        ts_ = tsCurr;

        speed_ = speed;
        if(ref)
            *ref = Ref(upos_, ts_);
    }
    return true;
}

bool SDC_Motor::SetNextPos(long upos, long ts, bool reset, Ref *ref)
{
    if(!running_)
        return false;

    long uposCurr, tsCurr;
    DoGetPos(&uposCurr, &tsCurr);
    if((ts > tsCurr ? ts - tsCurr : tsCurr - ts) > 10)   // ignore if timestamps are closer than 10 ms, due to bad accuracy
    {
        //upos_ = uposCurr;             // <-- Incorrect!
        upos_ += speed_*(tsCurr - ts_); // <-- Correct! To keep the PID state, we must use current LOGICAL position as the next reference point.
        ts_ = tsCurr;

        speed_ = double(upos - uposCurr)/double(ts - tsCurr);
        if(ref)
            *ref = Ref(uposCurr, tsCurr);
        return true;
    }
    return false;
}

void SDC_Motor::Stop()
{
    SDC_MotionType *mt = mt_;
    DoStop();
    if(mt)
        mt->MotorStopped(this, true);
}

void SDC_Motor::DoStop()
{
    running_ = false;
    mt_ = NULL;
    DoGetPos(&upos_, &ts_);
    speed_ = 0;
    pid_.SetMode(MANUAL);
    analogWrite(speedPin_, 0);
}

void SDC_Motor::DoGetPos(long *upos, long *ts)
{
    *ts = millis();
    *upos = *encPos_;
}
