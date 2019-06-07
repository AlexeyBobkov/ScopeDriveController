/*
 * SDC_Motor.cpp
 *
 * Created: 1/27/2019 10:25:32 AM
 *  Author: Alexey
 */ 

#include <Arduino.h>

#include "SDC_Motor.h"

SDC_Motor::SDC_Motor(const Options &options, uint8_t dirPin, uint8_t speedPin, volatile long *encPos)
    :   max_speed_(options.max_speed_), dirPin_(dirPin), speedPin_(speedPin), mt_(NULL), encPos_(encPos), running_(false),
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
    long uposCurr, tsCurr;
    DoGetPos(&uposCurr, &tsCurr);

    setpoint_ = upos_ + speed_*(tsCurr - ts_);
    input_ = uposCurr;
    pid_.Compute();

    if(!running_)
        return true;
    if(mt_ && !mt_->CanMove(this))
    {
        MotionType *mt = mt_;
        Stop();
        mt->MotorStopped(this);
        return true;
    }

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

bool SDC_Motor::GetPos(Ref *ref, long *setpoint) const
{
    if(ref)
        *ref = Ref(*encPos_, millis());
    if(setpoint)
        *setpoint = (long)setpoint_;
    return running_;
}

bool SDC_Motor::Start (double speed, MotionType *mt, Ref *ref)
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
    pid_.SetMode(AUTOMATIC);

    /*
    // start motor
    int sp;
    uint8_t direction;
    if(speed_ > 0)
    {
        sp = int(speed_*255/max_speed_ + 0.5);
        direction = HIGH;
    }
    else
    {
        sp = int(-speed_*255/max_speed_ + 0.5);
        direction = LOW;
    }
    if(sp > 255)
        sp = 255;
    else if (sp < 0)
        sp = 0;
    digitalWrite(dirPin_, direction);
    analogWrite(speedPin_, sp);
    */

    return true;
}

bool SDC_Motor::SetSpeed(double speed, Ref *ref)
{
    if(!running_)
        return false;

    if(speed != speed_)
    {
        DoGetPos(&upos_, &ts_);
        speed_ = speed;
        if(ref)
            *ref = Ref(upos_, ts_);
    }
    return true;
}

bool SDC_Motor::SetNextPos(long upos, long ts, Ref *ref)
{
    if(!running_)
        return false;

    long uposCurr, tsCurr;
    DoGetPos(&uposCurr, &tsCurr);
    if((ts > tsCurr ? ts - tsCurr : tsCurr - ts) > 10)   // ignore if timestamps are closer than 10 ms, due to bad accuracy
    {
        ts_ = tsCurr;
        upos_ = uposCurr;
        speed_ = double(upos - uposCurr)/double(ts - tsCurr);
        if(ref)
            *ref = Ref(uposCurr, tsCurr);
        return true;
    }
    return false;
}

void SDC_Motor::Stop()
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
