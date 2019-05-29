/*
 * SDC_Motor.cpp
 *
 * Created: 1/27/2019 10:25:32 AM
 *  Author: Alexey
 */ 

#include <Arduino.h>

#include "SDC_Configuration.h"
#include "SDC_Motor.h"

SDC_Motor::SDC_Motor(double max_speed, double Kp, double Ki, uint8_t dirPin, uint8_t speedPin, MotionType *mt, volatile long *encPos)
    :   max_speed_(max_speed), dirPin_(dirPin), speedPin_(speedPin), mt_(mt), encPos_(encPos), running_(false),
        pid_(&input_, &output_, &setpoint_, Kp, Ki, 0, DIRECT)
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
        Stop();
        mt_->MotorStopped(this);
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

bool SDC_Motor::Start (double speed, long *upos, long *ts)
{
    if(running_)
        return false;

    if(mt_ && !mt_->CanMove(this))
        return false;

    running_ = true;
    if(mt_)
        mt_->MotorStarted(this);

    DoGetPos(&upos_, &ts_);
    *upos = upos_;
    *ts = ts_;
    speed_ = speed;

    setpoint_ = input_ = upos_;
    pid_.SetMode(AUTOMATIC);

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

    return true;
}

bool SDC_Motor::GetPos(long *upos, long *ts, long *setpoint)
{
    *ts = millis();
    *upos = *encPos_;
    *setpoint = (long)setpoint_;
    return running_;
}

bool SDC_Motor::SetNextPos(long upos, long ts)
{
    if(!running_ || ts <= ts_)
        return false;

    speed_ = (upos - upos_)/(ts - ts_);
    upos_ = upos;
    ts_ = ts;
    return true;
}

void SDC_Motor::Stop()
{
    running_ = false;
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
