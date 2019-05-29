/*
 * SDC_Motor.cpp
 *
 * Created: 1/27/2019 10:25:32 AM
 *  Author: Alexey
 */ 

#include <Arduino.h>

#include "SDC_Configuration.h"
#include "EP_Encoders.h"
#include "SDC_Motor.h"

const long DBG_ENCODER_RATIO = 2;
const long MOTOR_RESOLUTION = RESOLUTION/DBG_ENCODER_RATIO;

SDC_Motor::SDC_Motor(double rpm, uint8_t dirPin, uint8_t speedPin, MotionType *mt, volatile long *encPos)
    :   max_speed_(rpm*MOTOR_RESOLUTION/60000), dirPin_(dirPin), speedPin_(speedPin), mt_(mt), encPos_(encPos), running_(false),
        //pid_(&input_, &output_, &setpoint_, 5, 2, 0, DIRECT)      // 10rpm
        //pid_(&input_, &output_, &setpoint_, 2.5, 1, 0, DIRECT)    // 10rpm
        pid_(&input_, &output_, &setpoint_, 1, 0.5, 0, DIRECT)   // 65rpm
        //pid_(&input_, &output_, &setpoint_, 1, 2, 0, DIRECT)   // 60rpm
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
    //if(uposCurr == upos_ && tsCurr - ts_ < 1000)
    //    return true;

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
    *upos = *encPos_/DBG_ENCODER_RATIO;
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
    *upos = *encPos_/DBG_ENCODER_RATIO;
}
