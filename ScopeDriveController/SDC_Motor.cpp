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

const long DBG_ENCODER_RATIO = 5;
const double SPEED_COEFF = PI*2/(double(RESOLUTION/DBG_ENCODER_RATIO)*24*60*60);

void SDC_Motor::Setup()
{
    pinMode(dirPin_, OUTPUT);
    digitalWrite(dirPin_, LOW);
    pinMode(speedPin_, OUTPUT);
    analogWrite(speedPin_, 0);
}

bool SDC_Motor::Run()
{
    /*
    if(!running_)
        return true;

    long uposCurr, long tsCurr;
    GetPos(&uposCurr, &tsCurr);
    if(uposCurr == upos_ && tsCurr - ts_ < 1000)
        return true;
    */

    return true;
}

bool SDC_Motor::Start (long uspeed, long *upos, long *ts)
{
    if(running_)
        return false;
    running_ = true;

    GetPos(&upos_, &ts_);
    *upos = upos_;
    *ts = ts_;

    speed_ = uspeed * SPEED_COEFF;

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

bool SDC_Motor::GetPos(long *upos, long *ts)
{
    *ts = millis();
    *upos = EP_GetAltEncoderPosition()/DBG_ENCODER_RATIO;
    return true;
}

bool SDC_Motor::SetNextPos(long upos, long ts)
{
    return true;
}

void SDC_Motor::Stop()
{
    running_ = false;
    analogWrite(speedPin_, 0);
}

bool SDC_Motor::Move(int speed)
{
    uint8_t direction;
    if(speed > 0)
    direction = HIGH;
    else
    {
        direction = LOW;
        speed = -speed;
    }
    if(speed > 255)
        speed = 255;
    else if (speed < 0)
        speed = 0;

    digitalWrite(dirPin_, direction);
    analogWrite(speedPin_, speed);
    return true;
}
