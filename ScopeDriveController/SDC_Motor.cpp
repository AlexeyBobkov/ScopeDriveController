/*
 * SDC_Motor.cpp
 *
 * Created: 1/27/2019 10:25:32 AM
 *  Author: Alexey
 */ 

#include <Arduino.h>
#include <limits.h>

#include "SDC_Motor.h"

//#define GET_TIME            millis()
//#define CONV_TIME(x)        (x)
//#define UCONV_TIME(x)       (x)
//#define CONV_TIME_VAR(x)

#define GET_TIME            micros()
#define CONV_TIME(x)        ((x) * 1000)
#define UCONV_TIME(x)       ((x) / 1000)
#define CONV_TIME_VAR(x)    x *= 1000;


#ifdef USE_EXPONENTIAL_APPROXIMATION

///////////////////////////////////////////////////////////////////////////////////////
SDC_Motor::PWMApproximation::PWMApproximation(const PWMProfile &lp, const PWMProfile &hp) : loProfile_(lp)
{
    double valDiff = hp.value_ - lp.value_;
    magnitudeCoeff_ = pow(double(hp.magnitude_)/double(lp.magnitude_),  1/valDiff);
    periodCoeff_    = pow(double(hp.period_)/double(lp.period_),        1/valDiff);
}

///////////////////////////////////////////////////////////////////////////////////////
void SDC_Motor::PWMApproximation::MakeApproximation(int absSp, uint8_t *magnitude, double *period)
{
    double power = absSp - loProfile_.value_;
    *magnitude   = loProfile_.magnitude_*pow(magnitudeCoeff_, power) + 0.5;
    *period      = loProfile_.period_*pow(periodCoeff_, power);
}

#else   // linear approximation

///////////////////////////////////////////////////////////////////////////////////////
SDC_Motor::PWMApproximation::PWMApproximation(const PWMProfile &lp, const PWMProfile &hp) : loProfile_(lp)
{
    double valDiff = hp.value_ - lp.value_;
    magnitudeCoeff_ = (hp.magnitude_ - lp.magnitude_)/valDiff;
    periodCoeff_    = (hp.period_ - lp.period_)/valDiff;
}

///////////////////////////////////////////////////////////////////////////////////////
void SDC_Motor::PWMApproximation::MakeApproximation(int absSp, uint8_t *magnitude, double *period)
{
    double diff = absSp - loProfile_.value_;
    *magnitude   = loProfile_.magnitude_ + magnitudeCoeff_ * diff + 0.5;
    *period      = loProfile_.period_ + periodCoeff_ * diff;
}

#endif


///////////////////////////////////////////////////////////////////////////////////////
SDC_Motor::SDC_Motor(const Options &options, uint8_t dirPin, uint8_t speedPin, volatile long *encPos)
    :   maxSpeed_(options.maxSpeed_), dirPin_(dirPin), speedPin_(speedPin), encPos_(encPos),
        loProfile_(options.loProfile_), hiProfile_(options.hiProfile_), pwmApprox_(options.loProfile_, options.hiProfile_), mt_(NULL), running_(false),
        pid_(&input_, &output_, &setpoint_, options.Kp_, options.Ki_, options.Kd_, P_ON_E, DIRECT)
#ifdef TEST_SLOW_PWM
        , testPWMVal_(0)
#endif
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

void SDC_Motor::SetVal(int val)
{
    if(val > 0)
    {
        digitalWrite(dirPin_, HIGH);
        analogWrite(speedPin_, val);
    }
    else
    {
        digitalWrite(dirPin_, LOW);
        analogWrite(speedPin_, -val);
    }
}

void SDC_Motor::SetVal(uint8_t val, bool positive)
{
    digitalWrite(dirPin_, positive ? HIGH : LOW);
    analogWrite(speedPin_, val);
}

bool SDC_Motor::Run()
{
#ifdef TEST_SLOW_PWM
    if(testPWMVal_ != 0)
    {
        if(mt_ && !mt_->CanMove(this))
        {
            SDC_MotionType *mt = mt_;
            DoStop();
            mt->MotorStopped(this, false);
            return true;
        }

        long now;
        switch(pwmState_)
        {
        default:
        case PWM_CONST: break;  // already set
        case PWM_HIGH:
            if((unsigned long)((now = GET_TIME) - tsPWMStart_ - hiPeriod_) < ULONG_MAX/2)
            {
                // switch to low
                analogWrite(speedPin_, 0);
                pwmState_ = PWM_LOW;
                tsPWMStart_ = now;
            }
            break;
        case PWM_LOW:
            if((unsigned long)((now = GET_TIME) - tsPWMStart_ - loPeriod_) < ULONG_MAX/2)
            {
                // switch to high
                SetVal(testPWMVal_);
                pwmState_ = PWM_HIGH;
                tsPWMStart_ = now;
            }
            break;
        }
        return true;
    }
#endif

    if(!running_)
        return true;
    if(mt_ && !mt_->CanMove(this))
    {
        SDC_MotionType *mt = mt_;
        DoStop();
        mt->MotorStopped(this, false);
        return true;
    }

    setpoint_ = upos_ + speed_*(millis() - ts_);
    input_ = *encPos_;
    if(pid_.Compute())
    {
#ifdef USE_SLOW_PWM
        int sp = round(output_);
        int absSp = (sp >= 0) ? sp : -sp;
        if(absSp >= hiProfile_.magnitude_)
        {
            pwmState_ = PWM_CONST;
            SetVal(sp);
            pid_.SetSampleTime(100);
            return true;
        }
        else if(absSp < loProfile_.value_)
        {
            pwmState_ = PWM_CONST;
            analogWrite(speedPin_, 0);
            pid_.SetSampleTime(100);
            return true;
        }

        uint8_t magnitude;
        double period;
        if(absSp < hiProfile_.value_)
            pwmApprox_.MakeApproximation(absSp, &magnitude, &period);
        else
        {
            magnitude = hiProfile_.magnitude_;
            period = hiProfile_.period_;
        }

        // set to high
        hiPeriod_ = (CONV_TIME(period) * absSp / magnitude) + 0.5;
        tsPWMStart_ = GET_TIME;
        pwmState_ = PWM_HIGH;
        SetVal(magnitude, sp > 0);
        pid_.SetSampleTime(period + 0.5);
#else
        int sp;
        if(output_ > 0)
        {
            sp = int(output_ + 0.5);
            digitalWrite(dirPin_, HIGH);
        }
        else
        {
            sp = int(-output_ + 0.5);
            digitalWrite(dirPin_, LOW);
        }
        analogWrite(speedPin_, sp > 255 ? 255 : sp);
#endif
    }
#ifdef USE_SLOW_PWM
    else if(pwmState_ == PWM_HIGH && (unsigned long)(GET_TIME - tsPWMStart_ - hiPeriod_) < ULONG_MAX/2)
    {
        // switch to low
        pwmState_ = PWM_LOW;
        analogWrite(speedPin_, 0);
    }
#endif
    return true;
}

bool SDC_Motor::GetPhysicalPos(Ref *ref, double *setpoint, double *dbgParam) const
{
    if(ref)
        *ref = Ref(*encPos_, millis());
    if(setpoint)
        *setpoint = setpoint_;
    if(dbgParam)
        *dbgParam = output_;
#ifndef TEST_SLOW_PWM
    return running_;
#else
    return running_ || testPWMVal_ != 0;
#endif
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
#ifndef TEST_SLOW_PWM
    if(running_)
        return false;
#else
    if(running_ || testPWMVal_ != 0)
        return false;
#endif

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
        double uposCurr;
        long tsCurr;
        DoGetPos(&uposCurr, &tsCurr);

        upos_ += speed_*(tsCurr - ts_); // To keep the PID state, we must use current LOGICAL position as the next reference point.
        ts_ = tsCurr;

        speed_ = speed;
        if(ref)
            *ref = Ref(upos_, ts_);
    }
    return true;
}

bool SDC_Motor::SetNextPos(double upos, long ts, bool reset, Ref *ref)
{
    if(!running_)
        return false;

    double uposCurr;
    long tsCurr;
    DoGetPos(&uposCurr, &tsCurr);
    if((ts > tsCurr ? ts - tsCurr : tsCurr - ts) > 10)   // ignore if timestamps are closer than 10 ms, due to bad accuracy
    {
        upos_ += speed_*(tsCurr - ts_); // To keep the PID state, we must use current LOGICAL position as the next reference point.
        ts_ = tsCurr;

        speed_ = (upos - uposCurr)/(ts - tsCurr);
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

#ifdef TEST_SLOW_PWM
bool SDC_Motor::StartSlowPWM(int val, long period, double dutyCycle, SDC_MotionType *mt, Ref *ref)
{
    if(running_)
        return false;

    if(val)
    {
        if(testPWMVal_ && mt_)
            mt = mt_;

        if(mt && !mt->CanMove(this))
        {
            if(testPWMVal_)
            {
                DoStop();
                mt->MotorStopped(this, true);
            }
            return false;
        }
        mt_ = mt;

        double uposCurr;
        long tsCurr;
        DoGetPos(&uposCurr, &tsCurr);
        if(ref)
            *ref = Ref(uposCurr, tsCurr);

        if(dutyCycle < 0)
            dutyCycle = 0;
        else if(dutyCycle > 1)
            dutyCycle = 1;

        CONV_TIME_VAR(period)
        hiPeriod_ = round(period*dutyCycle);
        loPeriod_ = period - hiPeriod_;
        if(hiPeriod_ == 0)
            testPWMVal_ = 0;
        else
        {
            if(loPeriod_ == 0)
                pwmState_ = PWM_CONST;
            else
            {
                pwmState_ = PWM_HIGH;
                tsPWMStart_ = GET_TIME;
            }

            if(!testPWMVal_ && mt_)
                mt_->MotorStarted(this);
            testPWMVal_ = val;
            SetVal(testPWMVal_);
            setpoint_ = 0;
            return true;
        }
    }

    mt = mt_;
    DoStop();
    if(mt)
        mt->MotorStopped(this, true);
    return true;
}
#endif

void SDC_Motor::DoStop()
{
    running_ = false;
    mt_ = NULL;
    DoGetPos(&upos_, &ts_);
    speed_ = 0;
#ifdef TEST_SLOW_PWM
    testPWMVal_ = 0;
#endif
    pid_.SetMode(MANUAL);
    analogWrite(speedPin_, 0);
}

void SDC_Motor::DoGetPos(double *upos, long *ts)
{
    *ts = millis();
    *upos = *encPos_;
}
