/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

/**********************************************************************************************
 * Alexey Bobkov: the original library is slightly modified.
 * 1) We are stabilizing the motor position, which is increasing all the time. So, we cannot use
 *    last input instead of last error, as it is often done in other scenarios.
 * 2) We always use P_ON_E mode and newer P_ON_M. So, the appropriate code is deleted.
 * 3) Some other small refactoring.
 **********************************************************************************************/


#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_v1.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);               //default output limit corresponds to
                                                //the arduino pwm limits

    SampleTime = 100;                           //default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis() - SampleTime;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
    if(!inAuto)
        return false;
    unsigned long now = millis();
    if(now - lastTime < SampleTime)
        return false;

    /*Compute all the working error variables*/
    double input = *myInput;
    double error = *mySetpoint - input;
    double dError = (error - lastError);

    /*Compute integral sum*/
    outputSum += (ki * error);
    if(outputSum > outMax)
        outputSum = outMax;
    else if(outputSum < outMin)
        outputSum = outMin;

    /*Compute Rest of PID Output*/
    double output = kp * error + outputSum + kd * dError;
    if(output > outMax)
        output = outMax;
    else if(output < outMin)
        output = outMin;
    *myOutput = output;

    /*Remember some variables for next time*/
    lastError = error;
    lastTime = now;
    return true;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd)
{
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;

    orgKp = Kp;
    orgKi = Ki;
    orgKd = Kd;

    double SampleTimeInSec = ((double)SampleTime)/1000;
    if(controllerDirection == REVERSE)
    {
        kp = -Kp;
        ki = -Ki * SampleTimeInSec;
        kd = -Kd / SampleTimeInSec;
    }
    else
    {
        kp = Kp;
        ki = Ki * SampleTimeInSec;
        kd = Kd / SampleTimeInSec;
    }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
    if (SampleTime != (unsigned long)NewSampleTime && NewSampleTime > 0)
    {
        double NewSampleTimeInSec  = ((double)NewSampleTime)/1000;
        if(controllerDirection == REVERSE)
        {
            ki = -orgKi * NewSampleTimeInSec;
            kd = -orgKd / NewSampleTimeInSec;
        }
        else
        {
            ki = orgKi * NewSampleTimeInSec;
            kd = orgKd / NewSampleTimeInSec;
        }
        SampleTime = (unsigned long)NewSampleTime;
    }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
    if(Min >= Max)
        return;
    outMin = Min;
    outMax = Max;

    if(inAuto)
    {
        if(*myOutput > outMax)
            *myOutput = outMax;
        else if(*myOutput < outMin)
            *myOutput = outMin;

        if(outputSum > outMax)
            outputSum = outMax;
        else if(outputSum < outMin)
            outputSum = outMin;
    }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {
        /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
    outputSum = *myOutput;
    lastError = *myInput;
    if(outputSum > outMax)
        outputSum = outMax;
    else if(outputSum < outMin)
        outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
    if(Direction != controllerDirection)
    {
        kp = -kp;
        ki = -ki;
        kd = -kd;
        controllerDirection = Direction;
    }
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp()     {return orgKp; }
double PID::GetKi()     {return orgKi;}
double PID::GetKd()     {return orgKd;}
int PID::GetMode()      {return inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection() {return controllerDirection;}

