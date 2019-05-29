/*
 * ScopeDriveController.cpp
 *
 * Created: 1/16/2019 9:31:16 PM
 * Author : Alexey
 */ 

#include <Arduino.h>

#include "SDC_Configuration.h"
#include "SDC_Encoders.h"
#include "SDC_Motor.h"
#include "SDC_Sound.h"

///////////////////////////////////////////////////////////////////////////////////////
/*
static void printHex(long val)
{
    byte buf[2];
    buf[0] = val - (buf[1] = val / 256) * 256;
    Serial.write(buf, 2);
}
*/

static void printHex2(unsigned long v)
{
    byte buf[4];
    buf[0] = v;
    buf[1] = v >> 8;
    buf[2] = v >> 16;
    buf[3] = v >> 24;
    Serial.write(buf, 4);
}

class IntlkMotionType : public SDC_Motor::MotionType
{
public:
    virtual bool    CanMove(const SDC_Motor*) const {return digitalRead(SWITCH_IPIN) == 0;}
    virtual void    MotorStarted(SDC_Motor*)        {}
    virtual void    MotorStopped(SDC_Motor*)        {MakeSound(400);}
};
IntlkMotionType intlk;

SDC_Motor motorALT(64, DIR1_OPIN, PWMA_OPIN, &intlk, SDC_GetMotorAltEncoderPositionPtr());
SDC_Motor motorAZM(64, DIR2_OPIN, PWMB_OPIN, &intlk, SDC_GetMotorAzmEncoderPositionPtr());

void setup()
{
    //pinMode(SOUND_OPIN, OUTPUT);

    pinMode(SWITCH_IPIN, INPUT_PULLUP);
    pinMode(ENABLE_OPIN, OUTPUT);
    //pinMode(DIR1_OPIN, OUTPUT);
    //pinMode(DIR2_OPIN, OUTPUT);
    //pinMode(PWMB_OPIN, OUTPUT);
    //pinMode(PWMA_OPIN, OUTPUT);

    // configure PWM
    ALT_AZM_TCCRB = (ALT_AZM_TCCRB & 0b11111000) | ALT_AZM_PRESCALER;

    SDC_EncodersSetup();

    motorALT.Setup();
    motorAZM.Setup();
    digitalWrite(ENABLE_OPIN, HIGH);

    SoundSetup();

    Serial.begin(115200);
}

static void SetSpeed(byte buf[], int, int)
{
    long speed = long((uint32_t(buf[3]) << 24) + (uint32_t(buf[2]) << 16) + (uint32_t(buf[1]) << 8) + uint32_t(buf[0]));

    long upos, ts;
    motorAZM.Start(double(speed)/(24.0*60.0*60000.), &upos, &ts);
    printHex2(upos);
    printHex2(ts);
}

static void NextPosition(byte buf[], int, int)
{
    long upos = long((uint32_t(buf[3]) << 24) + (uint32_t(buf[2]) << 16) + (uint32_t(buf[1]) << 8) + uint32_t(buf[0]));
    long ts   = long((uint32_t(buf[7]) << 24) + (uint32_t(buf[6]) << 16) + (uint32_t(buf[5]) << 8) + uint32_t(buf[4]));

    motorAZM.SetNextPos(upos, ts);
    Serial.print("r");
}

///////////////////////////////////////////////////////////////////////////////////////
#define SERIAL_BUF_SZ 8
static byte serialBuf[SERIAL_BUF_SZ];
static int serialBufCurr = 0;
static int serialBufWait = 0;
typedef void (*SERIAL_FN)(byte buf[], int curr, int wait);
static SERIAL_FN serialFn;
static void SetSerialBuf(int bufWait, SERIAL_FN fn)
{
    serialBufWait = bufWait;
    serialBufCurr = 0;
    serialFn = fn;
}

///////////////////////////////////////////////////////////////////////////////////////
static void ProcessSerialCommand(char inchar)
{
    if(serialBufWait)
    {
        serialBuf[serialBufCurr++] = (byte)inchar;
        if(serialBufCurr >= serialBufWait)
        {
            byte buf[SERIAL_BUF_SZ];
            memcpy(buf, serialBuf, sizeof(serialBuf));
            int bufCurr = serialBufCurr;
            int bufWait = serialBufWait;
            serialBufCurr = serialBufWait = 0;
            serialFn(buf, bufCurr, bufWait);
        }
        return;
    }

    switch(inchar)
    {
    case 'S':   // start and set speed
        SetSerialBuf(4, SetSpeed);
        break;

    case 'N':   // next position
        SetSerialBuf(8, NextPosition);
        break;

    case 'T':   // stop
        motorAZM.Stop();
        Serial.print("r");
        break;

    case 'P':   // poll
        {
            long upos, ts, setpoint;
            byte running = motorAZM.GetPos(&upos, &ts, &setpoint) ? 1 : 0;
            printHex2(upos);
            printHex2(ts);
            printHex2(setpoint);
            Serial.write(&running, 1);
        }
        break;

    default:
    break;
    }

}

void loop()
{
    SoundRun();

    // interlock
    digitalWrite(ENABLE_OPIN, digitalRead(SWITCH_IPIN) ? LOW : HIGH);

    // motor
    bool safe = motorALT.Run();
    bool safe2 = motorAZM.Run();
    safe = safe && safe2;

    // serial
    static int safeSkip = 0;
    if(Serial.available() && (safe || ++safeSkip > 10))
    {
        safeSkip = 0;
        char inchar = Serial.read();
        ProcessSerialCommand(inchar);
    }
}

