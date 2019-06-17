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
#include "SDC_EncPositionAdapter.h"
#include "SDC_Sound.h"
#include "SDC_Storage.h"

///////////////////////////////////////////////////////////////////////////////////////
const long RESOLUTION = 1000*4;

///////////////////////////////////////////////////////////////////////////////////////
static void printHex(long val)
{
    byte buf[2];
    buf[0] = val - (buf[1] = val / 256) * 256;
    Serial.write(buf, 2);
}

static void printHex2(unsigned long v)
{
    byte buf[4];
    buf[0] = v;
    buf[1] = v >> 8;
    buf[2] = v >> 16;
    buf[3] = v >> 24;
    Serial.write(buf, 4);
}

class IntlkMotionType : public SDC_MotionType
{
public:
    virtual bool    CanMove(const SDC_MotorItf*) const              {return digitalRead(SWITCH_IPIN) == 0;}
    virtual void    MotorStarted(SDC_MotorItf*)                     {}
    virtual void    MotorStopped(SDC_MotorItf*, bool byStopCommand) {if(!byStopCommand) MakeSound(400);}
};
IntlkMotionType intlk;

//
// For an ideal motor, the best Ki (integral coefficient in PID) choice, to avoid oscillations:
//
//  Ki <= RESOLUTION*RPM*(Kp^2)/(60*255*4)
//
// where
//  RESOLUTION  - encoder resolution per full cycle
//  RPM         - motor speed (rotations per minute)
//  Kp          - proportional coefficient in PID
//
// For example, if RESOLUTION = 4000, RPM = 65rpm, Kp = 0.5, Ki <= 4000*65*0.25/61200 = 1.06
//
// As a real motor is not ideal and has some threshold voltage to start rotation, it may be better to make the Ki smaller than the theoretical value.
// The system stabilizes slower but oscillates less on slow speeds. (Is oscillation on slow speed really a problem?)
//
SDC_Motor motorALT(SDC_Motor::Options(50*RESOLUTION/60000, 0.5, 0.4), DIR1_OPIN, PWMA_OPIN, SDC_GetMotorAltEncoderPositionPtr());   // 65rpm
SDC_Motor motorAZM(SDC_Motor::Options(50*RESOLUTION/60000, 0.5, 0.4), DIR2_OPIN, PWMB_OPIN, SDC_GetMotorAzmEncoderPositionPtr());   // 65rpm

SDC_MotorAdapter adapterALT(SDC_MotorAdapter::Options(224.9, 0.3, 0.8), SDC_GetAltEncoderPositionPtr(), SDC_GetMotorAltEncoderPositionPtr(), &motorALT);
SDC_MotorAdapter adapterAZM(SDC_MotorAdapter::Options(181.0, 0.3, 0.8), SDC_GetAzmEncoderPositionPtr(), SDC_GetMotorAzmEncoderPositionPtr(), &motorAZM);

uint8_t uSessionId;

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

    SoundSetup();
    SDC_EncodersSetup();
    motorALT.Setup();
    motorAZM.Setup();
    adapterALT.Setup();
    adapterAZM.Setup();

    digitalWrite(ENABLE_OPIN, HIGH);

    SDC_ReadSessionId(&uSessionId);
    SDC_WriteSessionId(++uSessionId);

    Serial.begin(115200);
}

// capabilities that we support
#define CONNCAPS_ALTAZM     1   // Digital Setting Circles
#define CONNCAPS_EQU        2   // Equatorial Platform - dummy emulation only
#define CONNCAPS_GOTO       8   // GoTo commands

static void ReportCapabilities()
{
    // hardcoded
    const uint8_t capabilities = (CONNCAPS_ALTAZM|CONNCAPS_EQU|CONNCAPS_GOTO);
    Serial.write(capabilities);
}


#define A_ALT   0       // command for alt adapter
#define A_AZM   1       // command for azm adapter
#define M_ALT   2       // command for alt motor (debug only)
#define M_AZM   3       // command for azm motor (debug only)

static SDC_MotorItf* GetMotor(byte b)
{
    switch(b)
    {
    default:
    case A_ALT: return &adapterALT;
    case A_AZM: return &adapterAZM;
    case M_ALT: return &motorALT;
    case M_AZM: return &motorAZM;
    }
}

static void StartMotor(byte buf[], int, int)
{
    byte *p = buf;
    SDC_MotorItf *motor = GetMotor(*p++);

    // speed: units/day
    long speed = long((uint32_t(p[3]) << 24) + (uint32_t(p[2]) << 16) + (uint32_t(p[1]) << 8) + uint32_t(p[0]));

    SDC_MotorItf::Ref ref;
    motor->Start(double(speed)/(24.0*60.0*60000.), &intlk, &ref);     // convert speed -> units/ms
    printHex2(ref.upos_);
    printHex2(ref.ts_);
}

static void SetMotorSpeed(byte buf[], int, int)
{
    byte *p = buf;
    SDC_MotorItf *motor = GetMotor(*p++);

    // speed: units/day
    long speed = long((uint32_t(p[3]) << 24) + (uint32_t(p[2]) << 16) + (uint32_t(p[1]) << 8) + uint32_t(p[0]));

    SDC_MotorItf::Ref ref;
    motor->SetSpeed(double(speed)/(24.0*60.0*60000.), &ref);     // convert speed -> units/ms
    printHex2(ref.upos_);
    printHex2(ref.ts_);
}

static void NextMotorPosition(byte buf[], int, int)
{
    byte *p = buf;
    SDC_MotorItf *motor = GetMotor(*p++);

    long upos = long((uint32_t(p[3]) << 24) + (uint32_t(p[2]) << 16) + (uint32_t(p[1]) << 8) + uint32_t(p[0]));
    long ts   = long((uint32_t(p[7]) << 24) + (uint32_t(p[6]) << 16) + (uint32_t(p[5]) << 8) + uint32_t(p[4]));

    SDC_MotorItf::Ref ref;
    motor->SetNextPos(upos, ts, false, &ref);
    printHex2(ref.upos_);
    printHex2(ref.ts_);
}

static void StopMotor(byte buf[], int, int)
{
    GetMotor(buf[0])->Stop();
    Serial.print("r");
}

static void PollMotor(byte buf[], int, int)
{
    SDC_MotorItf::Ref ref;
    long setpoint;
    long dbg;
    byte running = GetMotor(buf[0])->GetPos(&ref, &setpoint, &dbg) ? 1 : 0;
    printHex2(ref.upos_);
    printHex2(ref.ts_);
    printHex2(setpoint);
    printHex2(dbg);
    switch(buf[0])
    {
    default:
    case A_ALT: case M_ALT:
        printHex2(*SDC_GetMotorAltEncoderPositionPtr());
        printHex2(*SDC_GetAltEncoderPositionPtr());
        break;
    case A_AZM: case M_AZM:
        printHex2(*SDC_GetMotorAzmEncoderPositionPtr());
        printHex2(*SDC_GetAzmEncoderPositionPtr());
        break;
    }
    Serial.write(&running, 1);
}

///////////////////////////////////////////////////////////////////////////////////////
#define SERIAL_BUF_SZ 10
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
    // GENERAL COMMANDS

    case 'c':
        // report capabilities
        ReportCapabilities();
        break;

    case 's':
        // report capabilities and session id
        ReportCapabilities();
        Serial.write(uSessionId);
        break;


    // ALT/AZM (DIGITAL SETTING CURCLES) COMMANDS

    case 'p':
        // Dave Ek's format: report the number of encoder errors and reset the counter to zero
        Serial.write(uint8_t(0));   // no errors
        break;

    case 'h':
        // Dave Ek's format: report encoder resolutions
        printHex(SDC_GetAltEncoderResolution());
        printHex(SDC_GetAzmEncoderResolution());
        break;

    case 'y':
        // Dave Ek's format: report encoder positions
        printHex(SDC_GetAltEncoderPosition());
        printHex(SDC_GetAzmEncoderPosition());
        break;


    // EQUATORIAL PLATFORM COMMANDS - EMULATION ONLY

    case 'q':
        // report the equatorial angle resolution
        printHex(0x8000);
        break;

    case 'e':
        // report the equatorial angle
        printHex(0);
        break;

    case '1':   // star tracking
    case '2':   // moon tracking
    case '3':   // sun tracking
    case '8':   // fast move to center
    case '#':   // fast forward
    case '0':   // stop
    case '*':   // fast backward
        // don't do anything
        Serial.print("r");
        break;


    // GOTO COMMANDS

    case 'S':   // start and set speed
        SetSerialBuf(5, StartMotor);
        break;

    case 'V':   // set speed
        SetSerialBuf(5, SetMotorSpeed);
        break;

    case 'N':   // next position
        SetSerialBuf(9, NextMotorPosition);
        break;

    case 'T':   // stop
        SetSerialBuf(1, StopMotor);
        break;

    case 'P':   // poll
        SetSerialBuf(1, PollMotor);
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
    bool safe1 = adapterALT.Run();
    bool safe2 = motorALT.Run();
    bool safe3 = adapterAZM.Run();
    bool safe4 = motorAZM.Run();
    safe1 = safe1 && safe2 && safe3 && safe4;

    // serial
    static int safeSkip = 0;
    if(Serial.available() && (safe1 || ++safeSkip > 10))
    {
        safeSkip = 0;
        char inchar = Serial.read();
        ProcessSerialCommand(inchar);
    }
}

