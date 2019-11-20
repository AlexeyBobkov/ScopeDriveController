/*
 * ScopeDriveController.cpp
 *
 * Created: 1/16/2019 9:31:16 PM
 * Author : Alexey
 */ 

#include <Arduino.h>

#define LOGGING_ON

#include "SDC_Configuration.h"
#include "SDC_Encoders.h"
#include "SDC_Motor.h"
#include "SDC_EncPositionAdapter.h"
#include "SDC_Sound.h"
#include "SDC_Storage.h"

#ifdef LOGGING_ON
#include "RingBuffer.h"
#endif

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
    virtual void    MotorStopped(SDC_MotorItf*, bool byStopCommand) {if(!byStopCommand) MakeSound(50);}
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
SDC_Motor motorALT(SDC_Motor::Options(30*M_RESOLUTION/60000, 1.0, 0.8), DIR1_OPIN, PWMA_OPIN, SDC_GetMotorAltEncoderPositionPtr());   // 30rpm
SDC_Motor motorAZM(SDC_Motor::Options(60*M_RESOLUTION/60000, 0.5, 0.4), DIR2_OPIN, PWMB_OPIN, SDC_GetMotorAzmEncoderPositionPtr());   // 60rpm

SDC_MotorAdapter adapterALT(SDC_MotorAdapter::Options(223.3,    // ratio
                                                      0.04,     // Kp factor (only factor; Ki is calculated)
                                                      1.0,      // Ki factor (only factor; Kp is calculated)
                                                      0.4,      // Kp for fast movement
                                                      0.7,      // Kp for very fast movement
                                                      3.0,      // diff 1 (deviation allowing backward movement and no speed restrictions)
                                                      6.0,      // diff 2 (deviation allowing fast movement)
                                                      15.0),    // diff 3 (deviation allowing very fast movement)
                            SDC_GetAltEncoderResolution(),
                            SDC_GetAltEncoderPositionPtr(),
                            &motorALT);
SDC_MotorAdapter adapterAZM(SDC_MotorAdapter::Options(177.1,    // ratio
                                                      0.04,     // Kp factor
                                                      1.0,      // Ki factor
                                                      0.4,      // Kp for fast movement factor
                                                      0.7,      // Kp for very fast movement
                                                      3.0,      // diff 1
                                                      6.0,      // diff 2
                                                      15.0),    // diff 3
                            SDC_GetAzmEncoderResolution(),
                            SDC_GetAzmEncoderPositionPtr(),
                            &motorAZM);

uint8_t uSessionId;

void setup()
{
    //pinMode(SOUND_OPIN, OUTPUT);

    pinMode(SWITCH_IPIN, INPUT);
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

    MakeSound(100, 500);
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
    printHex2(round(ref.upos_));
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
    printHex2(round(ref.upos_));
    printHex2(ref.ts_);
}

static void NextMotorPosition(byte buf[], int, int)
{
    byte *p = buf;
    SDC_MotorItf *motor = GetMotor(*p++);

    double upos = *((double*)p);
    long ts   = long((uint32_t(p[7]) << 24) + (uint32_t(p[6]) << 16) + (uint32_t(p[5]) << 8) + uint32_t(p[4]));

    SDC_MotorItf::Ref ref;
    motor->SetNextPos(upos, ts, false, &ref);
    printHex2(round(ref.upos_));
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
    double setpoint;
    double dbg;
    byte running = GetMotor(buf[0])->GetPhysicalPos(&ref, &setpoint, &dbg) ? 1 : 0;
    printHex2(round(ref.upos_));
    printHex2(ref.ts_);
    printHex2(round(setpoint));
    printHex2(round(dbg));
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
#ifdef LOGGING_ON

#define CNT_BEFORE_SYNC 50          // count between re-sync
#define LOG_PERIOD      200         // ms
#define MSPEED_SCALE    4000        // motor speed scale
#define ASPEED_SCALE    600000.0    // adapter speed scale

#define LMODE_ALT       0
#define LMODE_AZM       0x8000

#define LMODE_FIRST     LMODE_MPOS
#define LMODE_MPOS      1
#define LMODE_MLOG      2
#define LMODE_MSPD      4
#define LMODE_MERR      8
#define LMODE_APOS      0x10
#define LMODE_ALOG      0x20
#define LMODE_ASPD      0x40
#define LMODE_AERR      0x80
#define LMODE_LAST      (LMODE_AERR<<1)

#define LMODE_OFF           0

struct LoggingData
{
    byte buf[4];

    LoggingData() {}
    LoggingData(byte unused, byte hiAbs1, byte hiAbs2, byte hiAbsTs) // re-synchronize
    {
        buf[0] = ((hiAbs2 & 0x80) >> 5) | ((hiAbs1 & 0x80) >> 6) | (hiAbsTs >> 7);
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0x80;
    }
    LoggingData(long v) // absolute
    {
        buf[0] = v;
        buf[1] = v >> 8;
        buf[2] = v >> 16;
        buf[3] = ((v >> 24) & 0x7f);
    }
    LoggingData(uint16_t rel1, uint16_t rel2) // relative
    {
        buf[0] = rel1;
        buf[1] = rel1 >> 8;
        buf[2] = rel2;
        buf[3] = rel2 >> 8;
    }
};

static uint16_t gLoggingMode    = LMODE_OFF;
static int gCntSinceLastSync    = CNT_BEFORE_SYNC;
static long gAbsPos0, gAbsPos1, gAbsTs;
static RingBuffer<LoggingData, 100> gRingBuf;

static void ReportLogging(byte buf[])
{
    int8_t cnt = int8_t(buf[1]);
    int8_t bufSize = int8_t(gRingBuf.count());
    int8_t toReport = cnt <= bufSize ? cnt : bufSize;

    Serial.write(cnt);                                      // entries requested
    Serial.write(toReport);                                 // entries actually reported
    Serial.write(bufSize - toReport);                       // entries count after this report
    Serial.write(gRingBuf.overflowedCount() > 0 ? 1 : 0);   // current overflowed count status

    // report data
    cnt -= toReport;
    while(--toReport >= 0)
    {
        Serial.write(gRingBuf.front().buf, 4);
        gRingBuf.pop_front();
    }
    // fill rest with 0s
    if(cnt > 0)
    {
        long i = 0;
        do
            Serial.write((byte*)&i, 4);
        while(--cnt > 0);
    }
    gRingBuf.resetOverflowedCnt();
}
static void PositionLogging(byte buf[], int, int)
{
    switch(buf[0])
    {
    case 'w':   // report to host
        ReportLogging(buf);
        break;

    case 'm':   // set mode
        {
            uint16_t newLoggingMode = (uint16_t(buf[2]) << 8) + uint16_t(buf[1]);
            if(gLoggingMode != newLoggingMode)
            {
                gRingBuf.clear();
                gLoggingMode = newLoggingMode;

                // force immediate re-sync
                gCntSinceLastSync = CNT_BEFORE_SYNC;
                gAbsTs = -(CNT_BEFORE_SYNC+1)*LOG_PERIOD;
            }
        }
        MakeSound(20);
        Serial.print("r");
        break;
    }
}

static void LogData()
{
    long ts;
    if(gLoggingMode == LMODE_OFF || (ts = millis()) < gAbsTs + (gCntSinceLastSync+1)*LOG_PERIOD)
        return;

    int i = 0;
    long pos[2] = {0, 0};
    for(uint16_t mask = LMODE_FIRST; mask != LMODE_LAST; mask <<= 1)
    {
        if(gLoggingMode & mask)
        {
            SDC_MotorItf::Ref ref;
            if(gLoggingMode & LMODE_AZM)
            {
                switch(mask)
                {
                default: break;
                case LMODE_MPOS:    pos[i] = *SDC_GetMotorAzmEncoderPositionPtr(); break;
                case LMODE_MLOG:    motorAZM.GetLogicalPos(&ref); pos[i] = (long)ref.upos_; break;
                case LMODE_MSPD:    pos[i] = long(motorAZM.GetSpeed()*MSPEED_SCALE); break;
                case LMODE_MERR:    motorAZM.GetDeviation(&ref); pos[i] = (long)ref.upos_; break;
                case LMODE_APOS:    pos[i] = *SDC_GetAzmEncoderPositionPtr(); break;
                case LMODE_ALOG:    adapterAZM.GetLogicalPos(&ref); pos[i] = (long)ref.upos_; break;
                case LMODE_ASPD:    pos[i] = long(adapterAZM.GetSpeed()*ASPEED_SCALE); break;
                case LMODE_AERR:    adapterAZM.GetDeviation(&ref); pos[i] = (long)ref.upos_; break;
                }
            }
            else
            {
                switch(mask)
                {
                default: break;
                case LMODE_MPOS:    pos[i] = *SDC_GetMotorAltEncoderPositionPtr(); break;
                case LMODE_MLOG:    motorALT.GetLogicalPos(&ref); pos[i] = (long)ref.upos_; break;
                case LMODE_MSPD:    pos[i] = long(motorALT.GetSpeed()*MSPEED_SCALE); break;
                case LMODE_MERR:    motorALT.GetDeviation(&ref); pos[i] = (long)ref.upos_; break;
                case LMODE_APOS:    pos[i] = *SDC_GetAltEncoderPositionPtr(); break;
                case LMODE_ALOG:    adapterALT.GetLogicalPos(&ref); pos[i] = (long)ref.upos_; break;
                case LMODE_ASPD:    pos[i] = long(adapterALT.GetSpeed()*ASPEED_SCALE); break;
                case LMODE_AERR:    adapterALT.GetDeviation(&ref); pos[i] = (long)ref.upos_; break;
                }
            }
            if(++i >= 2)
                break;
        }        
    }

    if(++gCntSinceLastSync <= CNT_BEFORE_SYNC)
        gRingBuf.push_back(LoggingData(uint16_t(pos[0] - gAbsPos0), uint16_t(pos[1] - gAbsPos1)));
    else
    {
        // resynchronize
        gCntSinceLastSync = 0;
        gAbsTs = ts;
        gRingBuf.push_back(LoggingData(0, byte(pos[0]>>24), byte(pos[1]>>24), byte(ts>>24)));
        gRingBuf.push_back(LoggingData(ts));
        gRingBuf.push_back(LoggingData(gAbsPos0 = pos[0]));
        gRingBuf.push_back(LoggingData(gAbsPos1 = pos[1]));
    }
}

#endif
///////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////
#define STATE_ALT_RUNNING   1
#define STATE_AZM_RUNNING   2
#define STATE_SWITCH_ON     4
static uint8_t GetState()
{
    uint8_t state = uint8_t(GetMotor(A_ALT)->IsRunning() ? STATE_ALT_RUNNING : 0) | uint8_t(GetMotor(A_AZM)->IsRunning() ? STATE_AZM_RUNNING : 0);
    if(digitalRead(SWITCH_IPIN) == 0)
        state |= STATE_SWITCH_ON;
    return state;
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

#ifdef LOGGING_ON
    case 'L':   // position logging
        SetSerialBuf(3, PositionLogging);
        break;
#endif

    case 'R':   // report current state
        {
            printHex2(millis());
            printHex2(*SDC_GetAltEncoderPositionPtr());
            printHex2(*SDC_GetAzmEncoderPositionPtr());
            Serial.write(GetState());
        }
        break;

    default:
        break;
    }
}

void loop()
{
    SoundRun();

#ifdef LOGGING_ON
    LogData();
#endif

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

