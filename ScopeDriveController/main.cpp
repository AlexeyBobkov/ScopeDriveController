/*
 * ScopeDriveController.cpp
 *
 * Created: 1/16/2019 9:31:16 PM
 * Author : Alexey
 */ 

#include <Arduino.h>

#include "SDC_Configuration.h"
#include "EP_Encoders.h"
#include "SDC_Motor.h"

//motor A connected between A01 and A02

//int STBY = 10; //standby

//Motor A
//int PWMA = 11; //Speed control
//int AIN1 = 9; //Direction
//int AIN2 = 8; //Direction

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

SDC_Motor motorALT(64, ALT_DIR_OPIN, ALT_PWM_OPIN);
SDC_Motor motorAZ (64, AZ_DIR_OPIN,  AZ_PWM_OPIN);

void setup()
{
    pinMode(ENABLE_OPIN, OUTPUT);
    pinMode(ALT_PWM_OPIN, OUTPUT);
    pinMode(ALT_DIR_OPIN, OUTPUT);
    pinMode(AZ_PWM_OPIN, OUTPUT);
    pinMode(AZ_DIR_OPIN, OUTPUT);
    
    ALT_AZ_TCCRB = (ALT_AZ_TCCRB & 0b11111000) | ALT_AZ_PRESCALER;

    EP_EncodersSetup();

    motorALT.Setup();
    motorAZ.Setup();
    digitalWrite(ENABLE_OPIN, HIGH);

    Serial.begin(115200);
}

static void SetSpeed(byte buf[], int, int)
{
    long speed = long((uint32_t(buf[3]) << 24) + (uint32_t(buf[2]) << 16) + (uint32_t(buf[1]) << 8) + uint32_t(buf[0]));

    long upos, ts;
    motorALT.Start(double(speed)/(24.0*60.0*60000.), &upos, &ts);
    printHex2(upos);
    printHex2(ts);
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
    case 'S':   // speed
        SetSerialBuf(4, SetSpeed);
        break;

    case 'T':   // stop
        motorALT.Stop();
        Serial.print("r");
        break;

    case 'P':   // poll
        {
            long upos, ts, setpoint;
            motorALT.GetPos(&upos, &ts, &setpoint);
            printHex2(upos);
            printHex2(ts);
            printHex2(setpoint);
        }
        break;

    default:
    break;
    }

}

void loop()
{
    // motor
    bool safe = motorALT.Run();
    bool safe2 = motorAZ.Run();
    safe = safe && safe2;

    // serial
    static int safeSkip = 0;
    if(Serial.available() && (safe || ++safeSkip > 10))
    {
        safeSkip = 0;
        char inchar = Serial.read();
        ProcessSerialCommand(inchar);
    }

    /*
    move(1, s1, 1); //motor 1, full speed, left

    delay(2000); //go for 1 second
    stop(); //stop
    delay(250); //hold for 250ms until move again

    move(1, s2, 0); //motor 1, half speed, right

    delay(2000);
    stop();
    delay(250);

    move(1, s3, 1); //motor 1, full speed, left

    delay(2000); //go for 1 second
    stop(); //stop
    delay(250); //hold for 250ms until move again

    move(1, s4, 0); //motor 1, half speed, right

    delay(2000);
    stop();
    delay(250);

    move(1, s5, 1); //motor 1, full speed, left

    delay(2000); //go for 1 second
    stop(); //stop
    delay(250); //hold for 250ms until move again

    move(1, s6, 0); //motor 1, half speed, right

    delay(2000);
    stop();
    delay(250);
    */
}

