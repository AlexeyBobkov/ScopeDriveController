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

void setup()
{
    pinMode(ENABLE_OPIN, OUTPUT);
    pinMode(ALT_PWM_OPIN, OUTPUT);
    pinMode(ALT_DIR_OPIN, OUTPUT);
    pinMode(AZ_PWM_OPIN, OUTPUT);
    pinMode(AZ_DIR_OPIN, OUTPUT);
    
    ALT_AZ_TCCRB = (ALT_AZ_TCCRB & 0b11111000) | ALT_AZ_PRESCALER;

    EP_EncodersSetup();
}

int s1 = 255;
int s2 = 128;
int s3 = 64;
int s4 = 32;
int s5 = 16;
int s6 = 8;

/*
int s1 = 15;
int s2 = 13;
int s3 = 11;
int s4 = 10;
int s5 = 9;
int s6 = 8;
*/

SDC_Motor motorALT(ALT_DIR_OPIN, ALT_PWM_OPIN);
SDC_Motor motorAZ (AZ_DIR_OPIN,  AZ_PWM_OPIN);

void move(int motor, int speed, int direction)
{
    //Move specific motor at speed and direction
    //motor: 0 for B 1 for A
    //speed: 0 is off, and 255 is full speed
    //direction: 0 clockwise, 1 counter-clockwise

    digitalWrite(ENABLE_OPIN, HIGH); //disable standby

    motorALT.Move(direction ? speed : -speed);
}

void stop()
{
    //enable standby
    digitalWrite(ENABLE_OPIN, LOW);
    motorALT.Move(0);
}

void loop()
{
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
}

