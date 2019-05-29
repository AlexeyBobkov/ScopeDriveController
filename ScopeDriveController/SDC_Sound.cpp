/*
 * SDC_Sound.cpp
 *
 * Created: 5/28/2019 8:42:14 PM
 *  Author: Alexey
 */ 

#include <Arduino.h>

#include "SDC_Configuration.h"
#include "SDC_Sound.h"

static unsigned long endTs, periodTs, period2;

void SoundSetup()
{
    endTs = 0;
    pinMode(SOUND_OPIN, OUTPUT);
}

void SoundRun()
{
    if(!endTs)
        return;

    if(millis() > endTs)
    {
        endTs = 0;
        digitalWrite(SOUND_OPIN, LOW);
    }
    else
    {
        unsigned long ts = micros();
        if(ts - periodTs >= period2)
        {
            periodTs = ts;
            digitalWrite(SOUND_OPIN, digitalRead(SOUND_OPIN) ? LOW : HIGH);
        }
    }
}

void MakeSound(unsigned long duration, unsigned int frequency)
{
    if(frequency <= 0 || !duration)
        return;

    endTs = millis() + duration;
    periodTs = micros();
    period2 = ((unsigned long)500000)/((unsigned long)frequency);
}
