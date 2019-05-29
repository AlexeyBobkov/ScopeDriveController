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
static uint8_t level = LOW;

void SDC_Sound_Setup()
{
    endTs = 0;
    pinMode(SOUND_OPIN, OUTPUT);
}

void SDC_Sound_Run()
{
    if(!endTs)
        return;

    unsigned long ts = millis();
    if(ts > endTs)
    {
        endTs = 0;
        digitalWrite(SOUND_OPIN, LOW);
        return;
    }

    ts = micros() - periodTs;
    if(ts >= period2)
    {
        periodTs = ts;
        digitalWrite(SOUND_OPIN, level);
        level = (level == HIGH) ? LOW : HIGH;
    }
}

void SDC_Sound(unsigned long duration, unsigned int frequency)
{
    if(frequency <= 0 || !duration)
        return;

    endTs = millis() + duration;

    periodTs = micros();
    period2 = ((unsigned long)500000)/((unsigned long)frequency);
    if(!period2)
        period2 = 1;
}
