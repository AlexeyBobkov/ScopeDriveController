/*
 * EP_Encoders.cpp
 *
 * Created: 10/5/2014 2:29:13 PM
 *  Author: Alexey
 */ 

#include "Arduino.h"

#include "EP_Encoders.h"
#include "SDC_Configuration.h"
//#include "EP_Storage.h"

// if defined, both A and B generate interrupts, otherwise B only.
#define A_AND_B_INTERRUPTS

#if (AZ_A_IPIN > 7 || AZ_B_IPIN > 7)
#error Bad azimuth encoder configuration
#endif

#if (ALT_A_IPIN < 8 || ALT_A_IPIN > 13 || ALT_B_IPIN < 8 || ALT_B_IPIN > 13)
#error Bad altitude encoder configuration
#endif

// Pin change interrupt control register - enables interrupt vectors
// Bit 2 = enable PC vector 2 (PCINT23..16)
// Bit 1 = enable PC vector 1 (PCINT14..8)
// Bit 0 = enable PC vector 0 (PCINT7..0)
#define AZ_PCIE         PCIE2
#define AZ_PCMSK        PCMSK2
#define AZ_PCINT_vect   PCINT2_vect
#define ALT_PCIE        PCIE0
#define ALT_PCMSK       PCMSK0
#define ALT_PCINT_vect  PCINT0_vect

// Pin change mask registers decide which pins are enabled as triggers
#define AZ_A_PCINT      PCINT22     // arduino pin 6 = PD6 = az encoder A
#define AZ_B_PCINT      PCINT23     // arduino pin 7 = PD7 = az encoder B
#define ALT_A_PCINT     PCINT0      // arduino pin 8 = PB0 = alt encoder A
#define ALT_B_PCINT     PCINT1      // arduino pin 9 = PB1 = alt encoder B

// Input pins registers and bit offsets
#define AZ_PIN          PIND
#define AZ_PIN_OFFSET   0
#define ALT_PIN         PINB
#define ALT_PIN_OFFSET  8

#define AZ_A_SET        (1<<(AZ_A_IPIN-AZ_PIN_OFFSET))
#define AZ_B_SET        (1<<(AZ_B_IPIN-AZ_PIN_OFFSET))
#define AZ_AB_SET       ((1<<(AZ_A_IPIN-AZ_PIN_OFFSET))|(1<<(AZ_B_IPIN-AZ_PIN_OFFSET)))

#define ALT_A_SET       (1<<(ALT_A_IPIN-ALT_PIN_OFFSET))
#define ALT_B_SET       (1<<(ALT_B_IPIN-ALT_PIN_OFFSET))
#define ALT_AB_SET      ((1<<(ALT_A_IPIN-ALT_PIN_OFFSET))|(1<<(ALT_B_IPIN-ALT_PIN_OFFSET)))

long ALT_res = 1800*4, AZ_res = 1800*4;  // resolution of encoders
long ALT_pos, AZ_pos;                   // encoder positions

static uint8_t errors = 0;

///////////////////////////////////////////////////////////////////////////////////////
void EP_EncodersSetup()
{
    //ALT_res = EP_ReadDefEncResolutionAlt();
    //AZ_res = EP_ReadDefEncResolutionAzm();

    ALT_pos = ALT_res/2;
    AZ_pos = AZ_res/2;

    // initialize the encoder inputs
    pinMode(ALT_A_IPIN, INPUT);
    pinMode(ALT_B_IPIN, INPUT);
    pinMode(AZ_A_IPIN, INPUT);
    pinMode(AZ_B_IPIN, INPUT);

    Serial.begin(9600);

    // azimuth encoder interrupts
    PCICR |= (1 << AZ_PCIE);
#ifdef A_AND_B_INTERRUPTS
    AZ_PCMSK  |= (1 << AZ_A_PCINT)|(1 << AZ_B_PCINT);
#else
    AZ_PCMSK  |= (1 << AZ_B_PCINT);
#endif

    // altitude encoder interrupts
    PCICR |= (1 << ALT_PCIE);
#ifdef A_AND_B_INTERRUPTS
    ALT_PCMSK |= (1 << ALT_A_PCINT)|(1 << ALT_B_PCINT);
#else
    ALT_PCMSK |= (1 << ALT_B_PCINT);
#endif

    // enable interrupts
    interrupts();
}

///////////////////////////////////////////////////////////////////////////////////////
long EP_GetAzEncoderResolution()   {return AZ_res;}
long EP_GetAltEncoderResolution()  {return ALT_res;}

///////////////////////////////////////////////////////////////////////////////////////
long EP_GetAzEncoderPosition()      {return AZ_pos;}
long EP_GetAltEncoderPosition()     {return ALT_pos;}

///////////////////////////////////////////////////////////////////////////////////////
void EP_SetAzEncoderResolution(long lAz)
{
    if(AZ_res != lAz)
    {
        AZ_res = lAz;
        AZ_pos = AZ_res/2;
    }
}
void EP_SetAltEncoderResolution(long lAlt)
{
    if(ALT_res != lAlt)
    {
        ALT_res = lAlt;
        ALT_pos = ALT_res/2;
    }
}

///////////////////////////////////////////////////////////////////////////////////////
uint8_t EP_GetEncoderErrorCount()
{
    uint8_t tmp = errors;
    errors = 0;
    return tmp;
}

///////////////////////////////////////////////////////////////////////////////////////
// Interrupts
#ifndef A_AND_B_INTERRUPTS
ISR(AZ_PCINT_vect)
{
    int a = AZ_PIN & AZ_AB_SET;
    if (a == AZ_AB_SET || a == 0)
    {
        AZ_pos -= 2;
        if (AZ_pos < 0)
            AZ_pos = AZ_res - 2;
    }
    else
    {
        AZ_pos += 2;
        if (AZ_pos >= AZ_res)
            AZ_pos = 0;
    }
}
ISR(ALT_PCINT_vect)
{
    int a = ALT_PIN & ALT_AB_SET;
    if (a == ALT_AB_SET || a == 0)
    {
        ALT_pos -= 2;
        if (ALT_pos < 0)
            ALT_pos = ALT_res - 2;
    }
    else
    {
        ALT_pos += 2;
        if (ALT_pos >= ALT_res)
            ALT_pos = 0;
    }
}

#else   //A_AND_B_INTERRUPTS

typedef void (*FN_STATE)();

static void AZState00_Fwd();
static void AZState01_Fwd();
static void AZState10_Fwd();
static void AZState11_Fwd();
static void AZState00_Bck();
static void AZState01_Bck();
static void AZState10_Bck();
static void AZState11_Bck();

static void ALTState00_Fwd();
static void ALTState01_Fwd();
static void ALTState10_Fwd();
static void ALTState11_Fwd();
static void ALTState00_Bck();
static void ALTState01_Bck();
static void ALTState10_Bck();
static void ALTState11_Bck();

static FN_STATE fnAZState = AZState00_Fwd, fnALTState = ALTState00_Fwd;

///////////////////////////////////////////////////////////////////////////////////////
#define INC_1(enc,s)                        \
        if (enc##_pos == enc##_res - 1)     \
            enc##_pos = 0;                  \
        else                                \
            ++enc##_pos;                    \
        fn##enc##State = enc##State##s##_Fwd;

#define DEC_1(enc,s)                        \
        if (!enc##_pos)                     \
            enc##_pos = enc##_res - 1;      \
        else                                \
            --enc##_pos;                    \
        fn##enc##State = enc##State##s##_Bck;

#define INC_2(enc,s)                        \
        ++errors;                           \
        if ((enc##_pos += 2) >= enc##_res)  \
            enc##_pos -= enc##_res;         \
        fn##enc##State = enc##State##s##_Fwd;

#define DEC_2(enc,s)                        \
        ++errors;                           \
        if ((enc##_pos -= 2) < 0)           \
            enc##_pos += enc##_res;         \
        fn##enc##State = enc##State##s##_Bck;

///////////////////////////////////////////////////////////////////////////////////////
static void AZState00_Fwd()
{
    switch(AZ_PIN & AZ_AB_SET)
    {
    case 0:             ++errors; return;
    case AZ_B_SET:      INC_1(AZ, 01); return;
    case AZ_A_SET:      DEC_1(AZ, 10); return;
    case AZ_AB_SET:     INC_2(AZ, 11); return;
    }
}
static void AZState01_Fwd()
{
    switch(AZ_PIN & AZ_AB_SET)
    {
    case 0:             DEC_1(AZ, 00); return;
    case AZ_B_SET:      ++errors; return;
    case AZ_A_SET:      INC_2(AZ, 10); return;
    case AZ_AB_SET:     INC_1(AZ, 11); return;
    }
}
static void AZState10_Fwd()
{
    switch(AZ_PIN & AZ_AB_SET)
    {
    case 0:             INC_1(AZ, 00);return;
    case AZ_B_SET:      INC_2(AZ, 01);return;
    case AZ_A_SET:      ++errors; return;
    case AZ_AB_SET:     DEC_1(AZ, 11);return;
    }
}
static void AZState11_Fwd()
{
    switch(AZ_PIN & AZ_AB_SET)
    {
    case 0:             INC_2(AZ, 00); return;
    case AZ_B_SET:      DEC_1(AZ, 01); return;
    case AZ_A_SET:      INC_1(AZ, 10); return;
    case AZ_AB_SET:     ++errors; return;
    }
}
static void AZState00_Bck()
{
    switch(AZ_PIN & AZ_AB_SET)
    {
    case 0:             ++errors; return;
    case AZ_B_SET:      INC_1(AZ, 01); return;
    case AZ_A_SET:      DEC_1(AZ, 10); return;
    case AZ_AB_SET:     DEC_2(AZ, 11); return;
    }
}
static void AZState01_Bck()
{
    switch(AZ_PIN & AZ_AB_SET)
    {
    case 0:             DEC_1(AZ, 00); return;
    case AZ_B_SET:      ++errors; return;
    case AZ_A_SET:      DEC_2(AZ, 10); return;
    case AZ_AB_SET:     INC_1(AZ, 11); return;
    }
}
static void AZState10_Bck()
{
    switch(AZ_PIN & AZ_AB_SET)
    {
    case 0:             INC_1(AZ, 00);return;
    case AZ_B_SET:      DEC_2(AZ, 01);return;
    case AZ_A_SET:      ++errors; return;
    case AZ_AB_SET:     DEC_1(AZ, 11);return;
    }
}
static void AZState11_Bck()
{
    switch(AZ_PIN & AZ_AB_SET)
    {
    case 0:             DEC_2(AZ, 00); return;
    case AZ_B_SET:      DEC_1(AZ, 01); return;
    case AZ_A_SET:      INC_1(AZ, 10); return;
    case AZ_AB_SET:     ++errors; return;
    }
}

ISR(AZ_PCINT_vect)
{
    (*fnAZState)();
}

///////////////////////////////////////////////////////////////////////////////////////
static void ALTState00_Fwd()
{
    switch(ALT_PIN & ALT_AB_SET)
    {
    case 0:             ++errors; return;
    case ALT_B_SET:      INC_1(ALT, 01); return;
    case ALT_A_SET:      DEC_1(ALT, 10); return;
    case ALT_AB_SET:     INC_2(ALT, 11); return;
    }
}
static void ALTState01_Fwd()
{
    switch(ALT_PIN & ALT_AB_SET)
    {
    case 0:             DEC_1(ALT, 00); return;
    case ALT_B_SET:     ++errors; return;
    case ALT_A_SET:     INC_2(ALT, 10); return;
    case ALT_AB_SET:    INC_1(ALT, 11); return;
    }
}
static void ALTState10_Fwd()
{
    switch(ALT_PIN & ALT_AB_SET)
    {
    case 0:             INC_1(ALT, 00);return;
    case ALT_B_SET:     INC_2(ALT, 01);return;
    case ALT_A_SET:     ++errors; return;
    case ALT_AB_SET:    DEC_1(ALT, 11);return;
    }
}
static void ALTState11_Fwd()
{
    switch(ALT_PIN & ALT_AB_SET)
    {
    case 0:             INC_2(ALT, 00); return;
    case ALT_B_SET:     DEC_1(ALT, 01); return;
    case ALT_A_SET:     INC_1(ALT, 10); return;
    case ALT_AB_SET:    ++errors; return;
    }
}
static void ALTState00_Bck()
{
    switch(ALT_PIN & ALT_AB_SET)
    {
    case 0:             ++errors; return;
    case ALT_B_SET:     INC_1(ALT, 01); return;
    case ALT_A_SET:     DEC_1(ALT, 10); return;
    case ALT_AB_SET:    DEC_2(ALT, 11); return;
    }
}
static void ALTState01_Bck()
{
    switch(ALT_PIN & ALT_AB_SET)
    {
    case 0:             DEC_1(ALT, 00); return;
    case ALT_B_SET:     ++errors; return;
    case ALT_A_SET:     DEC_2(ALT, 10); return;
    case ALT_AB_SET:    INC_1(ALT, 11); return;
    }
}
static void ALTState10_Bck()
{
    switch(ALT_PIN & ALT_AB_SET)
    {
    case 0:             INC_1(ALT, 00);return;
    case ALT_B_SET:     DEC_2(ALT, 01);return;
    case ALT_A_SET:     ++errors; return;
    case ALT_AB_SET:    DEC_1(ALT, 11);return;
    }
}
static void ALTState11_Bck()
{
    switch(ALT_PIN & ALT_AB_SET)
    {
    case 0:             DEC_2(ALT, 00); return;
    case ALT_B_SET:     DEC_1(ALT, 01); return;
    case ALT_A_SET:     INC_1(ALT, 10); return;
    case ALT_AB_SET:    ++errors; return;
    }
}

ISR(ALT_PCINT_vect)
{
    (*fnALTState)();
}

#endif //A_AND_B_INTERRUPTS
