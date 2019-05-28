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

long ALT_res = 2500*4, AZM_res = 2500*4;                // resolution of telescope encoders
long MALT_res = 1000*4, MAZM_res = 1000*4;              // resolution of motor encoders
volatile long ALT_pos, AZM_pos, MALT_pos, MAZM_pos;     // encoder positions

///////////////////////////////////////////////////////////////////////////////////////
void EP_EncodersSetup()
{
    //ALT_res = EP_ReadDefEncResolutionAlt();
    //AZM_res = EP_ReadDefEncResolutionAzm();

    ALT_pos = ALT_res/2;
    AZM_pos = AZM_res/2;
    MALT_pos = MALT_res/2;
    MAZM_pos = MAZM_res/2;

    // initialize the encoder inputs
    pinMode(ALT_A_IPIN, INPUT);
    pinMode(ALT_B_IPIN, INPUT);
    pinMode(AZM_A_IPIN, INPUT);
    pinMode(AZM_B_IPIN, INPUT);
    pinMode(MALT_A_IPIN, INPUT);
    pinMode(MALT_B_IPIN, INPUT);
    pinMode(MAZM_A_IPIN, INPUT);
    pinMode(MAZM_B_IPIN, INPUT);

    // telescope altitude encoder interrupts
    PCICR |= (1 << ALT_PCIE);
    ALT_PCMSK |= (1 << ALT_A_PCINT)|(1 << ALT_B_PCINT);

    // telescope azimuth encoder interrupts
    PCICR |= (1 << AZM_PCIE);
    AZM_PCMSK |= (1 << AZM_A_PCINT)|(1 << AZM_B_PCINT);

    // motor altitude encoder interrupts
    PCICR |= (1 << MALT_PCIE);
    MALT_PCMSK |= (1 << MALT_A_PCINT)|(1 << MALT_B_PCINT);

    // motor azimuth encoder interrupts
    PCICR |= (1 << MAZM_PCIE);
    MAZM_PCMSK |= (1 << MAZM_A_PCINT)|(1 << MAZM_B_PCINT);

    // enable interrupts
    interrupts();
}

///////////////////////////////////////////////////////////////////////////////////////
long EP_GetAzEncoderResolution()   {return AZM_res;}
long EP_GetAltEncoderResolution()  {return ALT_res;}

///////////////////////////////////////////////////////////////////////////////////////
long EP_GetAzEncoderPosition()     {return AZM_pos;}
long EP_GetAltEncoderPosition()    {return ALT_pos;}

///////////////////////////////////////////////////////////////////////////////////////
volatile long* EP_GetMotorAltEncoderPositionPtr()   {return &MALT_pos;}
volatile long* EP_GetMotorAzmEncoderPositionPtr()   {return &MAZM_pos;}

///////////////////////////////////////////////////////////////////////////////////////
void EP_SetAzEncoderResolution(long lAz)
{
    if(AZM_res != lAz)
    {
        AZM_res = lAz;
        AZM_pos = AZM_res/2;
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
// Interrupts
typedef void (*FN_STATE)();

///////////////////////////////////////////////////////////////////////////////////////
#define INC_1(enc,s)                        \
        ++enc##_pos;                        \
        fn##enc##State = enc##State##s##_Fwd;

#define DEC_1(enc,s)                        \
        --enc##_pos;                        \
        fn##enc##State = enc##State##s##_Bck;

#define INC_2(enc,s)                        \
        enc##_pos += 2;                     \
        fn##enc##State = enc##State##s##_Fwd;

#define DEC_2(enc,s)                        \
        enc##_pos -= 2;                     \
        fn##enc##State = enc##State##s##_Bck;

#define A_SET(enc)      (1<<(enc##_A_IPIN-enc##_PIN_OFFSET))
#define B_SET(enc)      (1<<(enc##_B_IPIN-enc##_PIN_OFFSET))
#define AB_SET(enc)     (A_SET(enc)|B_SET(enc))

/////////////////////////////////////////////////////////////////////////
#define DEFINE_STATE_PROCEDURES(enc)                                    \
        static void enc##State00_Fwd();                                 \
        static void enc##State01_Fwd();                                 \
        static void enc##State10_Fwd();                                 \
        static void enc##State11_Fwd();                                 \
        static void enc##State00_Bck();                                 \
        static void enc##State01_Bck();                                 \
        static void enc##State10_Bck();                                 \
        static void enc##State11_Bck();                                 \
        static volatile FN_STATE fn##enc##State = enc##State00_Fwd;     \
        static void enc##State00_Fwd()                                  \
        {                                                               \
            switch(enc##_PIN & AB_SET(enc))                             \
            {                                                           \
                case 0:             return;                             \
                case B_SET(enc):    INC_1(enc, 01); return;             \
                case A_SET(enc):    DEC_1(enc, 10); return;             \
                case AB_SET(enc):   INC_2(enc, 11); return;             \
            }                                                           \
        }                                                               \
        static void enc##State01_Fwd()                                  \
        {                                                               \
            switch(enc##_PIN & AB_SET(enc))                             \
            {                                                           \
                case 0:             DEC_1(enc, 00); return;             \
                case B_SET(enc):    return;                             \
                case A_SET(enc):    INC_2(enc, 10); return;             \
                case AB_SET(enc):   INC_1(enc, 11); return;             \
            }                                                           \
        }                                                               \
        static void enc##State10_Fwd()                                  \
        {                                                               \
            switch(enc##_PIN & AB_SET(enc))                             \
            {                                                           \
                case 0:             INC_1(enc, 00);return;              \
                case B_SET(enc):    INC_2(enc, 01);return;              \
                case A_SET(enc):    return;                             \
                case AB_SET(enc):   DEC_1(enc, 11);return;              \
            }                                                           \
        }                                                               \
        static void enc##State11_Fwd()                                  \
        {                                                               \
            switch(enc##_PIN & AB_SET(enc))                             \
            {                                                           \
                case 0:             INC_2(enc, 00); return;             \
                case B_SET(enc):    DEC_1(enc, 01); return;             \
                case A_SET(enc):    INC_1(enc, 10); return;             \
                case AB_SET(enc):   return;                             \
            }                                                           \
        }                                                               \
        static void enc##State00_Bck()                                  \
        {                                                               \
            switch(enc##_PIN & AB_SET(enc))                             \
            {                                                           \
                case 0:             return;                             \
                case B_SET(enc):    INC_1(enc, 01); return;             \
                case A_SET(enc):    DEC_1(enc, 10); return;             \
                case AB_SET(enc):   DEC_2(enc, 11); return;             \
            }                                                           \
        }                                                               \
        static void enc##State01_Bck()                                  \
        {                                                               \
            switch(enc##_PIN & AB_SET(enc))                             \
            {                                                           \
                case 0:             DEC_1(enc, 00); return;             \
                case B_SET(enc):    return;                             \
                case A_SET(enc):    DEC_2(enc, 10); return;             \
                case AB_SET(enc):   INC_1(enc, 11); return;             \
            }                                                           \
        }                                                               \
        static void enc##State10_Bck()                                  \
        {                                                               \
            switch(enc##_PIN & AB_SET(enc))                             \
            {                                                           \
                case 0:             INC_1(enc, 00);return;              \
                case B_SET(enc):    DEC_2(enc, 01);return;              \
                case A_SET(enc):    return;                             \
                case AB_SET(enc):   DEC_1(enc, 11);return;              \
            }                                                           \
        }                                                               \
        static void enc##State11_Bck()                                  \
        {                                                               \
            switch(enc##_PIN & AB_SET(enc))                             \
            {                                                           \
                case 0:             DEC_2(enc, 00); return;             \
                case B_SET(enc):    DEC_1(enc, 01); return;             \
                case A_SET(enc):    INC_1(enc, 10); return;             \
                case AB_SET(enc):   return;                             \
            }                                                           \
        }
/////////////////////////////////////////////////////////////////////////

DEFINE_STATE_PROCEDURES(ALT)
ISR(ALT_PCINT_vect)
{
    (*fnALTState)();
}

DEFINE_STATE_PROCEDURES(AZM)
ISR(AZM_PCINT_vect)
{
    (*fnAZMState)();
}

DEFINE_STATE_PROCEDURES(MALT)
DEFINE_STATE_PROCEDURES(MAZM)
ISR(MALT_PCINT_vect)
{
    (*fnMALTState)();
    (*fnMAZMState)();
}
