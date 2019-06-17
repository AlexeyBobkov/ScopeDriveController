/*
 * SCD_Storage.cpp
 *
 * Created: 6/16/2019
 *  Author: Alexey
 */ 

#include "Arduino.h"
#include <EEPROM.h>

#include <stddef.h>
#include "SDC_Storage.h"

// EEPROM layout
struct Layout
{
    uint8_t uSessionId;
    long    lDefEncResolutionAlt;
    long    lDefEncResolutionAzm;
};


///////////////////////////////////////////////////////////////////////////////////////
template <typename T1, typename T2> struct StorageFn;
template <typename T> struct StorageFn<T, T>
{
    static void Write(const T &x, size_t offset)
    {
        for(int i = sizeof(T); --i >= 0;)
            EEPROM.write(offset+i, ((const uint8_t*)&x)[i]);
    }
    static void Read(T *px, size_t offset)
    {
        for(int i = sizeof(T); --i >= 0;)
            ((uint8_t*)px)[i] = EEPROM.read(offset+i);
    }
    static T Read(size_t offset)
    {
        T x;
        for(int i = sizeof(T); --i >= 0;)
            ((uint8_t*)&x)[i] = EEPROM.read(offset+i);
        return x;
    }
};

///////////////////////////////////////////////////////////////////////////////////////
template <typename T1, typename T2>
void WriteEEPROM(const T1 &x, const T2*, size_t offset)
{
    StorageFn<T1, T2>::Write(x, offset);
}
template <typename T1, typename T2>
void ReadEEPROM(T1 *px, const T2*, size_t offset)
{
    StorageFn<T1, T2>::Read(px, offset);
}

template <typename T>
T GetEEPROM(size_t offset)
{
    return StorageFn<T, T>::Read(offset);
}

///////////////////////////////////////////////////////////////////////////////////////
#define WRITE_EEPROM(x,f)   WriteEEPROM(x, &((Layout*)0)->f, offsetof(Layout,f))
#define READ_EEPROM(px,f)   ReadEEPROM(px, &((Layout*)0)->f, offsetof(Layout,f))
#define GET_EEPROM(T,f)     GetEEPROM<T>(offsetof(Layout,f))


///////////////////////////////////////////////////////////////////////////////////////
void SDC_WriteSessionId(uint8_t sessionId)
{
    WRITE_EEPROM(sessionId, uSessionId);
}
bool SDC_ReadSessionId(uint8_t *pSessionId)
{
    READ_EEPROM(pSessionId, uSessionId);
    return true;
}


///////////////////////////////////////////////////////////////////////////////////////
long SDC_ReadDefEncResolutionAlt()           {return GET_EEPROM(long, lDefEncResolutionAlt);}
long SDC_ReadDefEncResolutionAzm()           {return GET_EEPROM(long, lDefEncResolutionAzm);}
void SDC_WriteDefEncResolutionAlt(long val)  {WRITE_EEPROM(val, lDefEncResolutionAlt);}
void SDC_WriteDefEncResolutionAzm(long val)  {WRITE_EEPROM(val, lDefEncResolutionAzm);}
