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

template <bool> struct compile_time_assert;
template<> struct compile_time_assert<true> {};

#define EEPROM_LAYOUT_OFFESET 0xcddc0506UL

// EEPROM layout
struct Layout
{
    // every time layout is changed, increment the version
#define EEPROM_LAYOUT_VERSION 1

    uint32_t LayoutVersion_;

    // common data
    uint8_t SessionId_;
    char padding1[64];

    // altitude configuration block
    SDC_Motor::Options AltMotorOptions_;
    char padding2[64];
    SDC_MotorAdapter::Options AltAdapterOptions_;
    char padding3[64];

    // azimuth configuration block
    SDC_Motor::Options AzmMotorOptions_;
    char padding4[64];
    SDC_MotorAdapter::Options AzmAdapterOptions_;
    //char padding5[64];

    Layout();
};
Layout::Layout() { compile_time_assert<sizeof(Layout) <= 512>(); }

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

template <typename T1, typename T2>
T1 GetEEPROM(const T1*, const T2*, size_t offset)
{
    return StorageFn<T1, T2>::Read(offset);
}

///////////////////////////////////////////////////////////////////////////////////////
#define WRITE_EEPROM(x,f)   WriteEEPROM(x, &((Layout*)0)->f, offsetof(Layout,f))
#define READ_EEPROM(px,f)   ReadEEPROM(px, &((Layout*)0)->f, offsetof(Layout,f))
#define GET_EEPROM(T,f)     GetEEPROM((T*)0, &((Layout*)0)->f, offsetof(Layout,f))

#define CURRENT_VERSION     (uint32_t(EEPROM_LAYOUT_OFFESET) + uint32_t(EEPROM_LAYOUT_VERSION))

///////////////////////////////////////////////////////////////////////////////////////
bool SDC_IsEpromVersionOK()
{
    return GET_EEPROM(uint32_t, LayoutVersion_) == CURRENT_VERSION;
}
void SDC_SaveCurrentEpromVersion()
{
    WRITE_EEPROM(CURRENT_VERSION, LayoutVersion_);
}

#define DEFINE_EEPROM_FUNCTIONS(Type,Name)                                  \
    void SDC_Read##Name(Type *pval)         { READ_EEPROM(pval, Name##_); } \
    void SDC_Write##Name(const Type &val)   { WRITE_EEPROM(val, Name##_); }

///////////////////////////////////////////////////////////////////////////////////////
DEFINE_EEPROM_FUNCTIONS(uint8_t, SessionId)
DEFINE_EEPROM_FUNCTIONS(SDC_Motor::Options, AltMotorOptions)
DEFINE_EEPROM_FUNCTIONS(SDC_Motor::Options, AzmMotorOptions)
DEFINE_EEPROM_FUNCTIONS(SDC_MotorAdapter::Options, AltAdapterOptions)
DEFINE_EEPROM_FUNCTIONS(SDC_MotorAdapter::Options, AzmAdapterOptions)
