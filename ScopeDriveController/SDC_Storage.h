/*
 * SCD_Storage.h
 *
 * Created: 6/16/2019
 *  Author: Alexey
 */ 

#ifndef SDC_STORAGE_H_
#define SDC_STORAGE_H_

#include "SDC_EncPositionAdapter.h"

///////////////////////////////////////////////////////////////////////////////////////
bool SDC_IsEpromVersionOK();
void SDC_SaveCurrentEpromVersion();

#define DECLARE_EEPROM_FUNCTIONS(Type,Name) \
    void SDC_Read##Name(Type *pval);        \
    void SDC_Write##Name(const Type &val);

DECLARE_EEPROM_FUNCTIONS(uint8_t, SessionId)
DECLARE_EEPROM_FUNCTIONS(SDC_Motor::Options, AltMotorOptions)
DECLARE_EEPROM_FUNCTIONS(SDC_Motor::Options, AzmMotorOptions)
DECLARE_EEPROM_FUNCTIONS(SDC_MotorAdapter::Options, AltAdapterOptions)
DECLARE_EEPROM_FUNCTIONS(SDC_MotorAdapter::Options, AzmAdapterOptions)


#endif /* SDC_STORAGE_H_ */