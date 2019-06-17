/*
 * SCD_Storage.h
 *
 * Created: 6/16/2019
 *  Author: Alexey
 */ 

#ifndef SDC_STORAGE_H_
#define SDC_STORAGE_H_

///////////////////////////////////////////////////////////////////////////////////////
void SDC_WriteSessionId(uint8_t sessionId);
bool SDC_ReadSessionId(uint8_t *pSessionId);

///////////////////////////////////////////////////////////////////////////////////////
long SDC_ReadDefEncResolutionAlt();
long SDC_ReadDefEncResolutionAzm();
void SDC_WriteDefEncResolutionAlt(long val);
void SDC_WriteDefEncResolutionAzm(long val);

#endif /* SDC_STORAGE_H_ */