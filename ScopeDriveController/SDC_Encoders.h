/*
 * SDC_Encoders.h
 *
 * Created: 10/5/2014 2:29:35 PM
 *  Author: Alexey
 */


#ifndef SDC_ENCODERS_H_
#define SDC_ENCODERS_H_

void SDC_EncodersSetup();

volatile long* SDC_GetAltEncoderPositionPtr();
volatile long* SDC_GetAzmEncoderPositionPtr();
volatile long* SDC_GetMotorAltEncoderPositionPtr();
volatile long* SDC_GetMotorAzmEncoderPositionPtr();

#endif /* SDC_ENCODERS_H_ */