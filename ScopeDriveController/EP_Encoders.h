/*
 * EP_Encoders.h
 *
 * Created: 10/5/2014 2:29:35 PM
 *  Author: Alexey
 */ 


#ifndef EP_ENCODERS_H_
#define EP_ENCODERS_H_

void EP_EncodersSetup();

long EP_GetAzEncoderResolution();
long EP_GetAltEncoderResolution();

long EP_GetAzEncoderPosition();
long EP_GetAltEncoderPosition();

void EP_SetAzEncoderResolution(long lAz);
void EP_SetAltEncoderResolution(long lAlt);

volatile long* EP_GetMotorAltEncoderPositionPtr();
volatile long* EP_GetMotorAzmEncoderPositionPtr();

#endif /* EP_ENCODERS_H_ */