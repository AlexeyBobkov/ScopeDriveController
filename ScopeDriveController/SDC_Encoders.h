/*
 * SDC_Encoders.h
 *
 * Created: 10/5/2014 2:29:35 PM
 *  Author: Alexey
 */


#ifndef SDC_ENCODERS_H_
#define SDC_ENCODERS_H_

void SDC_EncodersSetup();

// pointers to exact values
volatile long* SDC_GetAltEncoderPositionPtr();
volatile long* SDC_GetAzmEncoderPositionPtr();
volatile long* SDC_GetMotorAltEncoderPositionPtr();
volatile long* SDC_GetMotorAzmEncoderPositionPtr();

// scope encoder resolutions
long SDC_GetAltEncoderResolution();
long SDC_GetAzmEncoderResolution();

// scope encoder positions in the range (0..resolution-1)
long SDC_GetAltEncoderPosition();
long SDC_GetAzmEncoderPosition();

#endif /* SDC_ENCODERS_H_ */