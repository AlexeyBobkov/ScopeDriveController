/*
 * SDC_Configuration.h
 *
 * Created: 01/17/2019
 *  Author: Alexey
 */ 


#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_


//#define FWD_SWITCH_IPIN         2       // forward motion switch pin #
//#define REV_SWITCH_IPIN         3       // reverse motion switch pin #
//#define MOTOR_STEPS_PER_REV     400     // motor steps per revolution

///////////////////////////////////////////////////////////////////////////////////////
// keypad configuration
//#define KPD_COUNT_OPIN          3
//#define KPD_RESET_OPIN          2
//#define KPD_VALUE_IPIN          A1

///////////////////////////////////////////////////////////////////////////////////////
// sound
#define SOUND_PIN               A0

///////////////////////////////////////////////////////////////////////////////////////
// motors configuration
#define ENABLE_OPIN             4       // enable pin #
#define AZ_DIR_OPIN             13      // AZ direction pin #
#define ALT_DIR_OPIN            12      // ALT direction pin #

///////////////////////////////////////////////////////////////////////////////////////
// altitude/azimuth PWM
// (configured to timer 2)
#define AZ_PWM_OPIN             3       // AZ PWM pin #
#define ALT_PWM_OPIN            11      // ALT PWM pin #
#define ALT_AZ_TCCRA            TCCR2A  // AZ/ALT timer/counter control register A
#define ALT_AZ_TCCRB            TCCR2B  // AZ/ALT timer/counter control register B
#define ALT_AZ_PRESCALER        0x01    // prescaler to 31373.55 Hz

///////////////////////////////////////////////////////////////////////////////////////
// altitude/azimuth encoders
#define AZ_A_IPIN               6       // digital pin 6 = PD6
#define AZ_B_IPIN               7       // digital pin 7 = PD7

#define ALT_A_IPIN              8       // digital pin 8 = PB0
#define ALT_B_IPIN              9       // digital pin 9 = PB1

#endif /* CONFIGURATION_H_ */