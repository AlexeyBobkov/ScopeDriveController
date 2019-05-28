/*
 * SDC_Configuration.h
 *
 * Created: 01/17/2019
 *  Author: Alexey
 */ 


#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

///////////////////////////////////////////////////////////////////////////////////////
const long RESOLUTION = 1000*4;
const long BEARING_RATIO = 22;
const long ADDITIONAL_RATIO = 20;


///////////////////////////////////////////////////////////////////////////////////////
// sound
#define SOUND_OPIN              A0

///////////////////////////////////////////////////////////////////////////////////////
// motors configuration
#define SWITCH_IPIN             2       // switches
#define ENABLE_OPIN             10      // enable pin #
#define DIR1_OPIN               12      // direction 1 - connected to AIN1/2
#define DIR2_OPIN               13      // direction 2 - connected to BIN1/2

///////////////////////////////////////////////////////////////////////////////////////
// motor configuration PWM
// (configured to timer 2)
#define PWMA_OPIN               11      // PWMA
#define PWMB_OPIN               3       // PWMB
#define ALT_AZM_TCCRA            TCCR2A  // AZM/ALT timer/counter control register A
#define ALT_AZM_TCCRB           TCCR2B  // AZM/ALT timer/counter control register B
#define ALT_AZM_PRESCALER       0x01    // prescaler to 31373.55 Hz



///////////////////////////////////////////////////////////////////////////////////////
// Telescope altitude encoder configuration
///////////////////////////////////////////////////////////////////////////////////////
#define ALT_A_IPIN              8           // digital pin 8 = PB0
#define ALT_B_IPIN              9           // digital pin 9 = PB1

// Pin change mask registers decide which pins are enabled as triggers
#define ALT_A_PCINT             PCINT0      // arduino pin 8 = PB0 = alt encoder A
#define ALT_B_PCINT             PCINT1      // arduino pin 9 = PB1 = alt encoder B

// Pin change interrupt control register - enables interrupt vectors
// Bit 0 = enable PC vector 0 (PCINT7..0)
#define ALT_PCIE                PCIE0
#define ALT_PCMSK               PCMSK0
#define ALT_PCINT_vect          PCINT0_vect

// Input pins registers and bit offsets
#define ALT_PIN                 PINB
#define ALT_PIN_OFFSET          8


///////////////////////////////////////////////////////////////////////////////////////
// Telescope azimuth encoder configuration
///////////////////////////////////////////////////////////////////////////////////////
#define AZM_A_IPIN              A1          // analog pin A1 = PC1
#define AZM_B_IPIN              A2          // analog pin A2 = PC2

// Pin change mask registers decide which pins are enabled as triggers
#define AZM_A_PCINT             PCINT9      // arduino pin A1 = PC1 = azm encoder A
#define AZM_B_PCINT             PCINT10     // arduino pin A2 = PC2 = azm encoder B

// Pin change interrupt control register - enables interrupt vectors
// Bit 1 = enable PC vector 1 (PCINT14..8)
#define AZM_PCIE                PCIE1
#define AZM_PCMSK               PCMSK1
#define AZM_PCINT_vect          PCINT1_vect

// Input pins registers and bit offsets
#define AZM_PIN                 PINC
#define AZM_PIN_OFFSET          A0


///////////////////////////////////////////////////////////////////////////////////////
// Motor altitude encoder configuration
///////////////////////////////////////////////////////////////////////////////////////
#define MALT_A_IPIN             4           // digital pin 4 = PD4
#define MALT_B_IPIN             5           // digital pin 5 = PD5

// Pin change mask registers decide which pins are enabled as triggers
#define MALT_A_PCINT            PCINT20     // arduino pin 4 = PD4 = motor alt encoder A
#define MALT_B_PCINT            PCINT21     // arduino pin 5 = PD5 = motor alt encoder B

// Pin change interrupt control register - enables interrupt vectors
// Bit 2 = enable PC vector 2 (PCINT23..16)
#define MALT_PCIE               PCIE2
#define MALT_PCMSK              PCMSK2
#define MALT_PCINT_vect         PCINT2_vect

// Input pins registers and bit offsets
#define MALT_PIN                PIND
#define MALT_PIN_OFFSET         0


///////////////////////////////////////////////////////////////////////////////////////
// Motor azimuth encoder configuration
///////////////////////////////////////////////////////////////////////////////////////
#define MAZM_A_IPIN             6           // digital pin 6 = PD6
#define MAZM_B_IPIN             7           // digital pin 7 = PD7

// Pin change mask registers decide which pins are enabled as triggers
#define MAZM_A_PCINT            PCINT22     // arduino pin 6 = PD6 = motor azm encoder A
#define MAZM_B_PCINT            PCINT23     // arduino pin 7 = PD7 = motor azm encoder B

// Pin change interrupt control register - enables interrupt vectors
// Bit 2 = enable PC vector 2 (PCINT23..16)
#define MAZM_PCIE               PCIE2
#define MAZM_PCMSK              PCMSK2
#define MAZM_PCINT_vect         PCINT2_vect     // same as MALT_PCINT_vect

// Input pins registers and bit offsets
#define MAZM_PIN                PIND
#define MAZM_PIN_OFFSET         0


#endif /* CONFIGURATION_H_ */