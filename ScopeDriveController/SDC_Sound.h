/*
 * SDC_Sound.h
 *
 * Created: 5/28/2019 8:39:41 PM
 *  Author: Alexey
 */ 


#ifndef SDC_SOUND_H_
#define SDC_SOUND_H_

extern void SoundSetup();
extern void SoundRun();
extern void MakeSound(unsigned long duration, unsigned int frequency = 1000);

#endif /* SDC_SOUND_H_ */