/*
 * SDC_Sound.h
 *
 * Created: 5/28/2019 8:39:41 PM
 *  Author: Alexey
 */ 


#ifndef SDC_SOUND_H_
#define SDC_SOUND_H_

extern void SDC_Sound_Setup();
extern void SDC_Sound_Run();
extern void SDC_Sound(unsigned long duration = 200, unsigned int frequency = 1000);

#endif /* SDC_SOUND_H_ */