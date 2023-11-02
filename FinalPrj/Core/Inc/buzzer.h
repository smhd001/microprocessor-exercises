#ifndef MICROLABTEMP_PROGRAM_H
#define MICROLABTEMP_PROGRAM_H
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif
void buzzerInit();
void buzzerChangeTone(uint16_t freq, uint16_t volume);
void Update_Melody();
void Change_Melody();
#ifdef __cplusplus
}
#endif

#endif //MICROLABTEMP_PROGRAM_H
