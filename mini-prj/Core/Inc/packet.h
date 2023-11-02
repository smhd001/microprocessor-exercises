#ifndef MICROLABTEMP_PACKET_H
#define MICROLABTEMP_PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define PREAMBLE 0XAA
# define PI 3.14159265358979323846

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

typedef struct {
    uint8_t dst;
    uint8_t src;
    uint8_t len;
    uint8_t data[255];
    uint8_t crc;
}packet;

typedef enum {
    RX_IDLE,
    READ_DST,
    READ_SRC,
	DATA_LEN,
	DATA,
	CRC_B,
}RxState;



void stateMachine(uint8_t gotByte);
void send_data(int t,int l);
void num_to_seven(int i);
void buzzerChangeTone(uint16_t freq, uint16_t volume);
void buzzerInit();
int average_l(int x);
int average_t(int x);




#ifdef __cplusplus
}
#endif

#endif //MICROLABTEMP_PACKET_H
