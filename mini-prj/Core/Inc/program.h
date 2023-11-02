#ifndef MICROLABTEMP_PROGRAM_H
#define MICROLABTEMP_PROGRAM_H

#ifdef __cplusplus
extern "C" {
#endif

void program_init();
int abs(int a);

typedef enum {
	cubic = 0x31,
	triangle,
	sinosoid
}BuzzerType;

typedef struct {
    uint8_t buzzer_type ;
    uint8_t buzzer_min ;
    uint8_t buzzer_max;
    uint8_t temperature_max;
    uint8_t light_max;
}Config;


#ifdef __cplusplus
}
#endif

#endif //MICROLABTEMP_PROGRAM_H
