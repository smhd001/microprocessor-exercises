#ifndef MICROLABTEMP_PROGRAM_H
#define MICROLABTEMP_PROGRAM_H
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

void progarm_init();
void sec_handel();
void tempretur_handle();
void update_state();
void senario(int key);
void update_lcd();

typedef struct {
	int temp_max;
	int temp_play_voice;
	int sec_active;
	int sec_play_voice;
	int light[4];
} Config;

typedef enum {
	cubic = 0x31, triangle, sinosoid
} BuzzerType;

typedef enum {
	tmp_warning,
	sec_warning,
	main_menu,
	temp_menu,
	sec_menu,
	light_menu,
	senario_menu,
	senario_eddite_menu,
	edit_hello,
	edit_bye
} LCD_state;

#ifdef __cplusplus
}
#endif

#endif //MICROLABTEMP_PROGRAM_H
