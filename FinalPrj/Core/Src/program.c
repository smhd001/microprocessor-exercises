#include "main.h"
#include "program.h"
#include "cmsis_os.h"
#include "LiquidCrystal.h"
#include "scenario.h"
#include "uart.h"
#include <stdio.h>

extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc2;
Config conf = { 36, 1, 0, 0, { 0, 0, 0, 0 } };
const Config default_scenario = { 0, 2, 2, 2, { 2, 2, 2, 2 } };
extern Config scenarios[];
int eddting_sen_num = 0;

LCD_state state = main_menu;
int num = 0;
int LED[4] = { GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_15 };
extern TIM_HandleTypeDef htim1;
extern int key;
int temp = 0;
int volume = 0;
int senario_eddite_mode = 0;
extern int pos;
char goodbye_msg[100] = "Goodbye";
char hello_msg[100] = "Welcome";
char last_char = 0;
char new_msg[100];

void progarm_init() {
	buzzerInit();
	HAL_ADC_Start_IT(&hadc2);
}
void sec_handel() {
	static LCD_state last_state;
	if (conf.sec_active) {
		last_state = state;
		state = sec_warning;
		update_lcd();
		if (conf.sec_play_voice) {
			for (size_t i = 0; i < 10; i++) {
				if (i % 2) {
					buzzerChangeTone(1000, 100);
				} else {
					buzzerChangeTone(0, 0);
				}
				osDelay(500);
			}
		}
		buzzerChangeTone(0, 0);
		state = last_state;
		update_lcd();
		log_uart("security warning");
	}
}

void tempretur_handle() {
	static LCD_state last_state;
	HAL_ADC_Start_IT(&hadc2);
	//    int t = 0;
	int t_time = 0;
	int static first = 1;
	if (temp > conf.temp_max && temp < 50) {
		if (conf.temp_play_voice) {
			t_time++;
			if (t_time > 1000) {
				t_time = 0;
			}
			buzzerChangeTone(t_time, 100);
		} else {
			buzzerChangeTone(0, 0);
		}
		if (first) {
			osDelay(1000);
			last_state = state;
			state = tmp_warning;
			first = 0;
			update_lcd();
		}
	} else {
		if (!first) {
			state = last_state;
			buzzerChangeTone(0, 0);
			first = 1;
			log_uart("temp warning temperature is %d", temp);
			update_lcd();
		}
	}
}
int edit_massage(char key, char *msg) {
	static char last_key = -1;
	static int i = 0;

	if (key == 'A') {
		return 1;
	}
	if (key == 'B') {
		strcpy(msg, new_msg);
		new_msg[0] = 0;
		i = 0;
	}
	if (key == 'C') {
		new_msg[0] = 0;
		i = 0;
	}
	if (key == 'D') {
		new_msg[i] = last_char;
		new_msg[i + 1] = 0;
		i++;
	}
	//enter message using keypad
	if (last_key == key) {
		if (key < 7) {
			char base = last_char - (((last_char % 3 - 1)  % 3 + 3) % 3);
			last_char = (last_char - base + 1) % 3 + base;
		} else if (key == 7){
			char base = 'p';
			last_char = (last_char - base + 1) % 4 + base;
		}else if (key == 8)
		{
			char base = 't';
			last_char = (last_char - base + 1) % 3 + base;
		}else if (key == 9)
		{
			char base = 'w';
			last_char = (last_char - base + 1) % 4 + base;
		}
		return 0;
	} else {
		if (key < 7) {
			last_char = (key - 2) * 3 + 'a';
		} else if (key == 7) {
			last_char = 'p';
		} else if (key == 8) {
			last_char = 't';
		} else if (key == 9) {
			last_char = 'w';
		}
	}
	last_key = key;
	return 0;
}
void update_state() {
	switch (state) {
	case main_menu:
		switch (key) {
		case 'A':
			state = temp_menu;
			break;
		case 'B':
			state = sec_menu;
			break;
		case 'C':
			state = light_menu;
			break;
		case 'D':
			state = senario_menu;
			break;
		case 1:
			state = edit_hello;
			break;
		case 2:
			state = edit_bye;
			break;
		}
		break;
	case temp_menu:
		if (key >= 0 && key <= 9) {
			num = num * 10 + key;
			break;
		}
		switch (key) {
		case 'A':
			num = 0;
			state = main_menu;
			break;
		case 'B':
			conf.temp_max = num;
			log_uart("max temp changed to %d", conf.temp_max);
			num = 0;
			break;
		case 'C':
			num = 0;
			break;
		case 'D':
			conf.temp_play_voice = !conf.temp_play_voice;
			log_uart("playing voice for temp %s",
					conf.temp_play_voice ? "enabled" : "disabled");
			break;
		default:
			break;
		}
		break;
	case sec_menu:
		switch (key) {
		case 'A':
			num = 0;
			state = main_menu;
			break;
		case 'B':
			conf.sec_active = !conf.sec_active;
			log_uart("security %s", conf.sec_active ? "enabled" : "disabled");
			break;
		case 'C':
			conf.sec_play_voice = !conf.sec_play_voice;
			log_uart("playing voice for security %s",
					conf.sec_play_voice ? "enabled" : "disabled");
			break;
		default:
			break;
		}
		break;
	case light_menu:
		if (key >= 1 && key <= 4) {
			conf.light[key - 1] = !conf.light[key - 1];
			HAL_GPIO_TogglePin(GPIOE, LED[key - 1]);
			log_uart("light %d %s", key,
					conf.light[key - 1] ? "enabled" : "disabled");
		}
		if (key == 'A') {
			state = main_menu;
		}
		break;
	case senario_menu:
		if (key == 'A') {
			state = main_menu;
			break;
		}
		if (key == 'B') {
			senario_eddite_mode = !senario_eddite_mode;
			break;
		}
		if (key > 0 && key <= 9) {
			if (senario_eddite_mode) {
				eddting_sen_num = key;
				num = scenarios[eddting_sen_num - 1].temp_max;
				state = senario_eddite_menu;
				break;
			} else {
				set_scenario(scenarios[key - 1]);
				log_uart("scenario %d activated", key);
				break;
			}
		}
	case senario_eddite_menu:
		if (scenario_edite(key, &scenarios[eddting_sen_num - 1])) {
			state = senario_menu;
		}
	case tmp_warning:
		if (key == 'A') {
			state = main_menu;
		}
	case sec_warning:
		if (key == 'A') {
			state = main_menu;
		}
	case edit_hello:
		if (key == 'A') {
			state = main_menu;
		}
		if (edit_massage(key, hello_msg)) {
			state = main_menu;
		}
		break;
	case edit_bye:
		if (key == 'A') {
			state = main_menu;
		}
		if (edit_massage(key, goodbye_msg)) {
			state = main_menu;
		}
		break;
	default:
		break;
		break;
	}
	key = -1;
	update_lcd();
}
void update_lcd() {
	char s[20];

	switch (state) {
	case main_menu:
		setCursor(0, 0);
		clear();
		print("A temp ");
		print("B sec ");
		setCursor(0, 1);
		print("C light ");
		print("D scen ");
		setCursor(0, 2);
		print("1 ed hello ");
		print("2 ed bye");
		break;
	case temp_menu:
		clear();
		setCursor(0, 0);
		sprintf(s, "temp threshold: %d", conf.temp_max);
		print(s);
		setCursor(0, 1);
		sprintf(s, "new temp treshold:%d", num);
		print(s);
		setCursor(0, 2);
		sprintf(s, "Alarm status is:%d", conf.temp_play_voice);
		print(s);
		break;
	case sec_menu:
		clear();
		setCursor(0, 0);
		sprintf(s, "Security status: %d", conf.sec_active);
		print(s);
		setCursor(0, 1);
		sprintf(s, "Alarm status is: %d", conf.sec_play_voice);
		print(s);
		break;
	case light_menu:
		clear();
		setCursor(0, 0);
		if (conf.light[0] == 0)
			print("X");
		else
			print("O");

		setCursor(19, 0);
		if (conf.light[1] == 0)
			print("X");
		else
			print("O");

		setCursor(0, 3);
		if (conf.light[2] == 0)
			print("X");
		else
			print("O");

		setCursor(19, 3);
		if (conf.light[3] == 0)
			print("X");
		else
			print("O");
		break;
	case senario_menu:
		clear();
		setCursor(0, 0);
		if (senario_eddite_mode) {
			print("scen eddite mode");
		} else {
			print("scen action mode");
		}
		setCursor(0, 1);
		print("enter scen number");

		break;
	case senario_eddite_menu:
		clear();
		setCursor(0, 0);
		sprintf(s, "temp threshold: %d", num);
		print(s);
		setCursor(0, 1);
		sprintf(s, "[ %d , %d , %d]",
				scenarios[eddting_sen_num - 1].temp_play_voice,
				scenarios[eddting_sen_num - 1].sec_active,
				scenarios[eddting_sen_num - 1].sec_play_voice);
		print(s);
		setCursor(0, 2);
		sprintf(s, "[ %d , %d , %d , %d]",
				scenarios[eddting_sen_num - 1].light[0],
				scenarios[eddting_sen_num - 1].light[1],
				scenarios[eddting_sen_num - 1].light[2],
				scenarios[eddting_sen_num - 1].light[3]);
		print(s);
		setCursor(0, 3);
		sprintf(s, "scen is %d pos is %d", pos, eddting_sen_num);
		print(s);
		break;
	case tmp_warning:
		clear();
		setCursor(0, 2);
		print("  temp is too high  ");
		break;
	case sec_warning:
		clear();
		setCursor(0, 2);
		print("  security Warning  ");
		break;
	case edit_hello:
		clear();
		setCursor(0, 0);
		print("editing greeting ");
		setCursor(0, 1);
		print("new msg:");
		setCursor(0, 2);
		print(new_msg);
		sprintf(s, "%c", last_char);
		print(s);
		break;
	case edit_bye:
		clear();
		setCursor(0, 0);
		print("editing goodBye ");
		setCursor(0, 1);
		print("new msg:");
		setCursor(0, 2);
		print(new_msg);
		sprintf(s, "%c", last_char);
		print(s);
		break;
	}
}
