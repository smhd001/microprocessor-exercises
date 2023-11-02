/*
 * scenario.c
 *
 *  Created on: Jul 5, 2023
 *      Author: mhmmd
 */
#include "program.h"
#include "main.h"


extern Config conf;
extern int num;
extern int LED[];
int pos = 0;

Config scenarios[9] = {
	{ 0, 1, 0, 0, { 1, 1, 1, 1 } }, { 0, 0, 1, 1, { 0, 0, 0, 0 } },
	{ 0, 2, 2, 2, { 2, 2, 2, 2 } }, { 0, 2, 2, 2, { 2, 2, 2, 2 } },
	{ 0, 2, 2, 2, { 2, 2, 2, 2 } }, { 0, 2, 2, 2, { 2, 2, 2, 2 } },
	{ 0, 2, 2, 2, { 2, 2, 2, 2 } }, { 0, 2, 2, 2, { 2, 2, 2, 2 } },
	{ 0, 2, 2, 2, { 2, 2, 2, 2 } }
};

int set_scenario(Config setting_sen) {

	if (setting_sen.temp_max != 0)
	{
		conf.temp_max = setting_sen.temp_max;
	}
	if (setting_sen.temp_play_voice != 2) {
		conf.temp_play_voice = setting_sen.temp_play_voice;
	}
	if (setting_sen.sec_active)
	{
		conf.sec_active = setting_sen.sec_active;
	}
	if (setting_sen.sec_play_voice != 2)
	{
		conf.sec_play_voice = setting_sen.sec_play_voice;
	}
	for (int i = 0; i < 4 ; i++)
	{
		if (setting_sen.light[i] != 2)
		{
			conf.light[i] = setting_sen.light[i];
			HAL_GPIO_WritePin(GPIOE, LED[i], conf.light[i]);
		}
	}
	return 1;
}

int scenario_edite(int key,Config * edditing_sen)
{
	if (key == 'A')
	{
		num = 0;
		return 1;
	}
	if (key == 'B')
	{
		edditing_sen->temp_max = num;
		return 0;
	}
	if (key == 'C')
	{
		num = 0;
		return 0;
	}
	if (key == 'D')
	{
		if (pos == 0)
		{
			edditing_sen->temp_play_voice = (edditing_sen->temp_play_voice + 1) % 3;
			return 0;
		}
		if (pos == 1)
		{
			edditing_sen->sec_active = (edditing_sen->sec_active + 1) % 3;
			return 0;
		}
		if (pos == 2)
		{
			edditing_sen->sec_play_voice = (edditing_sen->sec_play_voice + 1) % 3;
			return 0;
		}
		if (pos >= 3 && pos <= 6)
		{
			edditing_sen->light[pos - 3] = (edditing_sen->light[pos - 3] + 1) % 3;
			return 0;
		}
	}
	if (key >= 0 && key <= 9)
	{
		num = num * 10 + key;
		return 0;
	}
	if (key == '*')
	{
		pos = (pos - 1) % 7;
		return 0;
	}
	if (key == '#')
	{
		pos = (pos + 1) % 7;
		return 0;
	}
	return 0;
}
