/*
 * Menu.c
 *
 *  Created on: Apr 21, 2021
 *      Author: КомпудахтерPC148823D
 */

#include "Menu.h"
#include "ILI9225.h"
#include "main.h"
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;
extern FlagUnion IsButtonPressed;
uint8_t IsModeSwithced = 1;
enum Modes CurrentMode = WALKINGCUBE;  //Режим по умолчанию - ходячий куб
const uint16_t Timers_cube[TOTAL_MODES] =
	{
		260, 220, 350, 200, 300, 500, 100, 8
	};

uint16_t Speed = 100;
char ModesStrings[][30] =
	{
		"Rain", "Wall", "Woop-woop", "Jumping cube", "Text", "Lit cube", "Walking cube", "Fading", "Clear Cube"
	};

void SwitchingModes()
{

    if (IsModeSwithced)
    {
	IsModeSwithced = 0;
//	fill_rectangle(0, 0, 220, 164, COLOR_RED); не работает

	for (int8_t j = TOTAL_MODES/2; j > -(TOTAL_MODES/2); --j)
	{
	    char str1[13] = "";	//строка для вывода
	    char str2[13] = "";	//вспомогательная строка
	    uint8_t i;
	    int8_t SubCurrentMode = (int8_t)CurrentMode + j < 0 ? (TOTAL_MODES + (int8_t)CurrentMode + j) % (TOTAL_MODES) : ((int8_t)CurrentMode + j) % (TOTAL_MODES);
	    for (i = 0; i < ((13 - strlen(ModesStrings[SubCurrentMode])) / 2); ++i)
	    {
		str1[i] = ' ';
		str2[i] = ' ';
	    }
	    str1[i + 1] = '\0';
	    strcat(str1, ModesStrings[SubCurrentMode]);
	    strcat(str1, str2);
	    draw_string(60, 83 + 10 * -j, str1, Font_7x10, COLOR_WHITE, COLOR_RED);


	}
	draw_char(50, 83, '>', Font_7x10, COLOR_WHITE, COLOR_RED);
	draw_char(153, 83, '<', Font_7x10, COLOR_WHITE, COLOR_RED);


	draw_bitmap(0, 103, 7, 10, Up_PTR);
	draw_string(8, 103, "LU", Font_7x10, COLOR_WHITE, COLOR_RED);
	draw_bitmap(0, 63, 7, 10, Down_PTR);
	draw_string(8, 63, "LB", Font_7x10, COLOR_WHITE, COLOR_RED);
    }

    if ((IsButtonPressed.IsButtonPressedRx >> LB) & 1)
    {
	IsButtonPressed.IsButtonPressedRx &= ~(1 << LB);
	CurrentMode = ((int8_t)CurrentMode + 1) % TOTAL_MODES; //учет переполнения сверху
	Speed = Timers_cube[CurrentMode];
	IsModeSwithced = 1;
    }
    if ((IsButtonPressed.IsButtonPressedRx >> LU) & 1)
    {
	IsButtonPressed.IsButtonPressedRx &= ~(1 << LU);
	CurrentMode = (int8_t) CurrentMode - 1 < 0 ?
		       TOTAL_MODES - 1 : (int8_t) CurrentMode - 1; //учет переполнения снизу
	Speed = Timers_cube[CurrentMode];
	IsModeSwithced = 1;				//Перерисовываем экран
    }

}


void ShowingCurrentMode()
{
//    static uint8_t IsFirstIt = 1;
    if (IsModeSwithced)
    {

	IsModeSwithced = 0;
	//fill_rectangle(0, 0, 220, 164, COLOR_RED);
	//HAL_Delay(100);
	char str[] = "Current mode:     Speed:";
	draw_string(20, 83, str, Font_7x10, COLOR_WHITE, COLOR_RED);
	sprintf(str, "Press LL for synchro");
	draw_string(20, 20, str, Font_7x10, COLOR_WHITE, COLOR_RED);

    }
    /*---------------Выравнивание по центру(Режим)---------------------*/
    char str[30] = "";
    char str1[20] = ""; //вспомогательная строка для выравнивания
    uint8_t i;
    for (i = 0; i < ((13 - strlen(ModesStrings[CurrentMode])) / 2); ++i)
    {
	str[i] = ' ';
	str1[i] = ' ';
    }
    str[i + 1] = '\0';
    strcat(str, ModesStrings[CurrentMode]);
    strcat(str, str1);
    draw_string(20, 73, str, Font_7x10, COLOR_WHITE, COLOR_RED);
    /*---------------Выравнивание по центру(Режим)---------------------*/

    /*---------------Отображение скорости(без выравнивания)------------------*/
    if (CurrentMode == LIGHTCUBE || CurrentMode == CONST_CHANGE_BRIGHTNESS)
    {
	sprintf(str, "N/A");

    }
    else
    {
	sprintf(str, "%d     ", Speed);
    }
    draw_string(153, 73, str, Font_7x10, COLOR_WHITE, COLOR_RED);
    /*---------------Отображение скорости(без выравнивания)------------------*/

    /*---------------Обработка кнопок(изменение скорости)------------------*/
    if ((IsButtonPressed.IsButtonPressedRx >> RB) & 1)
    {
	IsButtonPressed.IsButtonPressedRx &= ~(1 << RB);

	Speed += 10;
	if (Speed > 1000)
	{
	    Speed = 1000;
	}

    }
    if ((IsButtonPressed.IsButtonPressedRx >> RU) & 1)
    {
	IsButtonPressed.IsButtonPressedRx &= ~(1 << RU);

	Speed = (int16_t)Speed - 10 < 0? 0 : Speed - 10;
    }
    /*---------------Обработка кнопок(изменение скорости)------------------*/
}
