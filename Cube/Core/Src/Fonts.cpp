/*
 * Fonts.cpp
 *
 *  Created on: 8 дек. 2020 г.
 *      Author: bw90
 */
#include "Cube.h"


uint8_t Cube::getFont(uint8_t font, uint8_t layer)
{
    font = font - '0' + 16;			//перевод символа из таблицы ASCII в номер согласно нумерации массива
    if (font < 126)
    {
	return fontBIN[font][7 - layer];	// для английских букв и символов
    }
    else
    {
	return fontBIN[font - 65][7 - layer];		// для русских букв и символов (смещение -65 по массиву)
    }
}


