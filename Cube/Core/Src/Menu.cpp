/*
 * Menu.cpp
 *
 *  Created on: May 10, 2021
 *      Author: Арсений
 */
#include "Menu.h"


void SwitchingModes()
{

    if ((IsButtonPressed.IsButtonPressedRx >> LB) & 1)
    {
	cube.incCurrentMode();
	IsButtonPressed.IsButtonPressedRx &= ~(1 << LB);
    }
    if ((IsButtonPressed.IsButtonPressedRx >> LU) & 1)
    {
	cube.decCurrentMode();
	IsButtonPressed.IsButtonPressedRx &= ~(1 << LU);
    }
}



void ChangingSpeed()
{
    if ((IsButtonPressed.IsButtonPressedRx >> LL) & 1)
    {
	cube.changeMode();
	uint8_t TxVar = cube.getMode();
	//HAL_UART_Transmit(&huart3, &TxVar, 1, 0xFF);
	IsButtonPressed.IsButtonPressedRx &= ~(1 << LL);
    }
    /*
     * Чтобы не сбить таймер в режиме, пропускаем инкремент скорости
     * и на всякий случай сбрасываем флаги "кнопок"
     */
    if(cube.getMode() == CONST_CHANGE_BRIGHTNESS)
    {
	IsButtonPressed.IsButtonPressedRx &= ~(1 << RB);
	IsButtonPressed.IsButtonPressedRx &= ~(1 << RU);
	return;
    }

    if ((IsButtonPressed.IsButtonPressedRx >> RB) & 1)
    {
	cube.setSpeed(cube.getSpeed() + 10);
	if (cube.getSpeed() > 1000)
	{
	    cube.setSpeed(1000);
	}
	IsButtonPressed.IsButtonPressedRx &= ~(1 << RB);
    }

    if ((IsButtonPressed.IsButtonPressedRx >> RU) & 1)
    {
	cube.setSpeed(cube.getSpeed() - 10);
	if (cube.getSpeed() < 0)
	{
	    cube.setSpeed(0);
	}
	IsButtonPressed.IsButtonPressedRx &= ~(1 << RU);
    }

}

