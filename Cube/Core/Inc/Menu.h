/*
 * Menu.h
 *
 *  Created on: May 10, 2021
 *      Author: Арсений
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_


#include "main.h"
extern Cube cube;
extern FlagUnion IsButtonPressed;
extern UART_HandleTypeDef huart3;

void SwitchingModes();
void ChangingSpeed();

#endif /* INC_MENU_H_ */
