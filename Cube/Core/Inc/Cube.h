/*
 * Cube.h
 *
 *  Created on: Nov 14, 2020
 *      Author: bw90
 */

#ifndef INC_CUBE_H_
#define INC_CUBE_H_


#define TOTAL_MODES 8
#include "stm32f1xx_hal.h"

#include "CubeFonts.h"
//#include "main.h"
#include "getRandomNumber.h"

enum Axis
{
	XAXIS = 0,
	YAXIS,
	ZAXIS
};

enum Direction
{
	POS_X = 0,
	NEG_X,
	POS_Y,
	NEG_Y,
	POS_Z,
	NEG_Z
};
enum Bright_Vector
{
	L2H = 0,			//Low to high
	H2L
};

enum Modes				//TODO: обновить заголовочник в файле для куба
{
	RAIN = 0,
	WALL,
	WOOP_WOOP,
	CUBE_JUMP,
	TEXT,
	LIGHTCUBE,
	WALKINGCUBE,
	CONST_CHANGE_BRIGHTNESS
};

enum Timers
{
	RAIN_TIME = 		260,
	WALL_TIME = 		220,
	WOOP_WOOP_TIME = 	350,
	CUBE_JUMP_TIME = 	200,
	TEXT_TIME = 		300,
	CLOCK_TIME = 		500, //У режима Lit нет времени, но поле необходимо для удобной записи
	WALKING_TIME = 		100,
	BRIGHT_TIME = 		0
};

struct String				//Структура для режима текста
{
	char *stringPtr;
	uint8_t lengh;
	uint8_t charCounter;		//номер текущего символа
	uint8_t charPosition;		//координата символа в кубе по Y(нужно для формирования символа)
};

class Cube
{
	public:

	void printFrame();													//метод для отрисовки куба
	void setDot(uint8_t x, uint8_t y, uint8_t z, uint8_t set_num);		//метод для установки определенного светодиода


	void changeMode();
	void incCurrentMode();
	void decCurrentMode();



	void text();								//метод для бегущих букв
	void clearCube();							//потушить все светодиоды
	void lightCube();							//зажечь все светодиоды
	void walkingCube();							//перемещающийся по осям куб
	void rain();								//Дождь
	void walk_and_flying_Wall();						//Стена, перемещающаяся по случайным осям
	void woopWoop();							//Расширяющийся и сужающийся куб
	void cubeJump();
	void constChangeBrightnes();

	void setTextString(char *text, uint8_t lengh);
	void setSpeed(int16_t timer);
	int32_t getSpeed();		//возможно временное решение для изменения инкремента "на лету"

	typedef void (Cube::*ModePtr)();			//тип указателя, который указывает на метод текущего режима, который должен вызываться в обработчике таймера
	ModePtr m_currentModePtr = &Cube::lightCube;		//по умолчанию указывает на режим светящегося куба

	Modes getMode();					//геттер для режима
	private:

	void setWall(Axis axis, uint8_t i);
	void drawCube(uint8_t x, uint8_t y, uint8_t z, uint8_t s);
	void shift(Direction dir);

	uint8_t m_OutFrame[8][8] =
		{
			{0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
			{0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
			{0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
			{0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
			{0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
			{0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
			{0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
			{0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111},
		};		//[z][x], y - номер бита (отсчет слева направо)
	Modes m_currentMode = LIGHTCUBE;
	uint8_t getFont(uint8_t font, uint8_t layer);
	uint8_t m_IsModeSwitched = 1;		//Переменная-статус, показывающая факт того, что режим был переключен

	/*----------------------------Ходячий куб------------------------------*/
	int16_t m_coord[3];		//координаты
	int8_t m_vector[3];		//вектора
	/*----------------------------Ходячий куб------------------------------*/

	/*----------------------------Стена---------------------------------*/
	Direction wallDirection;
	uint8_t	wallPosition = 0;
	uint8_t isLooped = 0;
	/*----------------------------Стена---------------------------------*/

	/*---------------------------woopWoop-------------------------------*/
	uint8_t cubeSize;
	uint8_t cubeExpanding = 0;
	/*---------------------------woopWoop-------------------------------*/

	/*----------------------------Прыгающй куб--------------------------*/
	uint8_t xPos;
	uint8_t yPos;
	uint8_t zPos;
	/*----------------------------Прыгающй куб--------------------------*/

	String Text;				//Структура с текстом


	/*-----------------------таймеры------------------------------------*/
	uint32_t m_lastTrigger;	//Время последнего "кадра" режима
	int32_t m_modeTimer = TEXT_TIME;	//период кадров в режиме
	/*-----------------------таймеры------------------------------------*/

	/*-----------------------Меняющаяся яркость-------------------------*/

	uint16_t brightness = 0;
	Bright_Vector vector = L2H;

	//uint8_t m_killADC = 0;     //Возможно не нужно, потом надо убрать
	/*-----------------------Меняющаяся яркость-------------------------*/
};



#endif /* INC_CUBE_H_ */
