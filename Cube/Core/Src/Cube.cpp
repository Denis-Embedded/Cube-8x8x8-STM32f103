/*
 * Cube.cpp
 *
 *  Created on: Nov 14, 2020
 *      Author: bw90
 */
#include "Cube.h"
//#include "getRandomNumber.h"
#define RESETSERCLK GPIOA -> BSRR |= 1 << (2 + 16)
#define SETSERCLK GPIOA -> BSRR |= 1 << 2

#define RESETRCLK GPIOA -> BSRR |= 1 << (16 + 1)
#define SETRCLK GPIOA -> BSRR |= 1 << 1

void Cube::printFrame()
{
	for (uint8_t z = 0; z < 8; ++z)
	{
		uint8_t currentLayer = ~(128 >> (7 - z));

		RESETRCLK;

		for (uint8_t i = 0; i < 8; ++i) 							//Посылаем байт с текущим слоем в сдвиговый регистр
		{
			RESETSERCLK;
			GPIOA -> BSRR |= 1 << (16 * !((currentLayer >> i) & 1));
			SETSERCLK;
		}

		for (uint8_t x = 0; x < 8; ++x)
		{
			for (uint8_t y = 0; y < 8; ++y)
			{
				RESETSERCLK;
				GPIOA -> BSRR |= 1 << (16 * ((m_OutFrame[7 - z][x] >> y) & 1 ));
				SETSERCLK;
			}
		}


		SETRCLK;
		//HAL_Delay(1);
	}
}

void Cube::setSpeed(int16_t timer)
{
    m_modeTimer = timer;
}

int32_t Cube::getSpeed()
{
    return m_modeTimer;
}

Modes Cube::getMode()
{
    return m_currentMode;
}

void Cube::setDot(uint8_t x, uint8_t y, uint8_t z, uint8_t set_num)
{
    m_OutFrame[7 - z][x] &= ~(128 >> y); 							//сбрасываем бит
    m_OutFrame[7 - z][x] |= (128 >> y) * set_num; 						//устанавливаем 0 или 1 в зависимости от set_num
}



void Cube::changeMode()					//метод для переключения режима в зависимости от m_currentMode
{
	clearCube();
	m_IsModeSwitched = 1;
	m_lastTrigger = HAL_GetTick();

	ADC2->CR1 |= 1 << 5; //включаем ацп										//всегда устанавливаем бит в 1
																//А обнуляем только если текущий режим
																//CONST_CHANGE_BRIGHTNESS

	switch(m_currentMode)
	{
		case CONST_CHANGE_BRIGHTNESS:
			m_modeTimer = BRIGHT_TIME;
			ADC2->CR1 &= ~(1 << 5);	//выключаем ацп, если измерения не нужны							//обнуление бита EOCIE(End Of Conversion Int Enable)
																//для того чтобы ацп не перебивал режим
			m_currentModePtr = &Cube::constChangeBrightnes;
			break;

		case RAIN:
			m_modeTimer = RAIN_TIME;
			m_currentModePtr = &Cube::rain;
			break;

		case WALL:
			m_modeTimer = WALL_TIME;
			m_currentModePtr = &Cube::walk_and_flying_Wall;
			break;

		case WOOP_WOOP:
			m_modeTimer = WOOP_WOOP_TIME;
			m_currentModePtr = &Cube::woopWoop;
			break;

		case CUBE_JUMP:
			m_modeTimer = CUBE_JUMP_TIME;
			m_currentModePtr = &Cube::cubeJump;
			break;

		case TEXT:
			m_modeTimer = TEXT_TIME;
			m_currentModePtr = &Cube::text;
			break;

		case LIGHTCUBE:
			m_currentModePtr = &Cube::lightCube;
			break;



		case WALKINGCUBE:
			m_modeTimer = WALKING_TIME;
			m_currentModePtr = &Cube::walkingCube;
			break;

	}

}

void Cube::incCurrentMode()
{
	m_currentMode = static_cast<Modes>(static_cast<uint8_t>(m_currentMode) + 1);
    if (m_currentMode > TOTAL_MODES - 1)
    {
    	m_currentMode = RAIN;
    }
}

void Cube::decCurrentMode()
{
	m_currentMode = static_cast<Modes>(static_cast<uint8_t>(m_currentMode) - 1);
    if (m_currentMode > TOTAL_MODES - 1)
    {
	m_currentMode = static_cast<Modes>(TOTAL_MODES - 1); 		//защита от OutOfBounds
    }
}

void Cube::clearCube()									//очистка куба
{
    for(uint8_t x = 0; x < 8; ++x)
    {
    	for(uint8_t z = 0; z < 8; ++z)
    	{
    		m_OutFrame[z][x] = 0;
    	}
    }
}

void Cube::lightCube()									//Горящий куб
{
    if(m_IsModeSwitched)
    {
    	for(uint8_t x = 0; x < 8; ++x)
    	{
    		for(uint8_t z = 0; z < 8; ++z)
    		{
    			m_OutFrame[z][x] = 0b11111111;
    		}
    	}
    	m_IsModeSwitched = 0;
    }
}

void Cube::constChangeBrightnes()
{
	/*---------------------Setup----------------------------*/
	if (m_IsModeSwitched)
	{
		lightCube();
		brightness = 0;
		vector = L2H;
	}
	/*---------------------Setup----------------------------*/

	/*-----------------------Main loop----------------------*/
	/*
	 * Таймер нужен только для того, чтобы ярксоть менялась более плавно
	 * в момент когда идет смена векторов
	 */
	if (HAL_GetTick() - m_lastTrigger > m_modeTimer)
	{
		m_lastTrigger = HAL_GetTick();
		switch(vector)
		{
			case L2H:
			    ++brightness;//if(++brightness == 4095)
			break;

			case H2L:
			    --brightness;
			break;
		}

		if(brightness == 0 && vector == H2L)
		{
		    vector = L2H;
		    m_lastTrigger += 1000;
		}

		if (brightness == 4094 && vector == L2H)
		{
		    vector = H2L;
		}

		TIM2->CCR4 = brightness;
	}
	/*-----------------------Main loop----------------------*/
}

void Cube::text()
{
	/*---------------------Setup----------------------------*/
	if (m_IsModeSwitched)
	{
		if(Text.stringPtr == nullptr)
		{
		    return;				//если строка null, выходим
		}
		m_IsModeSwitched = 0;
		clearCube();
		Text.charPosition = -1;
		Text.charCounter = 0;
	}
	/*---------------------Setup----------------------------*/

	/*-----------------------Main loop----------------------*/
	if (HAL_GetTick() - m_lastTrigger > m_modeTimer)
	{
		m_lastTrigger = HAL_GetTick();

		shift(POS_X);
		if(++Text.charPosition == 7)
		{

			if(++Text.charCounter > Text.lengh - 1)
			{
				Text.charCounter = 0;
			}
			Text.charPosition = 0;
		}
		if(Text.charPosition == 0)
		{
			for(uint8_t i = 0; i < 8; ++i)
			{
				m_OutFrame[i][0] = getFont(Text.stringPtr[Text.charCounter], i);
			}
		}
	}
	/*-----------------------Main loop----------------------*/
}

void Cube::setTextString(char *text, uint8_t lengh)
{
	Text.stringPtr = text;
	Text.lengh = lengh;
}


void Cube::walkingCube()								//Летающий куб внутри пространства
{
	/*---------------------Setup----------------------------*/
	if (m_IsModeSwitched)
	{
		clearCube();
		m_IsModeSwitched = 0;
		m_modeTimer = WALKING_TIME;
		for (uint8_t i = 0; i < 3; ++i)
		{
			m_coord[i] = 300;							//300, чтобы не использовать долгие вычисления с плавающей точкой
			m_vector[i] = getRandomNumber(3, 7) * 15;	//В f103 нет блока FPU
		}
		m_lastTrigger = HAL_GetTick();
	}
	/*---------------------Setup----------------------------*/


	/*-----------------------Main loop----------------------*/
	if (HAL_GetTick() - m_lastTrigger > m_modeTimer)
	{
		m_lastTrigger = HAL_GetTick();

		for(uint8_t i = 0; i < 3; ++i)
		{
			m_coord[i] += m_vector[i];
			if (m_coord[i] <= 1)				//Защита от отрицательных координат и векторов
			{
				m_coord[i] = 1;
				m_vector[i] = -m_vector[i];
				m_vector[i] += getRandomNumber(0, 5) - 3;
			}

			if (m_coord[i] >= (700 - 100))
			{
				m_coord[i] = (700 - 100);
				m_vector[i] = -m_vector[i];
				m_vector[i] += getRandomNumber(0, 5) - 3;
			}
		}
		clearCube();
		int8_t thisX = m_coord[0] / 100;
		int8_t thisY = m_coord[1] / 100;
		int8_t thisZ = m_coord[2] / 100;

		setDot(thisX, 		thisY, 		thisZ, 		1);
		setDot(thisX + 1, 	thisY, 		thisZ, 		1);
		setDot(thisX, 		thisY + 1, 	thisZ, 		1);
		setDot(thisX, 		thisY, 		thisZ + 1, 	1);
		setDot(thisX + 1, 	thisY + 1, 	thisZ, 		1);
		setDot(thisX, 		thisY + 1, 	thisZ + 1, 	1);
		setDot(thisX + 1, 	thisY, 		thisZ + 1, 	1);
		setDot(thisX + 1, 	thisY + 1, 	thisZ + 1, 	1);
	}
	/*-----------------------Main loop----------------------*/
}

void Cube::rain()											//Режим дождя
{
	/*---------------------Setup----------------------------*/
	if (m_IsModeSwitched)
	{
		clearCube();
		m_IsModeSwitched = 0;
		m_lastTrigger = HAL_GetTick();
	}
	/*---------------------Setup----------------------------*/

	/*-----------------------Main loop----------------------*/
	if (HAL_GetTick() - m_lastTrigger > m_modeTimer)
	{
		m_lastTrigger = HAL_GetTick();
		shift(NEG_Y);
		uint8_t numDrops = getRandomNumber(0, 4);
		for (uint8_t i = 0; i < numDrops; ++i)
		{
			setDot(getRandomNumber(0, 7), getRandomNumber(0, 7), 7, 1);
		}

	}
	/*-----------------------Main loop----------------------*/
}


void Cube::walk_and_flying_Wall()						//режим, в котором есть двигающаяся по случайной оси "стена"
{

	/*---------------------Setup----------------------------*/
	if (m_IsModeSwitched)
	{

	    clearCube();
		uint8_t axis = getRandomNumber(0, 2);
		wallPosition = getRandomNumber(0, 1) * 7;
		setWall(static_cast<Axis>(axis), wallPosition);
		switch(static_cast<Axis>(axis))
		{
			case XAXIS:
				if(wallPosition == 0)
				{
					wallDirection = POS_X;
				}
				else
				{
					wallDirection = NEG_X;
				}
				break;


			case YAXIS:
				if(wallPosition == 0)
				{
					wallDirection = POS_Y;
				}
				else
				{
					wallDirection = NEG_Y;
				}
				break;


			case ZAXIS:
				if(wallPosition == 0)
				{
					wallDirection = POS_Z;
				}
				else
				{
					wallDirection = NEG_Z;
				}
				break;
		}
		isLooped = 0;
		m_IsModeSwitched = 0;
		m_lastTrigger = HAL_GetTick();
	}
	/*---------------------Setup----------------------------*/

	/*-----------------------Main loop----------------------*/
	if (HAL_GetTick() - m_lastTrigger > m_modeTimer)
	{
		m_lastTrigger = HAL_GetTick();
		shift(wallDirection);
		if (wallDirection % 2 == 0)
		{

			if (++wallPosition == 7)
			{
				if (isLooped)
				{
					m_IsModeSwitched = 1;			//когда достигло края, перезагружаем режим
				}
				else
				{
					wallDirection = static_cast<Direction>(static_cast<uint8_t>(wallDirection) + 1);  //Это просто работает

					isLooped = 1;
				}
			}
		}
		else
		{

			if (--wallPosition == 0)
			{
				if (isLooped)
				{
					m_IsModeSwitched = 1;
				}
				else
				{
					wallDirection = static_cast<Direction>(static_cast<uint8_t>(wallDirection) - 1);
					isLooped = 1;
				}
			}
		}
	}
	/*-----------------------Main loop----------------------*/
}



void Cube::woopWoop()
{
	/*---------------------Setup----------------------------*/
	if (m_IsModeSwitched)
	{
		m_modeTimer = WOOP_WOOP_TIME;
		clearCube();
		cubeSize = 2;
		cubeExpanding = 1;
		m_IsModeSwitched = 0;
	}
	/*---------------------Setup----------------------------*/

	/*-----------------------Main loop----------------------*/
	if (HAL_GetTick() - m_lastTrigger > m_modeTimer)
	{
		m_lastTrigger = HAL_GetTick();

		/*-----------переключение из сжимающегося в расширающийся(или обратно)-----------*/
		if (cubeExpanding)
		{
			cubeSize += 2;
			if (cubeSize == 8)
			{
				cubeExpanding = 0;
			}
		}
		else
		{
			cubeSize -= 2;
			if (cubeSize == 2)
			{
				cubeExpanding = 1;
			}

		}
		/*-----------переключение из сжимающегося в расширающийся(или обратно)-----------*/

		clearCube();
		drawCube(4 - cubeSize / 2, 4 - cubeSize / 2, 4 - cubeSize / 2, cubeSize);
	}
	/*-----------------------Main loop----------------------*/
}

void Cube::cubeJump()
{
	/*---------------------Setup----------------------------*/
	if (m_IsModeSwitched)
	{
		m_IsModeSwitched = 0;
		clearCube();
		xPos = getRandomNumber(0, 1) * 7;
		yPos = getRandomNumber(0, 1) * 7;
		zPos = getRandomNumber(0, 1) * 7;
		cubeSize = 8;
		cubeExpanding = 0;
	}
	/*---------------------Setup----------------------------*/

	/*-----------------------Main loop----------------------*/
	if (HAL_GetTick() - m_lastTrigger > m_modeTimer)
	{
		m_lastTrigger = HAL_GetTick();
		clearCube();
		/*--------------------------Определение координат угла--------*/
		if (xPos == 0 && yPos == 0 && zPos == 0)
		{
			drawCube(xPos, yPos, zPos, cubeSize);
		}
		else if (xPos == 7 && yPos == 7 && zPos == 7)
		{
			drawCube(xPos + 1 - cubeSize, yPos + 1 - cubeSize, zPos + 1 - cubeSize, cubeSize);
		}
		else if (xPos == 7 && yPos == 0 && zPos == 0)
		{
			drawCube(xPos + 1 - cubeSize, yPos, zPos, cubeSize);
		}
		else if (xPos == 0 && yPos == 7 && zPos == 0)
		{
		    drawCube(xPos, yPos + 1 - cubeSize, zPos, cubeSize);
		}
		else if (xPos == 0 && yPos == 0 && zPos == 7)
		{
		    drawCube(xPos, yPos, zPos + 1 - cubeSize, cubeSize);
		}
		else if (xPos == 7 && yPos == 7 && zPos == 0)
		{
		    drawCube(xPos + 1 - cubeSize, yPos + 1 - cubeSize, zPos, cubeSize);
		}
		else if (xPos == 0 && yPos == 7 && zPos == 7)
		{
		    drawCube(xPos, yPos + 1 - cubeSize, zPos + 1 - cubeSize, cubeSize);
		}
		else if (xPos == 7 && yPos == 0 && zPos == 7)
		{
		    drawCube(xPos + 1 - cubeSize, yPos, zPos + 1 - cubeSize, cubeSize);
		}
		/*--------------------------Определение координат угла--------*/

		if(cubeExpanding)
		{
			if (++cubeSize == 8)
			{
				cubeExpanding = 0;
				xPos = getRandomNumber(0, 1) * 7;
				yPos = getRandomNumber(0, 1) * 7;
				zPos = getRandomNumber(0, 1) * 7;
			}
		}
		else
		{
			if (--cubeSize == 1)
			{
				cubeExpanding = 1;
			}
		}
	}
	/*-----------------------Main loop----------------------*/
}



void Cube::drawCube(uint8_t x, uint8_t y, uint8_t z, uint8_t s)
{
	for (uint8_t i = 0; i < s; ++i)
	{
		setDot(x, y + i, z, 1);
		setDot(x + i, y, z, 1);
		setDot(x, y, z + i, 1);
		setDot(x + s - 1, y + i, z + s - 1, 1);
		setDot(x + i, y + s - 1, z + s - 1, 1);
		setDot(x + s - 1, y + s - 1, z + i, 1);
		setDot(x + s - 1, y + i, z, 1);
		setDot(x, y + i, z + s - 1, 1);
		setDot(x + i, y + s - 1, z, 1);
		setDot(x + i, y, z + s - 1, 1);
		setDot(x + s - 1, y, z + i, 1);
		setDot(x, y + s - 1, z + i, 1);
	}
}


void Cube::setWall(Axis axis, uint8_t i)			//в зависимости от оси и точки на этой оси, закрашиваем поверхность
{
	for(uint8_t j = 0; j < 8; ++j)
	{
		for(uint8_t k = 0; k < 8; ++k)
		{
			switch(axis)
			{
				case XAXIS:
					setDot(i, j, k, 1);
					break;
				case YAXIS:
					setDot(j, i, k, 1);
					break;
				case ZAXIS:
					setDot(j, k, i, 1);
			}
		}
	}
}





void Cube::shift(Direction dir)
{
	switch(dir)
	{
		case POS_X:													//Каждый столбец начиная со восьмого копирует то, что
																	//находится в левом столбце
			for (uint8_t x = 7; x > 0; --x)
			{
				for (uint8_t z = 0; z < 8; ++z)
				{
					m_OutFrame[z][x] = m_OutFrame[z][x - 1];
				}
			}
			for (uint8_t i = 0; i < 8; ++i)							//Затем все что в самом левом столбце затирается
			{
				m_OutFrame[i][0] = 0;
			}
			break;


		case NEG_X:
			for (uint8_t x = 0; x < 7; ++x)							//Каждый столбец начиная с нулевого копирует то, что
																	//находится в правом столбце
			{
				for (uint8_t z = 0; z < 8; ++z)
				{
					m_OutFrame[z][x] = m_OutFrame[z][x + 1];
				}
			}
			for (uint8_t i = 0; i < 8; ++i)							//Затем все что в самом правом столбце затирается
			{
				m_OutFrame[i][7] = 0;
			}
			break;


		case POS_Y:
			for (uint8_t x = 0; x < 8; ++x)							//Каждая ячейка сдвигатся на 1 вправо
			{
				for (uint8_t z = 0; z < 8; ++z)
				{
					m_OutFrame[z][x] >>= 1;
				}
			}
			break;


		case NEG_Y:
			for (uint8_t x = 0; x < 8; ++x)							//Каждая ячейка сдвигатся на 1 влево
			{
				for (uint8_t z = 0; z < 8; ++z)
				{
					m_OutFrame[z][x] <<= 1;
				}
			}
			break;


		case POS_Z:
			for (uint8_t x = 0; x < 8; ++x)							//Каждая строка, начиная с нулевой копирует то, что
			{														//находится в следующей
				for (uint8_t z = 0; z < 7; ++z)
				{
					m_OutFrame[z][x] = m_OutFrame[z + 1][x];
				}
			}
			for (uint8_t i = 0; i < 8; ++i)							//Затем все что в самой нижней строке затирается
			{
				m_OutFrame[7][i] = 0;
			}
			break;


		case NEG_Z:													//Каждая строка, начиная с седьмой копирует то, что
			for (uint8_t x = 0; x < 8; ++x)							//находится в предыдущей
			{
				for (uint8_t z = 7; z > 0; --z)
				{
					m_OutFrame[z][x] = m_OutFrame[z - 1][x];
				}
			}
			for (uint8_t i = 0; i < 8; ++i)							//Затем все что в самом правом столбце затирается
			{
				m_OutFrame[0][i] = 0;
			}
			break;
	}
}
