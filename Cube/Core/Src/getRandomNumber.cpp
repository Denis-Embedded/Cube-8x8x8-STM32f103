/*
   Возвращает случайное число в диапазоне (int min, int max)

 1. Объявить функцию перед int main:

 	 int getRandomNumber(int min, int max);

 2. Добавить include в main.h (блок USER CODE BEGIN Includes):

 		// инклуды для работы со случайными числами
	#include <cstdlib> 	// для функций rand() и srand()
	#include <ctime> 	// для функции time().


 3. Перед циклом while вызвать функцию начального числа ГСПЧ,
  	  которая установит это число в зависимости от прошедших секунд
  	  с полуночи 1 января 1970 года:

  	  srand(static_cast<unsigned int>(time(0)));

  	  Примечание.
  	  Подход с полуночи 1 января 1970 года некорректно работает в случае с МК. Возникает переполнение
  	  32-х разрядной переменной. В случае ПК, вероятно, бдует работать корректно.
  	  Как вариант - использовать в качестве стартового числа для ГСПЧ считывание наводки с АЦП какого-либо канала.
	  Это реализовано в других примерах.
	  Либо ипользовать текущее время в мс, если в проекте используется RTC

 */


//#include "main.h"
#include "cstdlib"
// Генерируем рандомное число между значениями min и max
// Предполагается, что функцию srand() уже вызывали
int getRandomNumber(int min, int max)
{
    // Равномерно распределяем рандомное число в нашем диапазоне
    return (rand() % (max - min + 1)) + min;
}


