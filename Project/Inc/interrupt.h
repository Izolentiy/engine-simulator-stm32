#pragma once

// Функции, которые вызываются из обработчиков прерываний таймеров (stm32f1xx_it.c)

void Motor_Step();       // TIM1: шаг модели ДПТ
void Controllers_Step(); // TIM2: регуляторы скорости и тока
void Reference_Step();   // TIM3: режим, задание и момент нагрузки
