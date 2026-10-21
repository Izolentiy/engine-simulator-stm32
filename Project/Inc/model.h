#pragma once
#include "stm32f1xx_hal.h"

#define REG_W_DIV 2 // Регулятор скорости считается каждый N-й вызов регулятора тока

typedef enum
{
    MODE_OPEN_LOOP = 0, // Разомкнутая система, задание - напряжение Uyac
    MODE_CURRENT   = 1, // Контур тока, задание - ток i_ref
    MODE_SPEED     = 2, // Двухконтурная САУ, задание - скорость w_ref
    MODE_COUNT          // Количество режимов, не режим
} Mode_Type;

// Параметры задания для одного режима
typedef struct
{
    float half_period; // Время между сменами знака задания, с
    float amplitude;   // Модуль задания: В для Uyac, А для i_ref, рад/с для w_ref
} Mode_Config;

extern const Mode_Config Modes[MODE_COUNT];

extern volatile uint16_t Mode_Request; // Записывается из MCUViewer для смены режима
extern volatile float Mload;
extern volatile float I_max;
extern volatile float U_max;

extern uint16_t Mode, Period, Counter;
extern float dt, Kpr, Tpr, Ryac, Tyac, Je, c, K1pr, K2pr, Epr, K1yac, K2yac, i_yac, KJe, w_dv, Uyac, Mc;
extern float Kot, Kpro_i, Kint_i, Kos, Kpro_w, Kint_w, OutP_i, OutI_i, Out_i, OutP_w, OutI_w, Out_w, Ti_i, Ti_w, dt_i, dt_w, dt_ref, i_ref, w_ref, di, dw;
extern float w_ref_f, Kf_w;
extern float Tmu_i, Tmu_w;
extern float Ref_Sign;

void Set_Mode(Mode_Type mode);
void Init_Variables();
