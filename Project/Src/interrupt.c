#include "interrupt.h"
#include "model.h"

// Один шаг модели ДПТ с силовым преобразователем. Вызывается из прерывания TIM1
void Motor_Step()
{
    Epr = Uyac*K1pr + Epr*K2pr;                 // Преобразователь
    i_yac = (Epr - c*w_dv)*K1yac + i_yac*K2yac; // Якорная цепь
    w_dv = w_dv + (i_yac*c - Mc)*KJe;           // Механика
}

// ПИ-регулятор скорости с фильтром задания на входе
static void Speed_Controller()
{
    w_ref_f = w_ref_f + (w_ref - w_ref_f) * Kf_w; // Фильтр задания
    dw = w_ref_f - Kos*w_dv;
    OutP_w = dw * Kpro_w;
    float temp_OutI_w = OutI_w + OutP_w * Kint_w;
    float temp_Out_w = OutP_w + temp_OutI_w;

    // Условное интегрирование (защита от интегрального насыщения)
    if (temp_Out_w > I_max)
    {
        Out_w = I_max;
        if (dw < 0) OutI_w = temp_OutI_w;
    }
    else if (temp_Out_w < -I_max)
    {
        Out_w = -I_max;
        if (dw > 0) OutI_w = temp_OutI_w;
    }
    else
    {
        Out_w = temp_Out_w;
        OutI_w = temp_OutI_w;
    }

    if (Mode == MODE_SPEED) i_ref = Out_w;
}

// ПИ-регулятор тока
static void Current_Controller()
{
    di = i_ref - Kot*i_yac;
    OutP_i = di * Kpro_i;
    float temp_OutI_i = OutI_i + OutP_i * Kint_i;
    float temp_Out_i = OutP_i + temp_OutI_i;

    // Ограничение напряжением питания преобразователя
    // и условное интегрирование (защита от интегрального насыщения)
    if (temp_Out_i > U_max)
    {
        Out_i = U_max;
        if (di < 0) OutI_i = temp_OutI_i;
    }
    else if (temp_Out_i < -U_max)
    {
        Out_i = -U_max;
        if (di > 0) OutI_i = temp_OutI_i;
    }
    else
    {
        Out_i = temp_Out_i;
        OutI_i = temp_OutI_i;
    }

    if (Mode != MODE_OPEN_LOOP) Uyac = Out_i;
}

// Регуляторы. Вызывается из прерывания TIM2.
// Регулятор скорости считается каждый REG_W_DIV-й вызов и перед регулятором тока,
// чтобы новое задание тока сразу попало в регулятор тока
void Controllers_Step()
{
    static uint8_t Counter_w = 0;

    if (++Counter_w >= REG_W_DIV)
    {
        Counter_w = 0;
        Speed_Controller();
    }

    Current_Controller();
}

// Задатчик: смена режима, задание и момент нагрузки. Вызывается из прерывания TIM3
void Reference_Step()
{
    // Смена режима из MCUViewer: записать номер режима в Mode_Request
    if (Mode_Request != Mode)
    {
        if (Mode_Request < MODE_COUNT) Set_Mode((Mode_Type)Mode_Request);
        else Mode_Request = Mode; // Неверный номер режима - запрос отклоняется
    }

    // Смена знака задания каждые Period тиков
    if (++Counter >= Period)
    {
        Counter = 0;
        Ref_Sign = -Ref_Sign;
    }

    float ref = Ref_Sign * Modes[Mode].amplitude;
    switch (Mode)
    {
    case MODE_OPEN_LOOP: Uyac = ref; break;
    case MODE_CURRENT:   i_ref = ref; break;
    case MODE_SPEED:     w_ref = ref; break;
    default: break;
    }

    // Во второй половине полупериода - наброс нагрузки того же знака, что и задание
    if (Counter >= Period/2) Mc = Ref_Sign * Mload;
    else Mc = 0.0f;
}
