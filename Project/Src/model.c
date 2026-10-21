#include "model.h"
#include "tim.h"

volatile uint16_t Mode_Request; // Можно менять из MCUViewer
volatile float Mload;
volatile float I_max;
volatile float U_max;

uint16_t Mode, Period, Counter;
float dt, Kpr, Tpr, Ryac, Tyac, Je, c, K1pr, K2pr, Epr, K1yac, K2yac, i_yac, KJe, w_dv, Uyac, Mc;
float Kot, Kpro_i, Kint_i, Kos, Kpro_w, Kint_w, OutP_i, OutI_i, Out_i, OutP_w, OutI_w, Out_w, Ti_i, Ti_w, dt_i, dt_w, dt_ref, i_ref, w_ref, di, dw;
float w_ref_f, Kf_w; // Фильтр задания на вход контура скорости
float Tmu_i, Tmu_w; // Малые некомпенсированные постоянные для настройки регуляторов
float Ref_Sign; // Знак задания: +1 или -1, меняется каждые half_period

// Задание для каждого режима, индекс массива - номер режима
const Mode_Config Modes[MODE_COUNT] =
{
    [MODE_OPEN_LOOP] = { .half_period = 3.0f, .amplitude = 24.0f  },
    [MODE_CURRENT]   = { .half_period = 0.2f, .amplitude = 10.0f  },
    [MODE_SPEED]     = { .half_period = 3.0f, .amplitude = 20.0f  },
};

// Период прерывания таймера в секундах по фактическим PSC, ARR и тактированию
static float Timer_Period(TIM_HandleTypeDef *htim)
{
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t pclk = (htim->Instance == TIM1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    uint32_t f_tim = (pclk == hclk) ? pclk : 2*pclk; // при делителе APB != 1 таймер тактируется x2
    return (float)((htim->Instance->PSC + 1) * (htim->Instance->ARR + 1)) / f_tim;
}

void Init_Variables()
{
    dt = Timer_Period(&htim1);
    dt_i = Timer_Period(&htim2);
    dt_w = dt_i * REG_W_DIV;
    dt_ref = Timer_Period(&htim3);

    Epr = 0.0;
    Kpr = 1;
    Tpr = 0.0004;
    K1pr = dt*Kpr/Tpr;
    K2pr = (Tpr - dt)/Tpr;

    // Малые некомпенсированные постоянные, по которым настраиваются регуляторы.
    // Контур тока настроен по физической постоянной преобразователя.
    // Контур скорости по теории тоже настраивается по Tpr, но при напряжении 24 В
    // контур тока не успевает отрабатывать задания так быстро, как предполагает СО,
    // и регуляторы уходят в автоколебания. Поэтому контур скорости настроен медленнее:
    // по моделированию граница автоколебаний около 3 мс, 5 мс взято с запасом
    Tmu_i = Tpr;
    Tmu_w = 0.005;

    Uyac = 24.0;
    Ryac = 0.75;
    Tyac = 0.1;
    K1yac = dt/(Tyac*Ryac);
    K2yac = (Tyac - dt)/Tyac;

    c = 0.7;
    Je = 0.2;
    KJe = dt/Je;

    Mc = 0.0;
    Mload = 5.0;
    I_max = 20.0;
    U_max = 24.0; // Напряжение питания преобразователя

    i_yac = 0.0;
    w_dv = 0.0;

    i_ref = 0;
    w_ref = 0;

    w_ref_f = 0;
    Kf_w = dt_w/(8*Tmu_w);

    di = 0;
    dw = 0;

    OutP_i = 0;
    OutI_i = 0;
    Out_i = 0;

    OutP_w = 0;
    OutI_w = 0;
    Out_w = 0;

    Kot = 1.0;
    Kos = 1.0;

    Ti_i = Tyac;
    Kint_i = dt_i/Ti_i;
    Kpro_i = Tyac*Ryac/(2*Tmu_i*Kpr*Kot);

    Ti_w = 8*Tmu_w;
    Kint_w = dt_w/Ti_w;
    Kpro_w = Je*Kot/(4*Tmu_w*Kos*c);

    // Для того чтобы МК запоминал режим при сбросе
    __HAL_RCC_PWR_CLK_ENABLE(); // тактирование PWR: чтобы менять бит DBP
    __HAL_RCC_BKP_CLK_ENABLE(); // тактирование BKP: для чтения и записи DR1..DR10
    HAL_PWR_EnableBkUpAccess(); // DBP = 1: разрешить запись в backup-домен

    uint16_t Saved = BKP->DR1;
    if (Saved >= 1 && Saved <= MODE_COUNT) Set_Mode(Saved - 1);
    else Set_Mode(MODE_OPEN_LOOP);
}

// Переключение режима. Вызывается из Init_Variables() и из прерывания TIM3,
// поэтому регуляторы (TIM2) не могут выполниться посреди сброса
void Set_Mode(Mode_Type mode)
{
    Mode = mode;
    Mode_Request = mode;

    // Число тиков TIM3 между сменами знака задания
    Period = (uint16_t)(Modes[mode].half_period / dt_ref + 0.5f);
    if (Period < 1) Period = 1;

    Counter = 0;
    Ref_Sign = 1.0f;
    Mc = 0.0f;

    OutI_i = 0.0f;
    OutI_w = 0.0f;
    w_ref_f = 0.0f;

    // Для того чтобы МК запоминал режим при сбросе
    BKP->DR1 = mode + 1; // 0 - ничего не сохранено
}
