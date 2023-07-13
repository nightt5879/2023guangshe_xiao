#include "motor.h"
#include "GPIO.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

uint16_t fl_counter = 0;
uint16_t fr_counter = 0;
uint16_t bl_counter = 0;
uint16_t br_counter = 0;

/**
  * @brief  initialize the motor include the pwm and the direction gpio
  * @param  None
  * @retval None
  */
void motor_init(void)
{
	pwm_init();
	motor_dir_gpio_init();
}

/**
  * @brief  control the motor
  * @param  motor_select: the motor you want to control the define can find in motor.h
  * @param  direction: the direction of the motor  the define can find in motor.h
  * @param  duty_cycle: the duty cycle of the pwm
  * @retval None
  */
void control_motor(uint8_t motor_select, uint8_t direction, uint16_t duty_cycle) 
{
    pwm_set_duty_cycle(motor_select, duty_cycle);
    motor_dir_select(motor_select, direction);
}

/**
  * @brief  initialize the ETR for BR motor
  * @param  None
  * @retval None
  */
void TIM1_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM1, ENABLE);
}

/**
  * @brief  initialize the ETR for BL motor
  * @param  None
  * @retval None
  */
void TIM8_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM8, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM8, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM8, ENABLE);
}

/**
  * @brief  initialize the ETR for FL motor
  * @param  None
  * @retval None
  */
void TIM3_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  initialize the ETR for FR motor
  * @param  None
  * @retval None
  */
void TIM2_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  initialize the dir gpio input
  * @param  None
  * @retval None
  */
void dir_gpio_input_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure PC.09 (FL), PC.08 (FR), PA.04 (BL), PB.12 (BR) as input pull-up */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// the motor in the right side 0 means forward, 1 means backward, and the motor in the left side is the opposite
/**
  * @brief  read the FL motor direction
  * @param  None
  * @retval 1: forward, 0: backward
  */
uint8_t Read_FL_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9);
}

/**
  * @brief  read the FR motor direction
  * @param  None
  * @retval 0: forward, 1: backward
  */
uint8_t Read_FR_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8);
}

/**
  * @brief  read the BL motor direction
  * @param  None
  * @retval 1: forward, 0: backward
  */
uint8_t Read_BL_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
}

/**
  * @brief  read the BR motor direction
  * @param  None
  * @retval 0: forward, 1: backward
  */
uint8_t Read_BR_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
}

/**
  * @brief  initialize the TIM5 for encoder
  * @param  None
  * @retval None
  */
void TIM5_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 9999;  // 10ms = (9999 + 1) / 1MHz
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1;  // 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}

/**
  * @brief  the interrupt handler for TIM5 using for PID control of the motor
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

        // PID control code goes here.
        br_counter = TIM_GetCounter(TIM1);
        bl_counter = TIM_GetCounter(TIM8);
        fl_counter = TIM_GetCounter(TIM3);
        fr_counter = TIM_GetCounter(TIM2);
        // reset the counter
        TIM_SetCounter(TIM1, 0);
        TIM_SetCounter(TIM8, 0);
        TIM_SetCounter(TIM3, 0);
        TIM_SetCounter(TIM2, 0);
    }
}

