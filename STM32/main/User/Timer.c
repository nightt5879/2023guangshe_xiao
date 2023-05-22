#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "CountSensor.h"

uint8_t in_timer = 0;
uint16_t count_1_first,count_2_first;
uint16_t count_1_secend,count_2_secend;
uint16_t speed_1,speed_2;
extern uint16_t CountSensor_Count_1,CountSensor_Count_2;
void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_InternalClockConfig(TIM3);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM3, ENABLE);
}


void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		if (in_timer == 0)
		{
			in_timer = 1;
			//get the motor count
			count_1_first = CountSensor_Count_1;
			count_2_first = CountSensor_Count_2;
		}
		else 
		{
			in_timer = 0;
			//get the motor count
			count_1_secend = CountSensor_Count_1;
			count_2_secend = CountSensor_Count_2;
			//get the speed  10ms per count the speed = count*100 Uint:cm/s
			//The diameter of the wheel is 6.5cm,The circumference is 20.4cm.one count mean 1.02cm
			speed_1 = (count_1_secend - count_1_first);
			speed_2 = (count_2_secend - count_2_first);
			//clear the count
			CountSensor_Count_1 = 0;
			CountSensor_Count_2 = 0;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

