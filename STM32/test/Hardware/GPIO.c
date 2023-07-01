#include "stm32f10x.h"                  // Device header
#include "GPIO.h"

void GPIOInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 |GPIO_Pin_5 |GPIO_Pin_6 |GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void GPIO_Set(uint8_t CHx)  //control the motor rotation
{
    switch (CHx){
        case 1:  //the positive rotary of motor#1 
			GPIO_ResetBits(GPIOA, GPIO_Pin_3);
			GPIO_SetBits(GPIOA, GPIO_Pin_2);
			break;
        case 2:  //the negative rotary of motor#1
			GPIO_ResetBits(GPIOA, GPIO_Pin_2);
			GPIO_SetBits(GPIOA, GPIO_Pin_3);
			break;
        case 3:  //the motor#1 stopped
			GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
			break;
        case 4:  //the positive rotary of motor#2
			GPIO_ResetBits(GPIOA, GPIO_Pin_4);
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
			break;
		case 5:  //the negative rotary of motor#2
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
			GPIO_SetBits(GPIOA, GPIO_Pin_4);
			break;
		case 6:  //the motor#2 stopped
			GPIO_ResetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_5);
			break;
        default:break;
    }
}
