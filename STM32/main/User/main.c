#include "stm32f10x.h"
#include "PWM.h"
#include "GPIO.h"
#include "OLED.h"
#include "CountSensor.h"
#include "Timer.h"
#include "Delay.h"

extern uint16_t speed_1,speed_2;
extern uint16_t CountSensor_Count_1,CountSensor_Count_2;
uint16_t G1,G2;
int main(void)
{
	PWM_Init();
	GPIOInit();
	OLED_Init();
	CountSensor_Init();
	// Timer_Init();
	OLED_ShowString(1, 1, "Count1:");
	OLED_ShowString(2, 1, "Count2:");
	GPIO_Set(1);
	GPIO_Set(4);
	PWM_SetCompara(1,400);
	PWM_SetCompara(2,400);
	Delay_ms(1000);
	GPIO_Set(2);
	Delay_ms(1000);
	GPIO_Set(1);
	while (1)
	{
		OLED_ShowNum(1, 8, CountSensor_Count_1, 5);
		OLED_ShowNum(2, 8, CountSensor_Count_2, 5);
		// G1 = CountSensor_Get_1();
		// G2 = CountSensor_Get_2();
		// OLED_ShowNum(1, 8, G1, 5);
		// OLED_ShowNum(2, 8, G2, 5);
		// if(G1 > 500)
		// 	GPIO_Set(3);
		// if(G2 > 500)
		// 	GPIO_Set(6);
	}
}
