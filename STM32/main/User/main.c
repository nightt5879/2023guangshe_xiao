#include "stm32f10x.h"
#include "PWM.h"
#include "GPIO.h"
#include "OLED.h"
#include "CountSensor.h"

uint16_t G1,G2;
int main(void)
{
	PWM_Init();
	GPIOInit();
	OLED_Init();
	CountSensor_Init();
	OLED_ShowString(1, 1, "Count1:");
	OLED_ShowString(2, 1, "Count2:");
	GPIO_Set(1);
	GPIO_Set(4);
	PWM_SetCompara(1,400);
	PWM_SetCompara(2,400);
	while (1)
	{
		G1 = CountSensor_Get_1();
		G2 = CountSensor_Get_2();
		OLED_ShowNum(1, 8, G1, 5);
		OLED_ShowNum(2, 8, G2, 5);
		if(G1 > 500)
			GPIO_Set(3);
		if(G2 > 500)
			GPIO_Set(6);
	}
}
