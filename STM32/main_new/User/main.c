#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "GPIO.h"
#include "PWM.h"

//#include "Key.h"

uint8_t KeyNum;
uint16_t G1 = 0 ,G2 = 0;
int main(void)
{
	Serial_Init();
	PWM_Init();
	GPIOInit();
	OLED_Init();
	// CountSensor_Init();
	// // Timer_Init();
	OLED_ShowString(1, 1, "Count1:");
//	GPIO_Set(1);
//	GPIO_Set(4);
	PWM_SetCompara(1,400);
	PWM_SetCompara(2,400);
	// Delay_ms(1000);
	// GPIO_Set(2);
	// Delay_ms(1000);
	// GPIO_Set(1);
	while (1)
	{
		if (Serial_GetRxFlag() == 1)  //get the data from serial port
		{
			OLED_ShowString(2, 1, "Count2:");
			G1 = Serial_RxPacket[2] << 8 | Serial_RxPacket[3];
			G2 = Serial_RxPacket[4] << 8 | Serial_RxPacket[5];
			if(Serial_RxPacket[0] == 0x00 && Serial_RxPacket[1] == 0x00) //the car stop
			{
				GPIO_Set(3);
				GPIO_Set(6);
			}
			else if (Serial_RxPacket[0] == 0x01 && Serial_RxPacket[1] == 0x00) //the car forward
			{
				GPIO_Set(1);
				GPIO_Set(4);
			}
			else if (Serial_RxPacket[0] == 0x02 && Serial_RxPacket[1] == 0x00) //the car back
			{
				GPIO_Set(2);
				GPIO_Set(5);
			}
			else if (Serial_RxPacket[0] == 0x03 && Serial_RxPacket[1] == 0x00) //the car turn left
			{
				GPIO_Set(1);
				GPIO_Set(5);
			}
			else if (Serial_RxPacket[0] == 0x04 && Serial_RxPacket[1] == 0x00) //the car turn right
			{
				GPIO_Set(2);
				GPIO_Set(4);
			}
			PWM_SetCompara(1,G1);
			PWM_SetCompara(2,G2);
		}
	}
}

