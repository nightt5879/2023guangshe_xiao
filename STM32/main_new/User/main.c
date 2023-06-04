#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "GPIO.h"
#include "PWM.h"
#include "MPU6050.h"
#include "math.h"

//#include "Key.h"

uint8_t KeyNum;
uint16_t G1 = 0 ,G2 = 0;
uint8_t ID;
int16_t AX, AY, AZ, GX, GY, GZ;
int32_t start_GZ,acc_GZ = 0,del; //del represents the difference value.
int main(void)
{
	Serial_Init();
	PWM_Init();
	GPIOInit();
	OLED_Init();
	MPU6050_Init();
	OLED_ShowString(1, 1, "ID:");
	ID = MPU6050_GetID();
	OLED_ShowHexNum(1, 4, ID, 2);

	while (1)
	{
		// below are the serial port
		if (Serial_GetRxFlag() == 1)  //get the data from serial port
		{
			G1 = Serial_RxPacket[2] << 8 | Serial_RxPacket[3];
			G2 = Serial_RxPacket[4] << 8 | Serial_RxPacket[5];
//			OLED_ShowString(2, 1, "carmove:");
			if(Serial_RxPacket[0] == 0x00 && Serial_RxPacket[1] == 0x00) //the car stop
			{
				GPIO_Set(3);
				GPIO_Set(6);
				PWM_SetCompara(1,G1);
				PWM_SetCompara(2,G2);
			}
			else if (Serial_RxPacket[0] == 0x01 && Serial_RxPacket[1] == 0x00) //the car forward
			{
				GPIO_Set(1);
				GPIO_Set(4);
				PWM_SetCompara(1,G1);
				PWM_SetCompara(2,G2);
			}
			else if (Serial_RxPacket[0] == 0x02 && Serial_RxPacket[1] == 0x00) //the car back
			{
				GPIO_Set(2);
				GPIO_Set(5);
				PWM_SetCompara(1,G1);
				PWM_SetCompara(2,G2);
			}
			else if (Serial_RxPacket[0] == 0x03 && Serial_RxPacket[1] == 0x00) //the car turn left
			{
				GPIO_Set(1);
				GPIO_Set(5);
				PWM_SetCompara(1,G1);
			    PWM_SetCompara(2,G2);
			}
			else if (Serial_RxPacket[0] == 0x04 && Serial_RxPacket[1] == 0x00) //the car turn right
			{
				GPIO_Set(2);
				GPIO_Set(4);
				PWM_SetCompara(1,G1);
				PWM_SetCompara(2,G2);
			}
			else if (Serial_RxPacket[0] == 0x05)
			{
				OLED_ShowString(3, 1, "carmove222");
				for	(int i = 1;i < 6;i++)  // get the 6050 data for 5 times
				{
					MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
				}
				start_GZ = GZ;
				if (Serial_RxPacket[1] == 0x00) //turning left
				{
					GPIO_Set(1); 
					GPIO_Set(5);
				}
				else if (Serial_RxPacket[1] == 0x01) // turning right
				{
					GPIO_Set(2);
					GPIO_Set(4);
				}
				PWM_SetCompara(1,500);
				PWM_SetCompara(2,500);
				while(1)  // get the 6050 data
				{
					MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
					del = GZ - start_GZ;
					OLED_ShowSignedNum(2, 1,start_GZ, 5);
					OLED_ShowSignedNum(3, 1, GZ, 5);
					OLED_ShowSignedNum(4, 1, acc_GZ, 5);
					if (fabs(del) > 400)  //Filter out noise.
					{
						acc_GZ += (int)(del/10);
					}
					if (fabs(acc_GZ) > G1)
					{
						// stop the car
						GPIO_Set(3);
						GPIO_Set(6);
						acc_GZ = 0; // reset the acc_GZ
						break;
					}
				}
			}
		}
	}
}

