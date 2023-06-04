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
// 定义PID控制器的参数
double K = 2.0;  // 控制器增益
double Kp = 2.0;  // 比例增益
double Ki = 0.0;  // 积分增益
double Kd = 0.0;  // 微分增益

// 定义PID控制器的状态
double prev_error = 0.0;  // 上一次的误差
double integral = 0.0;  // 误差的累积值
double error = 0.0;  // 误差
int16_t pwm_left,pwm_right;  // 左右电机的PWM值
void car_forward(void);
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
			else if (Serial_RxPacket[0] == 0x06)
			{
				car_forward();
			}
		}
	}
}



// this the GPT code
void car_forward()  // 让小车按照MPU6050和PID控制器前进
{
    for (int i = 1; i < 6; i++)  // 读取6050的数据5次
    {
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
    }
    start_GZ = GZ;
    GPIO_Set(1);
    GPIO_Set(4);
	PWM_SetCompara(1,0);
	PWM_SetCompara(2,0);  //let the car stop
    while(1)  // 不断读取6050的数据
    {
		if (Serial_GetRxFlag() == 1)  // RX get the data
		{
			if(Serial_RxPacket[0] == 0x00 && Serial_RxPacket[1] == 0x00) //the car stop
			{
				GPIO_Set(3);
				GPIO_Set(6);
				// reset all the value
				integral = 0;
				prev_error = 0;
				error = 0;
				break;
			}
		}
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);	
		del = GZ - start_GZ;				
		if (fabs(del) > 500)  //Filter out noise.
		{
			error = (GZ - start_GZ)/10;  // 计算误差
		}
       else
	   {
			error = 0;
	   }

        // 计算PID的每个部分
        double P = error;  // 比例部分
        integral += error;  // 积分部分，注意我们需要累积所有的误差
        double D = error - prev_error;  // 微分部分，这是这次和上次误差的差

        // 计算控制输入
        double control = K*(Kp*P + Ki*integral + Kd*D);

        // TODO: 使用控制输入调整小车的速度
		// the pwm need the int type
		pwm_left = 400 - (int)(control);
		pwm_right = 400 + (int)(control);
		// OLED_ShowSignedNum(2, 6,(int)(control), 5);
		// OLED_ShowSignedNum(3, 6,pwm_left, 5);
		// OLED_ShowSignedNum(4, 6,pwm_right, 5);
		// 使用fmax和fmin将pwm_left和pwm_right限制在0到1000
		pwm_left = (int)fmax(0, fmin(1000, pwm_left));
		pwm_right = (int)fmax(0, fmin(1000, pwm_right));
		PWM_SetCompara(1,pwm_left);
		PWM_SetCompara(2,pwm_right);
        // 记录这次的误差，以供下次计算微分部分时使用
        prev_error = error;

        // 显示一些信息
        // OLED_ShowSignedNum(2, 1,start_GZ, 5);
        // OLED_ShowSignedNum(3, 1, GZ, 5);
		OLED_ShowSignedNum(2, 1, pwm_left, 5);
        OLED_ShowSignedNum(3, 1, pwm_right, 5);
        OLED_ShowSignedNum(4, 1, (int)(control), 5);
    }
}
