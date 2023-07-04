#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "GPIO.h"
#include "PWM.h"
#include "MPU6050.h"
#include "math.h"

//#include "Key.h"
#define DEBOUNCE_DELAY 20  // 去抖动延迟时间，单位为毫秒
#define COUNTER_THRESHOLD 10000  // 计数器阈值
uint8_t KeyNum;
uint16_t G1 = 0 ,G2 = 0;
uint8_t ID;
int16_t AX, AY, AZ, GX, GY, GZ;
int32_t start_GZ,acc_GZ = 0,del; //del represents the difference value.
// 定义PID控制器的参数
double K = 5.0;  // 控制器增益
double Kp = 1.0;  // 比例增益
double Ki = 0.0;  // 积分增益
double Kd = 20.0;  // 微分增益

// 定义PID控制器的状态
double prev_error = 0.0;  // 上一次的误差
double integral = 0.0;  // 误差的累积值
double error = 0.0;  // 误差
int16_t pwm_left,pwm_right;  // 左右电机的PWM值
void car_forward(void);
uint16_t a;
uint8_t level;
uint8_t change_flag = 0;
uint8_t b = 0;
uint16_t c = 0;

uint32_t GetTick(void)
{
    return SysTick->LOAD - SysTick->VAL;
}

int main(void)
{
		if (SysTick_Config(SystemCoreClock / 1000))  // 配置 SysTick 定时器每 1ms 中断一次
{
    while (1);  // 如果失败，进入死循环
}

	PWM_Init();
	GPIOInit();

	PWM_SetCompara(2,150);
//	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	GPIO_SetBits(GPIOA, GPIO_Pin_3);
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
Delay_s(5);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
Delay_s(5);
//	GPIO_SetBits(GPIOB, GPIO_Pin_5);
//	Delay_s(5);
//		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
//	 GPIO_SetBits(GPIOB, GPIO_Pin_2);

	uint8_t lastLevel = 0;  // 上一次读取的电平
	uint32_t lastChangeTime = 0;  // 上一次电平改变的时间
    uint32_t counter = 0;  // 计数器
	while (1)
	{
			level = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);  // 读取当前电平
			if (level == 0 ) b = 0;
			else if(level == 1) b = 1;
//			if (level != lastLevel)  // 如果电平改变了
//			{
//					if (GetTick() - lastChangeTime >= DEBOUNCE_DELAY)  // 如果距离上一次电平改变已经过去了足够的时间
//					{
//							lastLevel = level;  // 更新电平
//							lastChangeTime = GetTick();  // 更新时间

//							// 在这里处理电平改变事件
//							if (level == 0)
//							{
//									// 如果 DIR 信号为 0，设置电机为正转
//									a = 1;
//							}
//							else
//							{
//									// 如果 DIR 信号为 1，设置电机为反转
//									a = 0;
//							}
//					}
//			}
//			else
//			{
//					lastChangeTime = GetTick();  // 如果电平没有改变，也更新时间
//			}
			  counter+= 1;
				 c = TIM_GetCounter(TIM1);

        // 如果计数器达到阈值，切换 GPIOB4 和 GPIOB5 的状态
        if (counter >= COUNTER_THRESHOLD)
        {
					counter = 0;  // 重置计数器
					a ++;
				}
				if (a >= 300)
				{
            a = 0;
					if (change_flag == 0)
					{
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
				GPIO_ResetBits(GPIOA,GPIO_Pin_2);
						change_flag = 1;
					}
					else
					{
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
				GPIO_ResetBits(GPIOA,GPIO_Pin_3);	
					change_flag = 0;
					}
        }
	}

}