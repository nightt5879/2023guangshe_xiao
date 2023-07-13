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
//double K = 5.0;  // 控制器增益
//double Kp = 1.0;  // 比例增益
//double Ki = 0.0;  // 积分增益
//double Kd = 20.0;  // 微分增益

//// 定义PID控制器的状态
//double prev_error = 0.0;  // 上一次的误差
//double integral = 0.0;  // 误差的累积值
//double error = 0.0;  // 误差
int16_t pwm_left,pwm_right;  // 左右电机的PWM值
void car_forward(void);
uint16_t a;
uint8_t level;
uint8_t change_flag = 0;
uint8_t b = 0;
uint16_t c = 0;
uint16_t d = 0;
int32_t acc = 0;  //转动累计值
uint16_t target = 1650;
#define PWM_START_VALUE 200
#define PWM_END_VALUE 60
#define DECELERATION_DISTANCE 800
#define TARGET_VALUE 2000  // 替换为你的实际目标值
#define SPIN 1
#define GUAN 0x281
#define MAX_PWM = 500
#define MIN_PWM = 0
#define BASE_PWM 100
double speed;
void  pid_controller();
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

	PWM_SetCompara(2,BASE_PWM);
//	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
//Delay_s(5);
//	GPIO_SetBits(GPIOA, GPIO_Pin_2);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
//Delay_s(5);
//	GPIO_SetBits(GPIOB, GPIO_Pin_5);
//	Delay_s(5);
//		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
//	 GPIO_SetBits(GPIOB, GPIO_Pin_2);

	uint8_t lastLevel = 0;  // 上一次读取的电平
	uint32_t lastChangeTime = 0;  // 上一次电平改变的时间
    uint32_t counter = 0;  // 计数器
	uint8_t pwmValue = PWM_START_VALUE;
	 MPU6050_Init();
	while (1)
	{
//		level = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);  // 读取当前电平
//		c = TIM_GetCounter(TIM1);  // 读取计数器值
//		pid_controller();
//			if (level == 0 ) b = 0;
//			else if(level == 1) b = 1;
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
//			  counter+= 1;
//				 c = TIM_GetCounter(TIM1);
//			c++;

        // 如果计数器达到阈值，切换 GPIOB4 和 GPIOB5 的状态
        // if (counter >= COUNTER_THRESHOLD)
        // {
		// 			counter = 0;  // 重置计数器
		// 			a ++;
		// 		}
		// 		if (a >= 300)
		// 		{
        //     a = 0;
		// 			if (change_flag == 0)
		// 			{
		// 		GPIO_SetBits(GPIOA, GPIO_Pin_3);
		// 		GPIO_ResetBits(GPIOA,GPIO_Pin_2);
		// 				change_flag = 1;
		// 			}
		// 			else
		// 			{
		// 		GPIO_SetBits(GPIOA, GPIO_Pin_2);
		// 		GPIO_ResetBits(GPIOA,GPIO_Pin_3);	
		// 			change_flag = 0;
		// 			}
        // }
		// 如果编码器的值大于一定 就停下


//			// 在达到目标值前的 DECELERATION_DISTANCE 距离内开始减速
//			if (c > TARGET_VALUE - DECELERATION_DISTANCE)
//			{
//					// 在这个区间内线性减小 PWM 值
//					uint32_t range = TARGET_VALUE - (TARGET_VALUE - DECELERATION_DISTANCE);
//					uint32_t positionInRange = c - (TARGET_VALUE - DECELERATION_DISTANCE);
//					uint32_t pwmRange = PWM_START_VALUE - PWM_END_VALUE;
//		
//					// 根据当前位置在减速区间内的比例，调整 PWM 值
//					pwmValue = PWM_START_VALUE - ((positionInRange * pwmRange) / range);
//		
//					// 限制 PWM 值的最小值为 PWM_END_VALUE
//					if (pwmValue < PWM_END_VALUE)
//					{
//							pwmValue = PWM_END_VALUE;
//					}
//		
//					PWM_SetCompara(2, pwmValue);
//			}

			// 如果达到目标值，停止电机
//			if (c > (TARGET_VALUE * SPIN) -((SPIN-1)*GUAN) )
//			{
//					d = c;
//					GPIO_SetBits(GPIOA, GPIO_Pin_3);
//					GPIO_SetBits(GPIOA,GPIO_Pin_2);
//					TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //清除 TIM1 更新中断标志
//					TIM_SetCounter(TIM1, 0);
//			}
	}

}

//void TIM1_UP_IRQHandler(void) {
//				  GPIO_SetBits(GPIOA, GPIO_Pin_3);
//        GPIO_SetBits(GPIOA,GPIO_Pin_2);
//        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //清除 TIM1 更新中断标志
////    if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //确保是 TIM1 更新中断
////    {

////        // 在这里停止电机，使用GPIO操作
//////    }
//}
// TIM3 中断服务函数
void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  // 如果发生了更新中断
    {
        // 在这里读取你的外部中断值，计算速度，并清零
				speed = (float)TIM_GetCounter(TIM1) /1024 * 3/ 7 * 10;
				TIM_SetCounter(TIM1, 0);
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  // 清除更新中断标志位
    }
}

// PID coefficients
float Kp = 20, Ki = 0, Kd = 2;

// PID variables
float error, previous_error, integral, derivative;

// Desired and current speed
float target_speed = 4;

// Integral limit values
float integral_min = -10, integral_max = 10;

// PWM output
float pwm_output;

float PWM_MAX = 200, PWM_MIN = -200;
uint16_t set_pwm;
void pid_controller() {
    // Calculate error
    error = target_speed - speed;

    // Calculate integral term with saturation
    integral += error;
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < integral_min) {
        integral = integral_min;
    }

    // Calculate derivative term
    derivative = error - previous_error;

    // Calculate control output
    pwm_output = Kp * error + Ki * integral + Kd * derivative;
    // Ensure PWM output is within allowable range
    if (pwm_output > PWM_MAX) {
        pwm_output = PWM_MAX;
    } else if (pwm_output < PWM_MIN) {
        pwm_output = PWM_MIN;
    }
	set_pwm = (int)pwm_output + BASE_PWM;  // 把PWM_OUTPUT变成整数
    // Update the PWM signal
	PWM_SetCompara(2, set_pwm);
    // Store current error for next iteration
    previous_error = error;
}
