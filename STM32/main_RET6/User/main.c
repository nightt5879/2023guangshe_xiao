#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "motor.h"


#define TEST_PWM_DUTY 100
int main(void)
{
	motor_init();
	control_motor(MOTOR_FR, MOTOR_BACKWARD, TEST_PWM_DUTY);
	while (1)
	{	
	}
}
