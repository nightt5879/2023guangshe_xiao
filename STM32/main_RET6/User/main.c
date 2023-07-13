#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "motor.h"


#define TEST_PWM_DUTY 90

uint8_t fl_dir = 0;
uint8_t fr_dir = 0;
uint8_t bl_dir = 0;
uint8_t br_dir = 0;
int main(void)
{
	motor_init();	
	TIM1_ETR_Config();
	TIM8_ETR_Config();
	TIM3_ETR_Config();
	TIM2_ETR_Config();
	dir_gpio_input_Config();
//	control_motor(MOTOR_FL, MOTOR_BACKWARD, TEST_PWM_DUTY);
//	control_motor(MOTOR_FR, MOTOR_BACKWARD, TEST_PWM_DUTY);
//	control_motor(MOTOR_BL, MOTOR_BACKWARD, TEST_PWM_DUTY);
//	control_motor(MOTOR_BR, MOTOR_BACKWARD, TEST_PWM_DUTY);
	while (1)
	{	
		//test the for counter
		fl_dir = Read_FL_Direction();
        fr_dir = Read_FR_Direction();
        bl_dir = Read_BL_Direction();
        br_dir = Read_BR_Direction();
	}
}
