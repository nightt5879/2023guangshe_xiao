#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "motor.h"


#define TEST_PWM_DUTY 100

uint8_t fl_dir = 0;
uint8_t fr_dir = 0;
uint8_t bl_dir = 0;
uint8_t br_dir = 0;

uint8_t directions[] = {1,1,1,1};
float speeds[] = {10,10,10,10};
uint8_t control_flags[] = {0,0,0,1};



int main(void)
{
	Delay_ms(100);
	TIM6_Configuration();
	motor_init();	
	TIM1_ETR_Config();
	TIM8_ETR_Config();
	TIM3_ETR_Config();
	TIM2_ETR_Config();
	dir_gpio_input_Config();
	init_pid();
	control_motor(MOTOR_BL, MOTOR_FORWARD, TEST_PWM_DUTY);
	control_motor_speed(directions, speeds, control_flags);
	while (1)
	{	
		//test the for counter
		fl_dir = Read_FL_Direction();
        fr_dir = Read_FR_Direction();
        bl_dir = Read_BL_Direction();
        br_dir = Read_BR_Direction();
	}
}
