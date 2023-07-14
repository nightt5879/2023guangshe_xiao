#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "motor.h"
#include "UART.h"

#define TEST_PWM_DUTY 100

uint8_t fl_dir = 0;
uint8_t fr_dir = 0;
uint8_t bl_dir = 0;
uint8_t br_dir = 0;

uint8_t directions[] = {1,1,1,1};
float speeds[] = {8,8,8,8};
uint8_t control_flags[] = {1,1,1,1};
uint16_t break_flag = 0;

// the four speed of the motor
extern float fl_speed; 
extern float fr_speed;
extern float bl_speed;
extern float br_speed;
//send the data to the computer
float data[4] = {0.1, 0.2, 0.3, 0.4}; // 9 channels
uint8_t send_flag = 0;
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
	UART4_Init();
//	control_motor(MOTOR_BL, MOTOR_FORWARD, TEST_PWM_DUTY);
	control_motor_speed(directions, speeds, control_flags);

	while (1)
	{	
		//test the for counter
//		fl_dir = Read_FL_Direction();
//        fr_dir = Read_FR_Direction();
//        bl_dir = Read_BL_Direction();
//        br_dir = Read_BR_Direction();

		if (send_flag == 1) 
		{
			break_flag ++;
			send_flag = 0;
			data[0] = br_speed;
			data[1] = bl_speed;
			data[2] = fl_speed;
			data[3] = fr_speed;
			send_data(data, 4);  // send the data to the computer
		}
		if (break_flag > 100 && break_flag < 200) // change the direction
		{
			directions[0] = 1;
			directions[1] = 0;
			directions[2] = 0;
			directions[3] = 1;
			control_motor_speed(directions, speeds, control_flags);
		}
		else if(break_flag > 200 && break_flag < 300)
		{
			directions[0] = 0;
			directions[1] = 0;
			directions[2] = 0;
			directions[3] = 0;
			control_motor_speed(directions, speeds, control_flags);
		}
		else if(break_flag > 300)
		{
			directions[0] = 0;
			directions[1] = 1;
			directions[2] = 1;
			directions[3] = 0;
			control_motor_speed(directions, speeds, control_flags);
		}
		if(break_flag > 385)
		{
			break;
		}
	}
	//stop the motor
	speeds[0] = 0;
	speeds[1] = 0;
	speeds[2] = 0;
	speeds[3] = 0;
	control_motor_speed(directions, speeds, control_flags);
	control_motor(MOTOR_BL, MOTOR_FORWARD, 0);
	control_motor(MOTOR_BR, MOTOR_FORWARD, 0);
	control_motor(MOTOR_FL, MOTOR_FORWARD, 0);
	control_motor(MOTOR_FR, MOTOR_FORWARD, 0);
}
