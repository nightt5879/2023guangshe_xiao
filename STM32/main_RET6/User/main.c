#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "motor.h"
#include "UART.h"
#include "6050control.h"
#include "MPU6050.h"
#include <stdlib.h>
#include "GPIO.h"

#define TEST_PWM_DUTY 100
#define TARGET_DISTANCE 80
#define TEST_SPEED 6
#define DECELERATION_DISTANCE 20  // Decelerate when 10 units away from the target
#define REVERSE_DISTANCE 0  // Reverse when 10 units away from the target
#define STBLE_TIME 30
void stop_car(void);
void init(void);
void send_to_win(void);

float speeds[] = {TEST_SPEED, TEST_SPEED, TEST_SPEED, TEST_SPEED};
uint8_t control_flags[] = {1,1,1,1};
uint16_t break_flag = 0;

// the four speed of the motor
extern float fl_speed; 
extern float fr_speed;
extern float bl_speed;
extern float br_speed;
//the distance move of the car
extern float distance_x_encoder;
extern float distance_y_encoder;
extern float angle_z_encoder;
//the 6050 data from the car
extern float angle_z;
extern float distance_x;
extern float distance_y;
extern float speed_x;
extern float speed_y;
extern float delta_v;
extern uint8_t distance_flag; // the time in the thread in 0.01s
//send the data to the computer
float data[CH_COUNT]; 
uint8_t send_flag = 0;
float sys_clock;
uint16_t test_flag = 0;
float deceleration_factor;
//gray input
uint8_t rightModuleValues[5];
uint8_t leftModuleValues[5];
int main(void)
{
	init();
//	control_motor(MOTOR_BR, MOTOR_FORWARD, TEST_PWM_DUTY);
	toggle_delta_v(1);
//	control_move('x',TARGET_DISTANCE);
	control_motor_speed(speeds, control_flags);
	while (1)
	{	
		get_6050_data(); //I2C communication is too low, we get the data in main use in interrupt
		//send the data to the computer
		// read_gray_scale_module(rightModuleValues, leftModuleValues);
		send_to_win();
		// if (break_flag > 3)
		// {
		// 	distance = 0;
		// 	control_move('x',0);
		// 	break;
		// 	toggle_delta_v(0);
		// 	stop_car();
		// }
		if (distance_y_encoder > TARGET_DISTANCE)
		{
			stop_car();
		}
	}
	//stop the motor
	stop_car();
}

void stop_car(void)
{
	speeds[0] = 0;
	speeds[1] = 0;
	speeds[2] = 0;
	speeds[3] = 0;
	control_motor_speed(speeds, control_flags);
	control_motor(MOTOR_BL, 0);
	control_motor(MOTOR_BR, 0);
	control_motor(MOTOR_FL, 0);
	control_motor(MOTOR_FR, 0);
}

void init(void)
{
	SystemInit();
	Delay_ms(100);
	//6050 and 
	init_6050();
	//control of the motor, include 4PWM and 4DIR
	motor_init();
	//init the gray input
	init_gray_scale_module_gpio();
	//the four encoders
	TIM1_ETR_Config();
	TIM8_ETR_Config();
	TIM3_ETR_Config();
	TIM2_ETR_Config();
	dir_gpio_input_Config();
	//init the pid and the interrupt (10ms)
	init_pid();
	TIM6_Configuration();
	//UART init
	UART4_Init();
	//init the angle pid and interrupt init (1ms)
	init_pid_angle();
	TIM7_Configuration();
	//get the system clock
	// RCC_ClocksTypeDef RCC_Clocks;
	// RCC_GetClocksFreq(&RCC_Clocks);
	// sys_clock = (float)RCC_Clocks.SYSCLK_Frequency;
}

void send_to_win(void)
{
	if (send_flag == 1)
	{
		send_flag = 0;
		data[0] = fl_speed;
		data[1] = fr_speed;
		data[2] = bl_speed;
		data[3] = br_speed;
		data[4] = distance_x_encoder;
		data[5] = distance_y_encoder;
		data[6] = angle_z_encoder;
		data[7] = distance_x;
		data[8] = distance_y;
		data[9] = angle_z;
		data[10] = speed_x;
		data[11] = speed_y;
		data[12] = delta_v;
		data[13] = pid_fl.setpoint;
		data[14] = pid_fr.setpoint;
		data[15] = pid_bl.setpoint;
		data[16] = pid_br.setpoint;
		send_data(data, CH_COUNT);
	}
}
