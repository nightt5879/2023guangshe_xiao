#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "motor.h"
#include "UART.h"
#include "6050control.h"
#include "MPU6050.h"
#include <stdlib.h>
#include "GPIO.h"
#define TEST_PWM_DUTY 100
#define TARGET_DISTANCE 40
#define TEST_SPEED 6
#define DECELERATION_DISTANCE 20  // Decelerate when 10 units away from the target
#define REVERSE_DISTANCE 0  // Reverse when 10 units away from the target
#define STBLE_TIME 30
#define MOVE_X 40
#define MOVE_Y 30
void stop_car(void);
void init(void);
void send_to_win(void);
void move_control_main(float target_x, float target_y);

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
extern float distance_x_filter,distance_y_filter,move_target_distance_x,move_target_distance_y;
extern float fr_target_speed, fl_target_speed, br_target_speed, bl_target_speed;
extern uint8_t corner_flag;
extern uint8_t right_modle[], left_modle[], front_modle[], back_modle[];
extern int16_t distance_x_uart, distance_y_uart, correction_speed; // get the disatnce target from the uart
extern uint8_t one_move_flag;
uint8_t speed_test;
//send the data to the computer
float data[CH_COUNT]; 
uint8_t send_flag = 0;
float sys_clock;
uint16_t test_flag = 0;
float deceleration_factor;
uint8_t test_id;
int main(void)
{
	init();
	// Delay_ms(1000);
	// mpu_6050_corretion();
	// Delay_ms(1000);
	//	control_motor(MOTOR_BR, MOTOR_FORWARD, TEST_PWM_DUTY);
	toggle_delta_v(1);
	// control_move(MOVE_X,0);
	// control_move(0,MOVE_Y);
	// one_move_flag = 1;
	// move_control_main(0,-40);
	// move_control_main(40,0);
//	Delay_s(1);
//	 move_control_main(80,80);
//	control_motor_speed(speeds, control_flags);	
	// distance_flag = 0;
	// speed_test = 3;
	// fl_target_speed = 3;
	// fr_target_speed = 3;
	// bl_target_speed = 3;
	// br_target_speed = 3;

	while (1)
	{	
		get_6050_data(); //I2C communication is too low, we get the data in main use in interrupt
		test_id = MPU6050_GetID();
		send_to_win();
		read_gray_scale_module(right_modle, left_modle, front_modle, back_modle);
		if(one_move_flag == 1 && distance_flag == 1)
		{
			one_move_flag = 0;
			distance_flag = 0;
			// Delay_ms(10);
			//control the car
			Serial_SendByte(0x01);
		}
		// else
		// {
		// 	Serial_SendByte(0x02);
		// }

		if (break_flag > 1)
		{
			stop_the_car();
		}
		// if (distance_y_encoder > TARGET_DISTANCE)
		// {
		// 	stop_car();
		// }
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
	//serial to the raspberry
	Serial_Init();
	//get the system clock
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	sys_clock = (float)RCC_Clocks.SYSCLK_Frequency;
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
		data[17] = distance_x_filter;
		data[18] = distance_y_filter;
		data[19] = move_target_distance_x;
		data[20] = move_target_distance_y;
		data[21] = test_id;
		send_data(data, CH_COUNT);
	}
}

void move_control_main(float target_x, float target_y)
{
	while(distance_flag == 0)  //wait for the distance flag
	{	
	}
	if (distance_flag == 1)
	{
		distance_flag = 0;
		corner_flag = 1;
		control_move(target_x,target_y);
	}
}
