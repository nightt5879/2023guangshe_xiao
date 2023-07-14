#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "motor.h"
#include "UART.h"

#define TEST_PWM_DUTY 100
#define TARGET_DISTANCE 60
#define TEST_SPEED 10

void stop_car(void);
#define DECELERATION_DISTANCE 20  // Decelerate when 10 units away from the target
#define REVERSE_DISTANCE 0  // Reverse when 10 units away from the target
uint8_t fl_dir = 0;
uint8_t fr_dir = 0;
uint8_t bl_dir = 0;
uint8_t br_dir = 0;

uint8_t directions[] = {1,1,1,1};
float speeds[] = {TEST_SPEED, TEST_SPEED, TEST_SPEED, TEST_SPEED};
uint8_t control_flags[] = {1,1,1,1};
uint16_t break_flag = 0;

// the four speed of the motor
extern float fl_speed; 
extern float fr_speed;
extern float bl_speed;
extern float br_speed;
//the distance move of the car
extern float distance;
//send the data to the computer
float data[5] = {0.1, 0.2, 0.3, 0.4}; // 9 channels
uint8_t send_flag = 0;
float sys_clock;
uint16_t test_flag = 0;
float deceleration_factor;
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
	SystemInit();
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    sys_clock = (float)RCC_Clocks.SYSCLK_Frequency;
//	control_motor(MOTOR_BR, MOTOR_FORWARD, TEST_PWM_DUTY);
	control_motor_speed(directions, speeds, control_flags);
//	Delay_s(1);
//	stop_car();
	while (1)
	{	
		//test the for counter
//		fl_dir = Read_FL_Direction();
//        fr_dir = Read_FR_Direction();
//        bl_dir = Read_BL_Direction();
//        br_dir = Read_BR_Direction();

//		if (send_flag == 1) 
//		{
//			break_flag ++;
//			send_flag = 0;
//			data[0] = br_speed;
//			data[1] = bl_speed;
//			data[2] = fl_speed;
//			data[3] = fr_speed;
//			data[4] = distance;
//			send_data(data, 4);  // send the data to the computer
//		}
//		if (break_flag > 100 && break_flag < 200) // change the direction
//		{
//			directions[0] = 1;
//			directions[1] = 0;
//			directions[2] = 0;
//			directions[3] = 1;
//			control_motor_speed(directions, speeds, control_flags);
//		}
//		else if(break_flag > 200 && break_flag < 300)
//		{
//			directions[0] = 0;
//			directions[1] = 0;
//			directions[2] = 0;
//			directions[3] = 0;
//			control_motor_speed(directions, speeds, control_flags);
//		}
//		else if(break_flag > 300)
//		{
//			directions[0] = 0;
//			directions[1] = 1;
//			directions[2] = 1;
//			directions[3] = 0;
//			control_motor_speed(directions, speeds, control_flags);
//		}
//		if(break_flag > 385)
//		{
//			break;
//		}
//	if (distance > TARGET_DISTANCE - DECELERATION_DISTANCE && distance < TARGET_DISTANCE - REVERSE_DISTANCE) {
//		// Calculate a deceleration factor between 0 and 1.
//		deceleration_factor = (TARGET_DISTANCE - distance) / DECELERATION_DISTANCE; //Percentage deceleration
////		if (deceleration_factor < 0.2) deceleration_factor = 0.4;

//		// Apply the deceleration factor to the speeds.
//		// When deceleration_factor is 1, the speed will be the same as it was.
//		// When deceleration_factor is 0, the speed will be 0.
//		speeds[0] *= deceleration_factor;
//		speeds[1] *= deceleration_factor;
//		speeds[2] *= deceleration_factor;
//		speeds[3] *= deceleration_factor;
//		
//		// Apply the new speeds.
//		control_motor_speed(directions, speeds, control_flags);
//		// test_flag ++;
//	}
		// Reverse Motor Brake
//		if (TARGET_DISTANCE - REVERSE_DISTANCE <=distance && distance <= TARGET_DISTANCE) {
//			// Apply the new speeds.
//			directions[0] = 0;
//			directions[1] = 0;
//			directions[2] = 0;
//			directions[3] = 0;
//			speeds[0] = TEST_SPEED;
//			speeds[1] = TEST_SPEED;
//			speeds[2] = TEST_SPEED;
//			speeds[3] = TEST_SPEED;
//			control_motor_speed(directions, speeds, control_flags);
//			// test_flag ++;
//		}
		if (distance > TARGET_DISTANCE) break;
		if (test_flag >=1) break;
//		if (break_flag > 100) break;
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
	control_motor_speed(directions, speeds, control_flags);
	control_motor(MOTOR_BL, MOTOR_FORWARD, 0);
	control_motor(MOTOR_BR, MOTOR_FORWARD, 0);
	control_motor(MOTOR_FL, MOTOR_FORWARD, 0);
	control_motor(MOTOR_FR, MOTOR_FORWARD, 0);
}
