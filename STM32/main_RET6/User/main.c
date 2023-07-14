#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "motor.h"
#include "UART.h"

#define TEST_PWM_DUTY 100
<<<<<<< HEAD
#define TARGET_DISTANCE 60
#define TEST_SPEED 20
=======
>>>>>>> parent of 400d962 (Update keil-assistant.log)

uint8_t fl_dir = 0;
uint8_t fr_dir = 0;
uint8_t bl_dir = 0;
uint8_t br_dir = 0;

uint8_t directions[] = {1,1,1,1};
float speeds[] = {20,20,20,20};
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
<<<<<<< HEAD
float data[5] = {0.1, 0.2, 0.3, 0.4,0.4}; // 9 channels
uint8_t send_flag = 0;
uint16_t test_flag = 0;
RCC_ClocksTypeDef rcc_clocks;
uint32_t systemClockFrequency = 0;
=======
float data[4] = {0.1, 0.2, 0.3, 0.4}; // 9 channels
uint8_t send_flag = 0;
float sys_clock;
>>>>>>> parent of 400d962 (Update keil-assistant.log)
int main(void)
{
//	Delay_ms(100);
	TIM6_Configuration();
	motor_init();	
	TIM1_ETR_Config();
	TIM8_ETR_Config();
	TIM3_ETR_Config();
	TIM2_ETR_Config();
	dir_gpio_input_Config();
	init_pid();
	UART4_Init();
	RCC_GetClocksFreq(&rcc_clocks);
    
    systemClockFrequency = rcc_clocks.SYSCLK_Frequency;
//	control_motor(MOTOR_BR, MOTOR_FORWARD, TEST_PWM_DUTY);
<<<<<<< HEAD
	control_motor_speed(directions, speeds, control_flags);
=======
//	control_motor_speed(directions, speeds, control_flags);

>>>>>>> parent of 400d962 (Update keil-assistant.log)
	while (1)
	{	
		//test the for counter
//		fl_dir = Read_FL_Direction();
//        fr_dir = Read_FR_Direction();
//        bl_dir = Read_BL_Direction();
//        br_dir = Read_BR_Direction();

<<<<<<< HEAD
//		if (send_flag == 1) 
//		{
//			break_flag ++;
//			send_flag = 0;
//			data[0] = br_speed;
//			data[1] = bl_speed;
//			data[2] = fl_speed;
//			data[3] = fr_speed;
//			data[4] = distance;
//			send_data(data, 5);  // send the data to the computer
//		}
//	if (distance > TARGET_DISTANCE - DECELERATION_DISTANCE && distance < TARGET_DISTANCE - REVERSE_DISTANCE) {
//		// Calculate a deceleration factor between 0 and 1.
//		deceleration_factor = (TARGET_DISTANCE - distance) / DECELERATION_DISTANCE; //Percentage deceleration
//		if (deceleration_factor < 0.2) deceleration_factor = 0.4;

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
		if (distance > TARGET_DISTANCE) break;
=======
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
		if (distance > 40) break;
>>>>>>> parent of 400d962 (Update keil-assistant.log)
	}
	//stop the motor
	stop_car();
}
