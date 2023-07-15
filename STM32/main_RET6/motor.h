#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
void motor_init(void);
void control_motor(uint8_t motor_select,int16_t duty_cycle); 
void control_motor_speed(float speeds[], uint8_t control_flags[]);
void init_pid(void);
void TIM1_ETR_Config(void);
void TIM8_ETR_Config(void);
void TIM3_ETR_Config(void);
void TIM2_ETR_Config(void);
void dir_gpio_input_Config(void);
uint8_t Read_FL_Direction(void);
uint8_t Read_FR_Direction(void);
uint8_t Read_BL_Direction(void);
uint8_t Read_BR_Direction(void);
void TIM6_Configuration(void);
void TIM6_IRQHandler(void);
void toggle_delta_v(int enable);


#define MOTOR_FL 1
#define MOTOR_FR 2
#define MOTOR_BL 3
#define MOTOR_BR 4
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0

#define RADIUS 3.15f  //the radius of the wheel cm
#define PI 3.14f  //the pi
#define COS45 0.707f //cos45 for the speed calculate
#define MAX_OUTPUT 500 //max pwm
#define MIN_OUTPUT 0  // min pwm
/**
  * @brief  initialize the pid control structure
  * @param  None
  * @retval None
  */
typedef struct 
{
    float setpoint;         // Desired value
    float kp, ki, kd;       // PID coefficients
    float prev0_error;       // last time error
    float prev1_error;       // last last time error
    float output;           // the add output
} PID_Controller;
extern PID_Controller pid_fl, pid_fr, pid_bl, pid_br;
	
#endif
