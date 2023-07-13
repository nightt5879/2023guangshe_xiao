#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
void motor_init(void);
void control_motor(uint8_t motor_select, uint8_t direction, uint16_t duty_cycle) ;
void TIM1_ETR_Config(void);
void TIM8_ETR_Config(void);
void TIM3_ETR_Config(void);
void TIM2_ETR_Config(void);
void dir_gpio_input_Config(void);
uint8_t Read_FL_Direction(void);
uint8_t Read_FR_Direction(void);
uint8_t Read_BL_Direction(void);
uint8_t Read_BR_Direction(void);


#define MOTOR_FL 1
#define MOTOR_FR 2
#define MOTOR_BL 3
#define MOTOR_BR 4
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0
	
#endif
