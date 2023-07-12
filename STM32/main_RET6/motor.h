#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
void motor_init(void);
void control_motor(uint8_t motor_select, uint8_t direction, uint16_t duty_cycle) ;

#define MOTOR_FL 1
#define MOTOR_FR 2
#define MOTOR_BL 3
#define MOTOR_BR 4
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0
	
#endif
