#ifndef __GPIO_H
#define __GPIO_H
#include <stdint.h>

void pwm_init(void);
void pwm_set_duty_cycle(uint8_t CHx, uint16_t Compare);
void motor_dir_gpio_init(void);
void motor_dir_select(uint8_t motor_select, uint8_t diration);

#define MOTOR_FL 1
#define MOTOR_FR 2
#define MOTOR_BL 3
#define MOTOR_BR 4
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0

#endif
