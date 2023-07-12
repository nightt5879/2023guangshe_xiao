#include "motor.h"
#include "GPIO.h"

/**
  * @brief  initialize the motor include the pwm and the direction gpio
  * @param  None
  * @retval None
  */
void motor_init(void)
{
	pwm_init();
	motor_dir_gpio_init();
}

/**
  * @brief  control the motor
  * @param  motor_select: the motor you want to control the define can find in motor.h
  * @param  direction: the direction of the motor  the define can find in motor.h
  * @param  duty_cycle: the duty cycle of the pwm
  * @retval None
  */
void control_motor(uint8_t motor_select, uint8_t direction, uint16_t duty_cycle) 
{
    pwm_set_duty_cycle(motor_select, duty_cycle);
    motor_dir_select(motor_select, direction);
}
	
