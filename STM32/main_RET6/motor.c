#include "motor.h"
#include "GPIO.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include <math.h>
#include <stdlib.h>
#include "UART.h"
#include "6050control.h"

extern uint8_t send_flag;
//counter of the encoder
uint16_t fl_counter = 0;
uint16_t fr_counter = 0;
uint16_t bl_counter = 0;
uint16_t br_counter = 0;
// all speeds are the vertical component velocity of the wheel, measured in cm/s.
float fl_speed = 0; 
float fr_speed = 0;
float bl_speed = 0;
float br_speed = 0;
//the speed postive and negetive
int8_t fl_speed_dir = 0;
int8_t fr_speed_dir = 0;
int8_t bl_speed_dir = 0;
int8_t br_speed_dir = 0;
// the target speed you want to achieve
float fl_target_speed = 0;
float fr_target_speed = 0;
float bl_target_speed = 0;
float br_target_speed = 0;
// the motor direction
uint8_t fl_direction = 0;
uint8_t fr_direction = 0;
uint8_t bl_direction = 0;
uint8_t br_direction = 0;
// the motro direction input of the gpio
uint8_t fl_dir = 0;
uint8_t fr_dir = 0;
uint8_t bl_dir = 0;
uint8_t br_dir = 0;
// use for test
uint16_t test_a = 0;
uint16_t test_b = 0;
float distance = 0; // the move of the car
extern uint16_t break_flag;
//angle pid return delta_v
extern float delta_v;
int delta_v_enable = 0; // Define the global variable. Initially set it to 0 (delta_v disabled)
//move pid value
char move_axis = 'n';
float move_target_distance;
//distance flag
uint8_t distance_flag = 0;


PID_Controller pid_fl, pid_fr, pid_bl, pid_br;  //the 4 motors pid controller
// float kp = 0.5, ki = 0.6, kd = 0.5;  // These values should be tuned for your specific system
float kp = 0.7, ki = 0.6, kd = 0.5;  // These values should be tuned for your specific system
PID_Controller pid_move;  //the distance pid controller
float move_kp = 0.225, move_ki = 0.0005, move_kd = 0.8;  // These values should be tuned for your specific system


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
  * @param  duty_cycle: the duty cycle of the pwm. you can chose negetive or positive
  * @retval None
  */
void control_motor(uint8_t motor_select,int16_t duty_cycle) 
{
    if (duty_cycle < 0)  //it mean backward
    {
        pwm_set_duty_cycle(motor_select, -duty_cycle);
        motor_dir_select(motor_select, MOTOR_BACKWARD);
    }
    else if (duty_cycle >= 0)
    {
        pwm_set_duty_cycle(motor_select, duty_cycle);
        motor_dir_select(motor_select, MOTOR_FORWARD);
    }
}

/**
  * @brief  give the target speed and dir for the motor
  * @param  speeds: [FL, FR, BL, BR] float speed of motor in cm/s - mean backward
  * @param  control_flags: [FL, FR, BL, BR] 1 for control, 0 for not control
  * @retval None
  */
void control_motor_speed( float speeds[], uint8_t control_flags[])
{
    if (control_flags[0])
    {
        fl_target_speed = speeds[0];
    }
    if (control_flags[1])
    {
        fr_target_speed = speeds[1];
    }
    if (control_flags[2])
    {
        bl_target_speed = speeds[2];
    }
    if (control_flags[3])
    {
        br_target_speed = speeds[3];
    }
}
/**
  * @brief  initialize the ETR for BR motor
  * @param  None
  * @retval None
  */
void TIM1_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM1, ENABLE);
}

/**
  * @brief  initialize the ETR for BL motor
  * @param  None
  * @retval None
  */
void TIM8_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM8, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM8, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM8, ENABLE);
}

/**
  * @brief  initialize the ETR for FL motor
  * @param  None
  * @retval None
  */
void TIM3_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  initialize the ETR for FR motor
  * @param  None
  * @retval None
  */
void TIM2_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  initialize the dir gpio input
  * @param  None
  * @retval None
  */
void dir_gpio_input_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure PC.09 (FL), PC.08 (FR), PA.04 (BL), PB.12 (BR) as input pull-up */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// the motor in the right side 0 means forward, 1 means backward, and the motor in the left side is the opposite
/**
  * @brief  read the FL motor direction
  * @param  None
  * @retval 1: forward, 0: backward
  */
uint8_t Read_FL_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9);
}

/**
  * @brief  read the FR motor direction
  * @param  None
  * @retval 0: forward, 1: backward
  */
uint8_t Read_FR_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8);
}

/**
  * @brief  read the BL motor direction
  * @param  None
  * @retval 1: forward, 0: backward
  */
uint8_t Read_BL_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
}

/**
  * @brief  read the BR motor direction
  * @param  None
  * @retval 0: forward, 1: backward
  */
uint8_t Read_BR_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
}

/**
  * @brief  initialize the TIM5 for encoder
  * @param  None
  * @retval None
  */
void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 9999;  //
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM6, ENABLE);
}



/**
  * @brief  initialize the pid control
  * @param  None
  * @retval None
  */
void pid_init(PID_Controller *pid, float kp, float ki, float kd, float setpoint)
{
    pid->setpoint = setpoint;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev0_error = 0.0;
    pid->prev1_error = 0.0;
}

/**
  * @brief  initialize the 4 pid control
  * the kp ki kd in the header file
  * @param  None
  * @retval None
  */
void init_pid(void)
{
    pid_init(&pid_fl, kp, ki, kd, fl_target_speed);
    pid_init(&pid_fr, kp, ki, kd, fr_target_speed);
    pid_init(&pid_bl, kp, ki, kd, bl_target_speed);
    pid_init(&pid_br, kp, ki, kd, br_target_speed);
    pid_init(&pid_move, move_kp, move_ki, move_kd, 0.0);
}

/**
  * @brief  pid compute
  * the increment output += to the .output structure, you can use it to control the motor
  * max and min output in the head of the file
  * @param  PID_Controller *pid: the structure of the pid controller
  * @param  float measurement: the measurement value
  * @retval None
  */
void pid_compute(PID_Controller *pid, float measurement)
{
    // test_b += 1;
    // Calculate error
    float error = pid->setpoint - measurement;
    
    // Proportional term
    float P = pid->kp * (error - pid->prev0_error);  // P = Kp * (e[n] - e[n-1]) incremental PID
    
    // Integral term
    float I = pid->ki * error;  // I = Ki * e[n]

    // Derivative term
    float D = pid->kd * (error - 2 * pid->prev0_error + pid->prev1_error);  // D = Kd * (e[n] - 2e[n-1] + e[n-2])

    // Update previous error
    pid->prev1_error = pid->prev0_error;
    pid->prev0_error = error;

    // Calculate control variable
    float calculated_value = P + I + D;
    //add to the output
    pid->output += calculated_value;
    //control the min and max of the output
    if (pid->output > MAX_OUTPUT)
        pid->output = MAX_OUTPUT;
    else if (pid->output < -MAX_OUTPUT)
        pid->output = -MAX_OUTPUT;
}

void control_move(char axis,float target_disatance)
{
  move_axis = axis;
  move_target_distance = target_disatance;
}
/**
  * @brief  the interrupt handler for TIM5 using for PID control of the motor
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
		    test_a += 1;
        if (test_a >= 100)
        {
          test_a = 0;
          test_b ++;
        }
        send_flag = 1;
        fl_dir = Read_FL_Direction();
        fr_dir = Read_FR_Direction();
        bl_dir = Read_BL_Direction();
        br_dir = Read_BR_Direction();
        // PID control code goes here.
        br_counter = TIM_GetCounter(TIM1);
        bl_counter = TIM_GetCounter(TIM8);
        fl_counter = TIM_GetCounter(TIM3);
        fr_counter = TIM_GetCounter(TIM2);
        // reset the counter
        TIM_SetCounter(TIM1, 0);
        TIM_SetCounter(TIM8, 0);
        TIM_SetCounter(TIM3, 0);
        TIM_SetCounter(TIM2, 0);
        //the spped need the postive and negative
        if (fl_dir == 1) fl_speed_dir = 1;
        else fl_speed_dir = -1;
        if (fr_dir == 0) fr_speed_dir = 1;
        else fr_speed_dir = -1;
        if (bl_dir == 1) bl_speed_dir = 1;
        else bl_speed_dir = -1;
        if (br_dir == 0) br_speed_dir = 1;
        else br_speed_dir = -1;
        //The gear ratio is 3:7, and the encoder has 1024 lines. into the ISR is 10ms
        //Therefore, the speed of wheel (vertical) is (counter/7) * 3 / 1024 * 100 * R * COS45  cm/s
        br_speed = br_speed_dir * ((float)br_counter / 7) * 3 * 100 / 1024 * COS45 * RADIUS ;
        bl_speed = bl_speed_dir *((float)bl_counter / 7) * 3 * 100 / 1024 * COS45 * RADIUS ;
        fl_speed = fl_speed_dir *((float)fl_counter / 7) * 3 * 100 / 1024 * COS45 * RADIUS ;
        fr_speed = fr_speed_dir *((float)fr_counter / 7) * 3 * 100 / 1024 * COS45 * RADIUS ;
        //get the resived speed
        // Apply PID control
        if(delta_v_enable) 
        {
            pid_fl.setpoint = fl_target_speed - delta_v;  // Update the setpoint if it has changed
            pid_fr.setpoint = fr_target_speed + delta_v;
            pid_bl.setpoint = bl_target_speed - delta_v;  // Update the setpoint if it has changed
            pid_br.setpoint = br_target_speed + delta_v;
        } else
        {
            pid_fl.setpoint = fl_target_speed;  // Update the setpoint if it has changed
            pid_fr.setpoint = fr_target_speed;
            pid_bl.setpoint = bl_target_speed;  // Update the setpoint if it has changed
            pid_br.setpoint = br_target_speed;
        }
        pid_compute(&pid_fl, fl_speed);
        control_motor(MOTOR_FL,(int)pid_fl.output);

        pid_compute(&pid_fr, fr_speed);
        control_motor(MOTOR_FR,(int)pid_fr.output);

        pid_compute(&pid_bl, bl_speed);
        control_motor(MOTOR_BL, (int)pid_bl.output);

        pid_compute(&pid_br, br_speed);
        control_motor(MOTOR_BR, (int)pid_br.output);
//        distance += (fl_speed + fr_speed + bl_speed + br_speed) * 0.01;
        //define the forward and the move to the right is positive, and the backward and the move to the left is negative
        //this car is only used for translation operations.
        //the left side forward dir is 1, the right side forward dir is 0
        if(fl_dir == 1 && bl_dir == 1 && fr_dir == 0 && br_dir == 0) // it mean forward
            distance += ((float)br_counter + (float)bl_counter + (float)fl_counter + (float)fr_counter)/
            7 * 3 / 1024 * 5.2;//* 2 * PI * RADIUS * COS45 ;  //
        else if (fl_dir == 0 && bl_dir == 0 && fr_dir == 1 && br_dir == 1) // it mean backward
            distance -= ((float)br_counter + (float)bl_counter + (float)fl_counter + (float)fr_counter)/
            7 * 3 / 1024 * 5.2;//* 2 * PI * RADIUS * COS45 ;  //
        else if (fl_dir == 1 && bl_dir == 0 && fr_dir == 1 && br_dir == 0) // it mean move to the right
            distance += ((float)br_counter + (float)bl_counter + (float)fl_counter + (float)fr_counter)/
            7 * 3 / 1024 * 4.6;//* 2 * PI * RADIUS * COS45 ;  //
        else if (fl_dir == 0 && bl_dir == 1 && fr_dir == 0 && br_dir == 1) // it mean move to the left
            distance -= ((float)br_counter + (float)bl_counter + (float)fl_counter + (float)fr_counter)/
            7 * 3 / 1024 * 4.6;//* 2 * PI * RADIUS * COS45 ;  //
        else
            distance += 0;
		// break_flag ++;
    //below are the distance PID
    pid_move.setpoint = move_target_distance;
    pid_compute(&pid_move, distance);
    if(abs(pid_move.setpoint - distance) < DSITANCE_THRESHOLD)
    { 
      // distance_flag += 1;
      // distance = 0;
      // move_target_distance = 0;
      pid_move.output = 0; // Close enough to the target.
    }
    // axis X
    if (move_axis == 'x')
    {
        fl_target_speed = pid_move.output;
        fr_target_speed = pid_move.output;
        bl_target_speed = pid_move.output;
        br_target_speed = pid_move.output;
    }
    else if( move_axis == 'y')
    {
        move_kp = 0.32;
        move_ki = 0.0;
        move_kd = 0.0;  // y axis value of pid
        fl_target_speed = pid_move.output;
        fr_target_speed = -pid_move.output;
        bl_target_speed = -pid_move.output;
        br_target_speed = pid_move.output;
    }
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update); // Clear the interrupt flag
    }
}

/**
  * @brief  enanble or disable the delta_v
  * @param  None
  * @retval None
  */
void toggle_delta_v(int enable)
{
    delta_v_enable = enable;
}

