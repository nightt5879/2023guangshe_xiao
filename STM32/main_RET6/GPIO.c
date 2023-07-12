#include "GPIO.h"
#include "stm32f10x.h"                  // Device header

/**
  * @brief  Initializes TIM4 pwm output
  * @retval None
  */
void pwm_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //enable the internal clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //Enable GPIOB Clock
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	
	//Init All The PWM IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //TIM2_CH1,2,3,4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM4);  
	
	//All Timer is the same 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;		//ARR pwm frequency = 72MHz/1000 = 72KHz
	TIM_TimeBaseInitStructure.TIM_Prescaler = 2 - 1;		//PSC  do not pre-divide the frequency
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;  // do not use the repetition counter
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);  //init the timer
	
	//All The Servo is the Same Mod
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR start from 0 duty cycle
	
	//The 4 Servoes Init
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_Cmd(TIM4, ENABLE);
}

/**
  * @brief  change the pwm duty cycle
  * @param CHx: the channel of the pwm
  * @param Compare: the duty cycle
  * @retval None
  */
void pwm_set_duty_cycle(uint8_t CHx, uint16_t Compare)  // change the duty cycle 1 to 4 CHx
{
    switch (CHx){
        case 1:TIM_SetCompare1(TIM4, Compare);break;
        case 2:TIM_SetCompare2(TIM4, Compare);break;
        case 3:TIM_SetCompare3(TIM4, Compare);break;
        case 4:TIM_SetCompare4(TIM4, Compare);break;
        default:break;
    }
}

/**
  * @brief  Initializes motor direction gpio
  * @retval None
  */
void motor_dir_gpio_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	//Change the mapping of specified pins GPIO_Remap_SWJ_Disable completely disables SWJ (JTAG+SW-DP)
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	// Change the mapping of the specified pin GPIO_Remap_SWJ_JTAGDisable, disable JTAG-DP + enable SW-DP
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

	//init all the dir gpio
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_4); 	
    //make all the IO output low level
    GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
    GPIO_ResetBits(GPIOC, GPIO_Pin_12);
}

/**
  * @brief  change the motor direction
  * @param motor_select: the motor to change direction, here 1,2,3,4 mean the FL FB BL BR 
  *                     (forward_left, forward_back, back_left, back_right)
  * @param diration: the direction to change, here 1 means forward, 0 means back
  * @retval None
  */
void motor_dir_select(uint8_t motor_select, uint8_t diration)
{
    if (motor_select == 1) 
    {
        if (diration == 1) //forward
        {
			GPIO_ResetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_5); 	
        }
        else //backward
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_5); 	
        }
    }
    else if (motor_select == 2)
    {
        if (diration == 1)  //forward
        {
			GPIO_ResetBits(GPIOC, GPIO_Pin_12); 	
			GPIO_ResetBits(GPIOB, GPIO_Pin_3); 	
        }
        else //backward
        {
			GPIO_SetBits(GPIOC, GPIO_Pin_12); 	
			GPIO_SetBits(GPIOB, GPIO_Pin_3); 
        }
    }
    else if (motor_select == 3)
    {
        if (diration == 1)  //forward
        {
			GPIO_SetBits(GPIOA, GPIO_Pin_10 |GPIO_Pin_11); 	 	
        }
        else //backward
        {
			GPIO_ResetBits(GPIOA, GPIO_Pin_10 |GPIO_Pin_11); 
        }
    }
    else if (motor_select == 4)
    {
        if (diration == 1)  //forward
        {
			GPIO_SetBits(GPIOA, GPIO_Pin_8 |GPIO_Pin_9); 	 	
        }
        else //backward
        {
			GPIO_ResetBits(GPIOA, GPIO_Pin_8 |GPIO_Pin_9); 
        }
    }
    else
    {
		//it mean no the right select for motor do nothing
    }
}
