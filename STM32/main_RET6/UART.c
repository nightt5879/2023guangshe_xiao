#include "UART.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <string.h>
#include "motor.h"

int16_t distance_x_uart, distance_y_uart, speed, correction_speed;
uint8_t one_move_flag = 0;
extern float fl_speed, fr_speed, bl_speed, br_speed;
extern float distance_x_encoder, distance_y_encoder, angle_z_encoder;
extern float target_distance_x, target_distance_y;
extern float fl_target_speed, fr_target_speed, bl_target_speed, br_target_speed;
extern float distance_x_filter, distance_y_filter, move_target_distance_x, move_target_distance_y;
extern int16_t distance_x_uart, distance_y_uart; // send the distance to the motor

extern int16_t correction_speed;
/**
  * @brief  init USART4
  * @retval None
  */
void UART4_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // enable peripheral clock for USART4
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // set the TX pin as AF push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // set the RX pin as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // UART4 configuration
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
    
    // enable the USART4
    USART_Cmd(UART4, ENABLE);
}

/**
  * @brief  the data structure to be sent
  * @retval None
  */
struct Frame {
    float fdata[CH_COUNT];  // the channel data you can find in the header file
    unsigned char tail[4];
};

void USART_SendFrame(USART_TypeDef* USARTx, struct Frame *frame) {
    uint8_t *p = (uint8_t*)frame;
    for(int i = 0; i < sizeof(struct Frame); i++) {
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);  // wait for the data register to be empty
        USART_SendData(USARTx, *p++);
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);  // wait for the transmission to complete
    }
}

void send_data(float *data, uint8_t size) {
    struct Frame frame;
    size = size < CH_COUNT ? size : CH_COUNT;
    memcpy(frame.fdata, data, size * sizeof(float));
    unsigned char tail[4] = TAIL;
    memcpy(frame.tail, tail, sizeof(tail));
    USART_SendFrame(UART4, &frame);
}

uint8_t Serial_TxPacket[6];				//FF 01 02 03 04 FE
uint8_t Serial_RxPacket[6];
uint8_t Serial_RxFlag;

void Serial_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  //Enable the USART2 periph clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //Enable the GPIOA periph clock

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

//void Serial_Init(void)
//{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	USART_InitTypeDef USART_InitStructure;
//	USART_InitStructure.USART_BaudRate = 115200;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_Init(USART1, &USART_InitStructure);
//	
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	USART_Cmd(USART1, ENABLE);
//}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

//int fputc(int ch, FILE *f)
//{
//	Serial_SendByte(ch);
//	return ch;
//}

//void Serial_Printf(char *format, ...)
//{
//	char String[100];
//	va_list arg;
//	va_start(arg, format);
//	vsprintf(String, format, arg);
//	va_end(arg);
//	Serial_SendString(String);
//}


void Serial_SendPacket(void)
{
	Serial_SendByte(0xFF);
	Serial_SendArray(Serial_TxPacket, 6);
	Serial_SendByte(0xFE);
}

uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

void USART2_IRQHandler(void)
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART2);
        if (RxState == 0)
        {
            if (RxData == 0xFF)
            {
                RxState = 1;
                pRxPacket = 0;
            }
        }
        else if (RxState == 1)
        {
            Serial_RxPacket[pRxPacket] = RxData;
            pRxPacket ++;
            if (pRxPacket >= 6)
            {
                RxState = 2;
            }
        }
        else if (RxState == 2)
        {
            if (RxData == 0xFE) // here mean the right data
            {
                RxState = 0;
                Serial_RxFlag = 1;
                if (Serial_RxPacket[0] == 0x07) // forward to the corner
                {
                    distance_y_uart =  Serial_RxPacket[1]; // positive
                    speed = Serial_RxPacket[2];
                    correction_speed = Serial_RxPacket[3];
                    // fl_target_speed = speed;
                    // fr_target_speed = speed;
                    // bl_target_speed = speed;
                    // br_target_speed = speed;
                }
                else if (Serial_RxPacket[0] == 0x08) // backward to the corner
                {
                    distance_y_uart =  -Serial_RxPacket[1];  //negetive
                    speed = Serial_RxPacket[2];
                    correction_speed = Serial_RxPacket[3];
                    // fl_target_speed = -speed;
                    // fr_target_speed = -speed;
                    // bl_target_speed = -speed;
                    // br_target_speed = -speed;
                }
                else if (Serial_RxPacket[0] == 0x09) // move left to the corner
                {
                    distance_x_uart =  -Serial_RxPacket[1]; // negetive
                    speed = Serial_RxPacket[2];
                    correction_speed = Serial_RxPacket[3];
                    // fl_target_speed = -speed;
                    // fr_target_speed = speed;
                    // bl_target_speed = speed;
                    // br_target_speed = -speed;
                }
                else if (Serial_RxPacket[0] == 0xA9) // move left to the corner
                {
                    distance_y_uart =  Serial_RxPacket[1]; // positive
                    speed = Serial_RxPacket[2];
                    correction_speed = Serial_RxPacket[3];
                    // fl_target_speed = speed;
                    // fr_target_speed = -speed;
                    // bl_target_speed = -speed;
                    // br_target_speed = speed;
                }
                else if (Serial_RxPacket[0] == 0x0B)  // send the distance positve
                {
                    one_move_flag = 1;
                    stop_the_car();
                    distance_y_uart = Serial_RxPacket[1]; 
                    distance_x_uart = Serial_RxPacket[2];
                }
                else if (Serial_RxPacket[0] == 0x0C)
                {
                    one_move_flag = 1;
                    stop_the_car();
                    distance_y_uart = -Serial_RxPacket[1];
                    distance_x_uart = -Serial_RxPacket[2];
                }
                else if (Serial_RxPacket[0] == 0x0D) // control the MPU 6050
                {
                    if (Serial_RxPacket[1] == 0) // stop the angle and correction
                    {
                        toggle_delta_v(0);
                        mpu_6050_corretion();
                    }
                    else if (Serial_RxPacket[1] == 1)
                    {
                        toggle_delta_v(1);
                    }
                }
			Serial_TxPacket[1] = Serial_RxPacket[1];
			Serial_TxPacket[2] = Serial_RxPacket[2];
			Serial_TxPacket[3] = Serial_RxPacket[3];
			Serial_TxPacket[4] = Serial_RxPacket[4];
			Serial_TxPacket[5] = Serial_RxPacket[5];
            }
		
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

