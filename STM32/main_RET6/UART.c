#include "UART.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <string.h>

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
