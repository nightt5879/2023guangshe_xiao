#ifndef __UART_H
#define __UART_H
#include <stdint.h>

void UART4_Init(void);
void send_data(float *data, uint8_t size);

#define CH_COUNT 22  // the max channel count
#define TAIL {0x00, 0x00, 0x80, 0x7f}  // Data frame end


#endif
