#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare1);
void PWM_SetCompare2(uint16_t Compare2);
void PWM_SetCompare3(uint16_t Compare3);
void PWM_SetCompare4(uint16_t Compare4);
void PWM_SetCompara(uint8_t CHx, uint16_t Compare);
void PWM_Test(void);
void PWM_SetTest(uint8_t CHx, uint16_t Compare);
void PWM_Test3(void);

#endif
