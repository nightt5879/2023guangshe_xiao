#include "stm32f10x.h"                  // Device header
#include "6050control.h"
#include <math.h>
#include <stdlib.h>
#include "MPU6050.h"

uint8_t ID;
int16_t AX, AY, AZ, GX, GY, GZ;
float ax, ay, az, gx, gy, gz;
int16_t AX_CORR, AY_CORR, AZ_CORR, GX_CORR, GY_CORR, GZ_CORR;
int16_t ax_corr_done, ay_corr_done, az_corr_done, gx_corr_done, gy_corr_done, gz_corr_done;
float ax_prev = 0, ay_prev = 0, az_prev = 0, gx_prev = 0, gy_prev = 0, gz_prev = 0;
float now_z = 0;

/**
  * @brief  low pass filter
  * @retval None
  */
float low_pass_filter(float input, float prev) {
    return ALPHA * input + (1.0 - ALPHA) * prev;
}

/**
  * @brief  get the rightnow data of the 6050
  * @retval None
  */
void mpu_6050_corretion(void)
{
	uint8_t i = 0;
	//let the 6050 stable
	while(i < 10)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		i++;
	}
	i = 0;
	// get 2 times data and get the average
	while(i < 2)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		AX_CORR += AX;
		AY_CORR += AY;
		AZ_CORR += AZ - GRAVITY; // the g of the gravity
		GX_CORR += GX;
		GY_CORR += GY;
		GZ_CORR += GZ;
		i++;
	}
	AX_CORR /= 2;
	AY_CORR /= 2;
	AZ_CORR /= 2;
	GX_CORR /= 2;
	GY_CORR /= 2;
	GZ_CORR /= 2;
}

void init_6050(void)
{
	ID = MPU6050_GetID();
	mpu_6050_corretion();
	mpu_6050_corretion();
}

void get_6050_data(void)
{
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
	// Check the absolute values of the corrected data and set small values to 0.
	ax_corr_done = abs(AX - AX_CORR) < LOW_PASS_FILTR ? 0 : AX - AX_CORR;
	ay_corr_done = abs(AY - AY_CORR) < LOW_PASS_FILTR ? 0 : AY - AY_CORR;
	az_corr_done = abs(AZ - AZ_CORR - GRAVITY) < LOW_PASS_FILTR ? 0 : AZ - AZ_CORR - GRAVITY;
	gx_corr_done = abs(GX - GX_CORR) < LOW_PASS_FILTR ? 0 : GX - GX_CORR;
	gy_corr_done = abs(GY - GY_CORR) < LOW_PASS_FILTR ? 0 : GY - GY_CORR;
	gz_corr_done = abs(GZ - GZ_CORR) < LOW_PASS_FILTR ? 0 : GZ - GZ_CORR;
	//low pass filter
	// Use the low pass filter function on the corrected data.
	ax = low_pass_filter(ax_corr_done / 32768.0 * 2.0, ax_prev);
	ay = low_pass_filter(ay_corr_done / 32768.0 * 2.0, ay_prev);
	az = low_pass_filter(az_corr_done / 32768.0 * 2.0, az_prev);
	gx = low_pass_filter(gx_corr_done / 32768.0 * 250.0, gx_prev);
	gy = low_pass_filter(gy_corr_done / 32768.0 * 250.0, gy_prev);
	gz = low_pass_filter(gz_corr_done / 32768.0 * 250.0, gz_prev);

	// Update the previous values for the next loop.
	ax_prev = ax;
	ay_prev = ay;
	az_prev = az;
	gx_prev = gx;
	gy_prev = gy;
	gz_prev = gz;
}

/**
  * @brief  TIM7 configuration
  * @retval None
  */
void TIM7_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 999;  // 1ms
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7, ENABLE);
}

/**
  * @brief  TIM7 interrupt handler
  * @retval None
  */
void TIM7_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {

        // Clear interrupt flag.
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    }
}
