#ifndef _GPIO_H
#define _GPIO_H

#include "stm32f303xe.h"
#include "stdbool.h"

#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_AF			2
#define GPIO_MODE_ANA   	3

#define GPIO_OUT_TYPE_PP  0
#define GPIO_OUT_TYPE_OD  1

#define GPIO_OUT_SPEED_LOW   0
#define GPIO_OUT_SPEED_MED   1
#define GPIO_OUT_SPEED_HIGH  3

#define GPIO_PUPD_NO  0
#define GPIO_PUPD_PU  1
#define GPIO_PUPD_PD  2

typedef struct{
	
	uint8_t PIN_NO;
	uint8_t PIN_MODE;
	uint8_t OUT_TYPE;
	uint8_t OUT_SPEED;
	uint8_t PUPD;
	
}GPIO_Config;

typedef struct{
	
	GPIO_TypeDef *pGPIOx;
	GPIO_Config config;
	
}GPIO_Handle_t;
	

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Write(GPIO_TypeDef *pGPIOx, uint8_t PIN_NO, bool val);
void GPIO_assign(GPIO_TypeDef *GPIOx, uint8_t N, uint8_t Mode);

#endif
