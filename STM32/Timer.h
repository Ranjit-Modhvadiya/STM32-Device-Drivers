#ifndef _Timer_H
#define _Timer_H

#include "stm32f303xe.h"

#define OPM_OFF  0
#define OPM_ON   1

#define DIR_UP   0
#define DIR_DOWN 1

#define CMS_Edge    0
#define CMS_Centre1 1
#define CMS_Centre2 2
#define CMS_Centre3 3

#define ARPE_DIS 0
#define ARPE_EN  1

typedef struct{
	
	uint8_t OPM;
	uint8_t DIR;
	uint8_t CMS;
	uint8_t ARPE;
	uint16_t PSC;
	uint32_t ARR;
	TIM_TypeDef *pTIMx;
	
} TIM_Config_t;

void Timer_Config(TIM_Config_t *Time);
void Counter_Start(TIM_Config_t *Time);
void Counter_Stop(TIM_Config_t *Time);

#endif
