#include "Timer.h"

void Timer_Config(TIM_Config_t *Time){
	
	TIM3->CR1 = 0;
	TIM3->PSC = 0;

	Time->pTIMx->CR1 |= (Time->OPM << 3);
	Time->pTIMx->CR1 |=	 (Time->DIR << 4);
	Time->pTIMx->CR1 |= (Time->ARPE << 7);
	
	Time->pTIMx->PSC = Time->PSC;
	Time->pTIMx->ARR = Time->ARR;
	
}

void Counter_Start(TIM_Config_t *Time){
	
	Time->pTIMx->CR1 |= (1 << 0);

}

void Counter_Stop(TIM_Config_t *Time){
	Time->pTIMx->CR1 &= ~(1 << 0);

}