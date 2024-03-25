
#include "GPIO.h"
#include "Interrupt.h"

INT_Config_t INT;

void SysClockConfig(void);

void SysClockConfig(void){
	
	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY));
	RCC->CR |= (1<<7);
	 
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	RCC->CFGR |= 0;
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
}

int main(void){
	
	SysClockConfig();
	
	GPIO_Assign(GPIOB, 0, GPIO_MODE_OUT, GPIO_PUPD_PU);
	GPIO_Assign(GPIOB, 7, GPIO_MODE_OUT, GPIO_PUPD_PU);
	GPIO_Assign(GPIOB, 14, GPIO_MODE_OUT, GPIO_PUPD_PU);
	GPIO_Assign(GPIOC, 0, GPIO_MODE_IN, GPIO_PUPD_PD);
	

	INT.Line = 0;
	INT.Port = C;
	INT.Edge = RisingEdge;
	
	Int_Config(&INT);
	Int_EdgeSel(&INT);
	Int_Enable(&INT);
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	GPIO_Write(GPIOB, 0, 1);
	GPIO_Write(GPIOB, 7, 0);
	GPIO_Write(GPIOB, 14, 1);
	
	while(1)
	{
		
	}	
	
}

void EXTI0_IRQHandler(void)
{
	GPIO_Toggle(GPIOB, 0);
	GPIO_Toggle(GPIOB, 7);
	GPIO_Toggle(GPIOB, 14);
	Int_ClearPend(&INT);
}
