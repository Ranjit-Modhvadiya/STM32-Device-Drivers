#include "stm32f303xe.h"
#include "led_blink.h"
#include "GPIO.h"



int harsh=0;
void SysClockConfig(void);
void GPIOConfig(char Port, int Pin, char Mode);
void delay(void);
void GPIO_assign(GPIO_TypeDef *GPIOx, uint8_t N, uint8_t Mode);

void SysClockConfig(void){
	
	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY));
	RCC->CR |= (1<<7);
	
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	RCC->CFGR |= 0;
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
}

void delay(void){
	
	int volatile i,j;
	for(i=0;i<500;i++)
	{
		for(j=0;j<500;j++)
		{
		}
	}
}

int main(void){
	
	SysClockConfig();
	
	GPIO_assign(GPIOB, 14, GPIO_MODE_OUT);
	


	while(1)
	{
			GPIO_Write(GPIOB, 14, 1);
			delay();
			GPIO_Write(GPIOB, 14, 0);
			delay();
		}
}
