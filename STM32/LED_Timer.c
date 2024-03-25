#include "stm32f303xe.h"
#include "GPIO.h"
#include "Timer.h"

void SysClockConfig(void);
void delay(void);
	
void SysClockConfig(void){
	
	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY));
	RCC->CR |= (1<<7);
	
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	RCC->CFGR |= 0;
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
}

void delay(void){
	TIM3->CNT=0;
	while((TIM3->CNT < 0x0000007a));
}

int main(void){
	

	
	TIM_Config_t Time;
	SysClockConfig();
	
	Time.pTIMx = TIM3;
	Time.OPM = OPM_OFF;
	Time.DIR = DIR_UP;
	Time.CMS = CMS_Edge;
	Time.ARPE = ARPE_EN;
	Time.PSC = 0xffff;
	Time.ARR = 0x7a;
	Timer_Config(&Time);
	GPIO_assign(GPIOB, 0, GPIO_MODE_OUT);
	GPIO_assign(GPIOB, 7, GPIO_MODE_OUT);
	GPIO_assign(GPIOB, 14, GPIO_MODE_OUT);
	Counter_Start(&Time);

	
	while(1){
			
			GPIO_Write(GPIOB, 0, 1);
			delay();
		
			GPIO_Write(GPIOB, 0, 0);
			delay();

	}
}	