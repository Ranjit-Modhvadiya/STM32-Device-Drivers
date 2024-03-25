#include "GPIO.h"

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	
  uint32_t mode = 0;
	uint32_t type = 0;
	uint32_t speed = 0;
	uint32_t pupd = 0;
	
	mode = (uint32_t)(pGPIOHandle->config.PIN_MODE << (2*pGPIOHandle->config.PIN_NO));
	pGPIOHandle->pGPIOx->MODER &= (uint32_t)~(0x3<< (2*pGPIOHandle->config.PIN_NO));
	pGPIOHandle->pGPIOx->MODER |= mode;	
	
	type = (uint32_t)(pGPIOHandle->config.OUT_TYPE << pGPIOHandle->config.PIN_NO);
	pGPIOHandle->pGPIOx->OTYPER &= (uint32_t)~(1 << pGPIOHandle->config.PIN_NO);
	pGPIOHandle->pGPIOx->OTYPER |= type;
	
	speed = (uint32_t)(pGPIOHandle->config.OUT_SPEED << (2*pGPIOHandle->config.PIN_NO));
	pGPIOHandle->pGPIOx->OSPEEDR &= (uint32_t)~(0x3 << pGPIOHandle->config.PIN_NO);
	pGPIOHandle->pGPIOx->OSPEEDR |= speed;
	
	pupd = (uint32_t)(pGPIOHandle->config.PUPD << (2*pGPIOHandle->config.PIN_NO));
	pGPIOHandle->pGPIOx->PUPDR &= (uint32_t)~(0x3 << pGPIOHandle->config.PIN_NO);
	pGPIOHandle->pGPIOx->PUPDR |= pupd;
	
}

void GPIO_Write(GPIO_TypeDef *pGPIOx, uint8_t PIN_NO, bool val){
	
	if (val == 1)
	{	
		pGPIOx->BSRR |= (1 << PIN_NO);
	}
	
	if (val == 0)
	{	
		pGPIOx->BSRR |= (1 << (16 + PIN_NO));
	}

}

void GPIO_Assign(GPIO_TypeDef *GPIOx, uint8_t N, uint8_t Mode, uint8_t pupd){
	
	GPIO_Handle_t Name;
	Name.pGPIOx = GPIOx;
	Name.config.PIN_NO = N;
	Name.config.PIN_MODE = Mode;
	Name.config.PUPD = pupd;
	GPIO_Init(&Name);
	
}

void GPIO_Toggle(GPIO_TypeDef *pGPIOx, uint8_t PIN_NO){

 //(pGPIOx->ODR & (1 << PIN_NO)) ? (pGPIOx->ODR = (0 << (PIN_NO))) : (pGPIOx->ODR = (1 << (PIN_NO)));
 pGPIOx->ODR = (pGPIOx->ODR ^ (1<<PIN_NO));
	
}
