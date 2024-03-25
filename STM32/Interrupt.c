#include "Interrupt.h"

void Int_Config(INT_Config_t *INT){
	
	uint8_t x = 0;
	uint8_t y;
	y = INT->Line;

		switch(INT->Port)
		{
			case A:
				x = 0;
				break;
		
			case B:
				x = 1;
				break;
			
			case C:
				x = 2;
				break;
			
			case D:
				x = 3;
				break;
			
			case E:
				x = 4;
				break;
			
			case F:
				x = 5;
				break;
			
			case G:
				x = 6;
				break;
			
		}
		
		SYSCFG->EXTICR[(y/4)]	|= (x << 4*y);

}

void Int_EdgeSel(INT_Config_t *INT){
	
	if (INT->Edge == RisingEdge){
		
		EXTI->RTSR |= (1 << INT->Line);
		
	}
	
	if (INT->Edge == FallingEdge)
	{
		EXTI->FTSR |= (1 << INT->Line);
	}
	
}

void Int_Enable(INT_Config_t *INT){
	
	EXTI->IMR |= (1 << INT->Line);
	
}

void Int_EventEn(INT_Config_t *INT){
	
	EXTI->EMR |= (1 << INT->Line);
	
}

void Int_SWInt(INT_Config_t *INT){
	
	EXTI->SWIER |= (1 << INT->Line);
	
}

void Int_ClearPend(INT_Config_t *INT){
	
	EXTI->PR |= (1 << INT->Line);

}
		