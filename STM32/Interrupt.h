#ifndef _Interrupt_H
#define _Interrupt_H

#include "stm32f303xe.h"
#include "GPIO.h"

#define RisingEdge 1
#define FallingEdge 0

#define A 0
#define B 1
#define C 2
#define D 3
#define E 4
#define F 5
#define G 6

typedef struct{
	
	uint8_t Line;
	uint8_t Port;
	uint8_t Edge;
	
} INT_Config_t;

void Int_Config(INT_Config_t *INT);
void Int_EdgeSel(INT_Config_t *INT);
void Int_Enable(INT_Config_t *INT);
void Int_EventEn(INT_Config_t *INT);
void Int_SWInt(INT_Config_t *INT);
void Int_ClearPend(INT_Config_t *INT);

#endif