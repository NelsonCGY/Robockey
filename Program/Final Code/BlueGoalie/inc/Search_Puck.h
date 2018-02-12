#ifndef Search_Puck__
#define Search_Puck__

#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "Localization.h"

void init_ADC(void);
//Initialize ADC conversion

char conversion_ADC();
//Cycle through F1, F4~7 -> ADC_V[5]

void go_for_puck(int * goal);

char conversion_ADC_stalker(void);

int determine_catch(void);

int max(int * array);	//max from 1 -> 4

void compensator(void);

void stalker(void);

#endif 
