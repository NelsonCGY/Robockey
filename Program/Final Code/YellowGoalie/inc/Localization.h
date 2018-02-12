#ifndef Localization__
#define Localization__

#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "m_wii.h"

void calculate_pos(int * goal);

void go_for_goal(int * goal);

char go_to_goal(int * goal);

char compensator_indicator(void);

#endif 
