/*
 * mwiitest.c
 *
 * Created: 12/2/2016 7:38:24 PM
 * Author : CNelson
 */ 
#define F_CPU 16000000

#include <avr/io.h>
#include "m_general.h"
#include "m_wii.h"
#include "m_bus.h"

int main(void)
{
	if(m_wii_open()){
		m_green(ON);
	}
	else{
		m_red(ON);
	}
    /* Replace with your application code */
    while (1) 
    {
		
    }
}

