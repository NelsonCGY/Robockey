/*
 * mrf.c
 *
 * Created: 12/7/2016 4:53:08 PM
 * Author : CNelson
 */ 
#define F_CPU 16000000

#include <avr/io.h>
#include "m_general.h"
#include "m_bus.h"
#include "m_rf.h"

#define CHANNEL 1
#define PACKET_LENGTH 10
#define RXADDRESS	0x10
unsigned char buffer[PACKET_LENGTH] = {0,0,0,0,0,0,0,0,0,0};


int main(void)
{
	m_rf_open(CHANNEL, RXADDRESS, PACKET_LENGTH);
	buffer[0] = 0xA0;

    /* Replace with your application code */
    while (1) 
    {
		m_wait(1000);
		m_rf_send(0x1C, buffer, PACKET_LENGTH);
		for(int i = 0; i<30000; i++){
			for(int j = 0; j< 50; j++){}
		}
		m_red(TOGGLE);
    }
}

