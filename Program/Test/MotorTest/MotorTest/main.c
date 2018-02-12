/*
 * MotorTest.c
 *
 * Created: 11/17/2016 5:27:58 PM
 * Author : gongyaoc
 */ 
#define F_CPU 16000000
#include <avr/io.h>
#include "m_general.h"

void init_timer1(){
	OCR1A = 200;

	clear(TCCR1B, CS12);		// prescalar /1, 16M
	clear(TCCR1B, CS11);
	set(TCCR1B, CS10);

	set(TCCR1B, WGM13);			// (mode 15) UP to OCR1A, PWM mode
	set(TCCR1B, WGM12);
	set(TCCR1A, WGM11);
	set(TCCR1A, WGM10);

	
	set(TCCR1A, COM1B1);		// clear at OCR1B, set at rollover
	clear(TCCR1A, COM1B0);
	set(TCCR1A, COM1C1);
	clear(TCCR1A, COM1C0);		// clear at OCR1C, set at rollover


	set(DDRB, 6);				// left PWM OCR1B
	set(DDRB, 7);				// right PWM OCR1C
}

int main(void)
{
	set(DDRB, 3);			// B3 left
	set(DDRB, 2);			// B2 right
	init_timer1();
	OCR1B = 0.60 * OCR1A;
	OCR1C = 0.60 * OCR1A;
    /* Replace with your application code */
    while (1) 
    {
		set(PORTB,3);
		set(PORTB,2);
		for(int i = 0; i<30000; i++){
			for(int j = 0; j< 50; j++){}
		}
		m_red(TOGGLE);
		clear(PORTB,3); // R reverse
		set(PORTB,2);
		for(int i = 0; i<30000; i++){
			for(int j = 0; j< 50; j++){}
		}
		m_red(TOGGLE);
		clear(PORTB,3); // R reverse
		clear(PORTB,2); // L reverse
		for(int i = 0; i<30000; i++){
			for(int j = 0; j< 50; j++){}
		}
		m_red(TOGGLE);
    }
	
}

