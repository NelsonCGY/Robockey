/*
 * ADCTest.c
 *
 * Created: 11/17/2016 5:48:39 PM
 * Author : gongyaoc
 */ 
#define F_CPU 16000000
#include <avr/io.h>
#include "m_general.h"
#include "m_usb.h"

int ADC_V[6];

void init_ADC()
{
	m_disableJTAG();		// Enable adc access to F4-7

	clear(ADMUX, REFS1);	// voltage reference to Vcc
	set(ADMUX, REFS0);

	set(ADCSRA,ADPS0);		// prescaler to /128 -> 125kHz
	set(ADCSRA,ADPS1);
	set(ADCSRA,ADPS2);

	set(DIDR0,ADC0D);		// disabling digital inputs for F0
	set(DIDR0,ADC1D);		// disabling digital inputs for F1
	set(DIDR0,ADC4D);		// disabling digital inputs for F4
	set(DIDR0,ADC5D);		// disabling digital inputs for F5
	set(DIDR0,ADC6D);		// disabling digital inputs for F6
	set(DIDR0,ADC7D);		// disabling digital inputs for F7

	set(ADCSRA, ADIE);		// interrupt

	clear(ADMUX, MUX2);		// set F1 as input, 0001
	clear(ADMUX, MUX1);
	set(ADMUX, MUX0);

	set(ADCSRA, ADEN);		//enable conversion
	set(ADCSRA, ADSC);		//begin conversion
}

void conversion_ADC(){
	static int adc_port = 0;
	ADC_V[adc_port] = ADC;
	adc_port++;
	if (adc_port > 5)
	{
		adc_port = 0;
	}
	clear(ADCSRA,ADEN);
	switch(adc_port) {
		case 0:
		clear(ADMUX, MUX2);		// set F0 as input, 0000
		clear(ADMUX, MUX1);
		clear(ADMUX, MUX0);
		break;
		case 1:
		set(ADMUX, MUX0);		// set F1 as input, 0001
		break;
		case 2:
		clear(ADMUX, MUX0);		// set F4 as input, 0100
		set(ADMUX, MUX2);
		break;
		case 3:
		set(ADMUX, MUX0);		// set F5 as input, 0101
		break;
		case 4:
		clear(ADMUX, MUX0);		// set F6 as input, 0110
		set(ADMUX, MUX1);
		break;
		case 5:
		set(ADMUX, MUX0);		// set F7 as input, 0111
		break;
	}
	set(ADCSRA, ADEN);				//enable conversion
	set(ADCSRA, ADSC);				//begin conversion
}

ISR(ADC_vect){
	set(ADCSRA, ADIF);
	conversion_ADC();
	m_green(TOGGLE);
}

int main(void)
{
	init_ADC();
	m_usb_init();
    /* Replace with your application code */
    while (1) 
    {
		//m_wait(500);
		m_usb_tx_string("T1: ");
		m_usb_tx_int(ADC_V[0]);
		m_usb_tx_string("  ||  T2: ");
		m_usb_tx_int(ADC_V[1]);
		m_usb_tx_string("  ||  FR: ");
		m_usb_tx_int(ADC_V[2]);
		m_usb_tx_string("  ||  FL: ");
		m_usb_tx_int(ADC_V[3]);
		m_usb_tx_string("  ||  RR: ");
		m_usb_tx_int(ADC_V[4]);
		m_usb_tx_string("  ||  RL: ");
		m_usb_tx_int(ADC_V[5]);
		m_usb_tx_string("\n");
		m_red(TOGGLE);
    }
}

