/*
 * Search_Puck.c
 *
 * Created: 11/12/2016 5:02:44 PM
 * Author : mutian
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include "m_general.h"
#include "Search_Puck.h"
#include "m_rf.h"
#include "m_usb.h"
#include "m_bus.h"
#include "Localization.h"

#define CHANNEL 1
#define PACKET_LENGTH 10
#define RXADDRESS	0x1F	//31

#define GateR_x_attack -350
#define GateB_x_attack 370
#define GateR_y_attack -10
#define GateB_y_attack 10
#define GateR_x_defense -100
#define GateB_x_defense 100
#define GateR_y_defense -10
#define GateB_y_defense 10

void init(void);
void init_timer1(void);
void side_light(void);
void init_side_light(void);

int state = 0;
int scoreR;
int scoreB;
char flag_defense = 1;
unsigned char buffer[PACKET_LENGTH] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int goal[2] = {GateR_x_attack, GateR_y_attack};
int defense[2] = {GateB_x_defense, GateB_y_defense};
int gate_self[2] = {GateB_x_attack, GateB_y_attack};

int main(void)
{
	init();
    //clear(DDRF, 0);
    while (1) 
    {
//        set(PORTB, 2);		// only move forward
//        set(PORTB, 3);
//        OCR1B = OCR1A;
//        OCR1C = OCR1A;
        //go_for_goal(goal);
        //determine_catch();
		switch(state) {
			case 0:		// stop
				OCR1B = 0;
				OCR1C = 0;
				break;
                
			case 1:		// common test, flash the LED
                if (check(PINB, 0))		// B0 determines side
                {
                    set(PORTD, 7);
                    clear(PORTD, 6);
                    m_wait(200);
                    clear(PORTD, 7);
                    set(PORTD, 6);
                    m_wait(200);
                    set(PORTD, 7);
                    clear(PORTD, 6);
                    m_wait(200);
                    clear(PORTD, 7);    // red off
                    set(PORTD, 6);      // blue on
                }
                else
                {
                    clear(PORTD, 7);
                    set(PORTD, 6);
                    m_wait(200);
                    set(PORTD, 7);
                    clear(PORTD, 6);
                    m_wait(200);
                    clear(PORTD, 7);
                    set(PORTD, 6);
                    m_wait(200);
                    set(PORTD, 7);      // red on
                    clear(PORTD, 6);    // blue clear
                }
                state = 0;
                break;
                
			case 2:		// play, move
                m_green(ON);
                switch (conversion_ADC_stalker()) {
                    case 0:
                        flag_defense = go_to_goal(defense);
                        //go_for_goal(defense);
                        break;
                    case 1:
                        set(PORTC, 6);          // display go for the goal
                        go_for_goal(goal);		// catch the ball, go for the goal
                        flag_defense = 0;
                        clear(PORTC, 6);
                        break;
                    case 2:
                        set(PORTC, 7);          // display go for the puck
                        go_for_puck(goal);			// not catch the ball, go for the puck
                        flag_defense = 0;
                        clear(PORTC, 7);
                        break;
                    case 3:
                        set(PORTB, 4);
                        if (flag_defense) {
                            stalker();
                        } else {
                            flag_defense = go_to_goal(defense);
                        }
                        clear(PORTB, 4);
                        break;
                    default:
                        break;
                }
                m_green(OFF);
				break;
		}
	}
}

void init(){
	m_green(ON);
	m_clockdivide(0);		// 16MHz operation
    m_usb_init();
	m_bus_init();
    m_wait(2000);
    
	m_rf_open(CHANNEL,RXADDRESS,PACKET_LENGTH); // configure mRF

    init_ADC();
	init_timer1();

	set(DDRB, 3);			// B3 left
	set(DDRB, 2);			// B2 right
    
    set(DDRC, 6);
    set(DDRC, 7);
    set(DDRD, 3);
    set(DDRB, 4);

	sei();					// global interrupt
	init_side_light();		// initialize side and light

    m_wii_open();
    m_green(OFF);
}

void init_timer1(){
	OCR1A = 500;

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

ISR(INT2_vect){
    m_red(ON);
    m_rf_read(buffer,PACKET_LENGTH);	// pull the packet
    m_usb_tx_int(buffer[0]);
	switch(buffer[0]){
		case 0xA0:	//comm test
			usb_tx_string("Common Test: flash LED \n");
			state = 1;
			break;
            
		case 0xA1:	//play
			m_usb_tx_string("Play \n");
			state = 2;
			break;
            
		case 0xA2:	//goal R
			m_usb_tx_string("Goal R\n");
			state = 0;
			break;
            
		case 0xA3:	//goal B
			m_usb_tx_string("Goal B\n");
			state = 0;
			break;
            
		case 0xA4:	//pause
			m_usb_tx_string("Pause\n");
			state = 0;
			break;
            
		case 0xA6:	//halftime
			m_usb_tx_string("Halftime\n");
			state = 0;
			break;
            
		case 0xA7:	//game over
			m_usb_tx_string("Game over\n");
			state = 0;
			break;
            
		default:
			break;
	}
    m_red(OFF);
}

ISR(PCINT0_vect){
	side_light();
}

void init_side_light(){
	clear(DDRB, 0);
	clear(DDRB, 1);
	set(DDRB, 4);
	set(DDRB, 5);
	set(DDRD, 7);
	set(DDRD, 6);
	PORTB |=  ((1 << PORTB0) | (1 << PORTB1));// turn On the Pull-up, PB0 and PB1 are now inputs with pull-up enabled
	PCICR |= (1 << PCIE0);		// set PCIE0 to enable PCMSK0 scan
	PCMSK0 |= ((1 << PCINT0) | (1 << PCINT1));	// set PCINT0 to trigger an interrupt on state change
	side_light();
}

void side_light(){
//	if (check(PINB, 1))		// B1 determines goal and defense
//	{
//		
//		set(PORTB, 4);
//		clear(PORTB, 5);
//	}
//	else
//	{
//		
//		clear(PORTB, 4);
//		set(PORTB, 5);
//	}

	if (check(PINB, 0))		// B0 determines side
	{
        goal[0]= GateR_x_attack;
        goal[1]= GateR_y_attack;
        defense[0]= GateB_x_defense;
        defense[1]= GateB_y_defense;
        gate_self[0] = GateB_x_attack;
        gate_self[1] = GateB_y_attack;
        clear(PORTD, 7);    // red off
		set(PORTD, 6);      // blue on
	}
	else
	{
        goal[0]= GateB_x_attack;
        goal[1]= GateB_y_attack;
        defense[0]= GateR_x_defense;
        defense[1]= GateR_y_defense;
        gate_self[0] = GateR_x_attack;
        gate_self[1] = GateR_y_attack;
        set(PORTD, 7);      // red on
		clear(PORTD, 6);    // blue clear
	}
}





