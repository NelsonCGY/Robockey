#include "Search_Puck.h"

#define CatchPuck 200
#define EffectiveIR 150
#define threshold_front 400
#define threshold_com1 750
#define threshold_com2 850
#define threshold_stalker 150
#define threshold_rev 0.4
//#define compensator_uturn 600

#define Kp 0.21    // Red 0.05 Bule 0.025
#define Kd 0.015
#define Kp_forward_rev 0.4
#define Kd_forward_rev 0.03
#define Kp_rev 0.1
#define Kd_rev 0.01
#define Kp_com1 0.4
#define Kd_com1 0.025
#define Kp_com2 3
#define Kd_com2 0.2
#define Kp_threshold_front 2
#define Kd_threshold_front 0.2
#define Kp_stalker1 0.1
#define Kd_stalker1 0.01
#define Kp_stalker2 0.6
#define Kd_stalker2 0.04

int ADC_V[6];
char flag_compen = 0;
int diff_12, diff_34, diff_12_pre = 0, diff_34_pre = 0;


int max(int * array){
	int a = 0;
    int i;
	for (i = 1; i < 5; i++)
	{
		if ( array[i] > a)
		{
			a = array[i];
		}
	}
	return a;
}

char conversion_ADC(){
    clear(ADCSRA,ADEN);
    clear(ADMUX, MUX2);                 // set F0 as input, 0000
    clear(ADMUX, MUX1);
    clear(ADMUX, MUX0);
    set(ADCSRA, ADEN);                  // enable conversion
    set(ADCSRA, ADSC);                  // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[0] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    clear(ADCSRA,ADEN);
    clear(ADMUX, MUX2);                 // set F1 as input, 0001
    clear(ADMUX, MUX1);
    set(ADMUX, MUX0);
    set(ADCSRA, ADEN);                  // enable conversion
    set(ADCSRA, ADSC);                  // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[5] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    if (ADC_V[0] + ADC_V[5] > 1000) {
        m_red(ON);
        return 1;
    }
    m_red(OFF);
    
    clear(ADCSRA,ADEN);
    clear(ADMUX, MUX0);             // set F4 as input, 0100
    clear(ADMUX, MUX1);
    set(ADMUX, MUX2);
    set(ADCSRA, ADEN);              // enable conversion
    set(ADCSRA, ADSC);              // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[1] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    clear(ADCSRA,ADEN);
    set(ADMUX, MUX0);               // set F5 as input, 0101
    set(ADCSRA, ADEN);              // enable conversion
    set(ADCSRA, ADSC);              // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[2] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    clear(ADCSRA,ADEN);
    clear(ADMUX, MUX0);             // set F6 as input, 0110
    set(ADMUX, MUX1);
    set(ADCSRA, ADEN);              // enable conversion
    set(ADCSRA, ADSC);              // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[3] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    clear(ADCSRA,ADEN);
    set(ADMUX, MUX0);               // set F7 as input, 0111
    set(ADCSRA, ADEN);              // enable conversion
    set(ADCSRA, ADSC);              // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[4] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    if (max(ADC_V) < EffectiveIR) {
        m_usb_tx_string("No effective IR\n");
        return 0;
    }
    
    m_usb_tx_uint(ADC_V[0]);
    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[5]);
    m_usb_tx_string("   ");
//    m_usb_tx_uint(ADC_V[0] + ADC_V[5]);
//    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[1]);
    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[2]);
    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[3]);
    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[4]);
    m_usb_tx_string("\n");
    
    return 2    ;
}

char conversion_ADC_stalker(){
    clear(ADCSRA,ADEN);
    clear(ADMUX, MUX2);                 // set F0 as input, 0000
    clear(ADMUX, MUX1);
    clear(ADMUX, MUX0);
    set(ADCSRA, ADEN);                  // enable conversion
    set(ADCSRA, ADSC);                  // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[0] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    clear(ADCSRA,ADEN);
    clear(ADMUX, MUX2);                 // set F1 as input, 0001
    clear(ADMUX, MUX1);
    set(ADMUX, MUX0);
    set(ADCSRA, ADEN);                  // enable conversion
    set(ADCSRA, ADSC);                  // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[5] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    if (ADC_V[0] + ADC_V[5] > 1000) {
        m_red(ON);
        return 1;
    }
    m_red(OFF);
    
    clear(ADCSRA,ADEN);
    clear(ADMUX, MUX0);             // set F4 as input, 0100
    clear(ADMUX, MUX1);
    set(ADMUX, MUX2);
    set(ADCSRA, ADEN);              // enable conversion
    set(ADCSRA, ADSC);              // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[1] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    clear(ADCSRA,ADEN);
    set(ADMUX, MUX0);               // set F5 as input, 0101
    set(ADCSRA, ADEN);              // enable conversion
    set(ADCSRA, ADSC);              // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[2] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    clear(ADCSRA,ADEN);
    clear(ADMUX, MUX0);             // set F6 as input, 0110
    set(ADMUX, MUX1);
    set(ADCSRA, ADEN);              // enable conversion
    set(ADCSRA, ADSC);              // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[3] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    clear(ADCSRA,ADEN);
    set(ADMUX, MUX0);               // set F7 as input, 0111
    set(ADCSRA, ADEN);              // enable conversion
    set(ADCSRA, ADSC);              // begin conversion
    while (!check(ADCSRA, ADIF)) {}
    ADC_V[4] = 1024 - ADC;
    clear(ADCSRA, ADIF);
    
    if (max(ADC_V) < EffectiveIR) {
        m_usb_tx_string("No effective IR\n");
        return 0;
    }
    
    m_usb_tx_uint(ADC_V[0]);
    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[5]);
    m_usb_tx_string("   ");
    //    m_usb_tx_uint(ADC_V[0] + ADC_V[5]);
    //    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[1]);
    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[2]);
    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[3]);
    m_usb_tx_string("   ");
    m_usb_tx_uint(ADC_V[4]);
    m_usb_tx_string("\n");
    
    if ((ADC_V[1] > threshold_stalker) || (ADC_V[2] > threshold_stalker) || (ADC_V[3] > threshold_stalker) || (ADC_V[4] > threshold_stalker)) {
        return 2;
    }
    return 3;
}

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
}

void stalker() {
    diff_12 = ADC_V[1] - ADC_V[2];
    diff_34 = ADC_V[3] - ADC_V[4];
    if ((ADC_V[1] + ADC_V[2]) > (ADC_V[3] + ADC_V[4])) {
        if (diff_12 > 0) {  // turn right
            OCR1C = threshold_rev * 0.8 * OCR1A + Kp_stalker1 * diff_12 - Kd_stalker1 * (diff_12 - diff_12_pre);
            OCR1B = OCR1C;
            set(PORTB, 2);
            clear(PORTB, 3);
        } else {            // turn left
            OCR1C = threshold_rev * 0.8 * OCR1A - Kp_stalker1 * diff_12 + Kd_stalker1 * (diff_12 - diff_12_pre);
            OCR1B = OCR1C;
            clear(PORTB, 2);
            set(PORTB, 3);
        }
    } else {
        if (diff_34 > 0) {                                  // turn right
            OCR1C = 1 * 0.8 * OCR1A - Kp_rev * diff_34 - Kd_rev * (diff_34 - diff_34_pre);
            OCR1B = OCR1C;
            set(PORTB, 2);              // left forward
            clear(PORTB, 3);            // right backward
            m_usb_tx_string("Left Forward:  ");
            m_usb_tx_int(OCR1C);
            m_usb_tx_string("   Right Backward:  ");
            m_usb_tx_int(OCR1B);
            m_usb_tx_string("\n");
        } else {                                            // turn left
            OCR1C = 1 * 0.8 * OCR1A + Kp_rev * diff_34  + Kd_rev * (diff_34 - diff_34_pre);
            OCR1B = OCR1C;
            clear(PORTB, 2);            // left backward
            set(PORTB, 3);              // right forward
            m_usb_tx_string("Left Backward:  ");
            m_usb_tx_int(OCR1C);
            m_usb_tx_string("   Right Forward:  ");
            m_usb_tx_int(OCR1B);
            m_usb_tx_string("\n");
        }
    }
    diff_12_pre = diff_12;
    diff_34_pre = diff_12;
}

void go_for_puck(int * goal) {
    static int adc1_pre = 0, adc2_pre = 0;
    diff_12 = ADC_V[1] - ADC_V[2];
    diff_34 = ADC_V[3] - ADC_V[4];
    if ((ADC_V[1] + ADC_V[2]) > (ADC_V[3] + ADC_V[4])) {    // forward
        calculate_pos(goal);
        switch (compensator_indicator()) {
            case 0:
                m_usb_tx_string("Case 0: \n");
                if (diff_12 > 0) {
                    if (ADC_V[2] < threshold_front) { // right U-turn
                        OCR1C = Kp_forward_rev * diff_12 - Kd_forward_rev * (diff_12 - diff_12_pre);
                        if (ADC_V[1] < threshold_front) {
                            OCR1C += Kp_threshold_front * (threshold_front - ADC_V[1]) + Kd_threshold_front * (ADC_V[1] - adc1_pre);
                        }
                        OCR1B = OCR1C;
                        set(PORTB, 2);
                        clear(PORTB, 3);
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Backward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                        }
                    else {                                        // turn right
                        OCR1C = 0.8 * OCR1A;
                        OCR1B = 0.8 * OCR1A - Kp * diff_12 + Kd * (diff_12 - diff_12_pre);
                        set(PORTB, 2);	// left
                        set(PORTB, 3);	// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    }
                }
                else {
                    if (ADC_V[1] < threshold_front) { // left U-turn
                        OCR1C = - Kp_forward_rev * diff_12 + Kd_forward_rev * (diff_12 - diff_12_pre);
                        if (ADC_V[2] < threshold_front) {
                            OCR1C += Kp_threshold_front * (threshold_front - ADC_V[2]) + Kp_threshold_front * (ADC_V[2] - adc2_pre);
                        }
                        OCR1B = OCR1C;
                        clear(PORTB, 2);
                        set(PORTB, 3);
                        m_usb_tx_string("Left Backward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                        } else {                                        // turn right
                            OCR1C = 0.8 * OCR1A + Kp * diff_12 - Kd * (diff_12 - diff_12_pre);
                            OCR1B = 0.8 * OCR1A;
                            set(PORTB, 2);	// left
                            set(PORTB, 3);	// right
                            m_usb_tx_string("Left Forward:  ");
                            m_usb_tx_int(OCR1C);
                            m_usb_tx_string("   Right Forward:  ");
                            m_usb_tx_int(OCR1B);
                            m_usb_tx_string("\n");
                        }
                }
                        break;
        
            case 1:
                m_usb_tx_string("Case 1: \n");
                if (diff_12 > 0) {
                    if (diff_12 < threshold_com1) {              // turn left for compensation
                        OCR1C = 0.8 * OCR1A - Kp_com1 * (threshold_com1 - diff_12) - Kd_com1 * (diff_12 - diff_12_pre);
                        OCR1B = 0.8 * OCR1A;
                        set(PORTB, 2);// left
                        set(PORTB, 3);  // right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    } else if (diff_12 > threshold_com2) {                                    // turn right for compensation
                        OCR1C = 0.8 * OCR1A;
                        OCR1B = 0.8 * OCR1A - Kp_com2 * (diff_12 - threshold_com2) + Kd_com2 * (diff_12 - diff_12_pre);
                        set(PORTB, 2);	// left
                        set(PORTB, 3);	// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    }
                    else {
                        OCR1C = 0.8 * OCR1A;
                        OCR1B = 0.8 * OCR1A;
                        set(PORTB, 2);	// left
                        set(PORTB, 3);	// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    }
                } else {
                    if (diff_12 > - threshold_com1) {            // turn right for compensation
                        OCR1C = 0.8 * OCR1A;
                        OCR1B = 0.8 * OCR1A - Kp_com1 * (threshold_com1 + diff_12) + Kd_com1 * (diff_12 - diff_12_pre);
                        set(PORTB, 2);  // left
                        set(PORTB, 3);// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    } else if (diff_12 < - threshold_com2) {                                    // turn left for compensation
                        OCR1C = 0.8 * OCR1A + Kp_com2 * (threshold_com2 + diff_12) - Kd_com2 * (diff_12 - diff_12_pre);
                        OCR1B = 0.8 * OCR1A;
                        set(PORTB, 2);	// left
                        set(PORTB, 3);	// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                        }
                        else {
                            OCR1C = 0.8 * OCR1A;
                            OCR1B = 0.8 * OCR1A;
                            set(PORTB, 2);	// left
                            set(PORTB, 3);	// right
                            m_usb_tx_string("Left Forward:  ");
                            m_usb_tx_int(OCR1C);
                            m_usb_tx_string("   Right Forward:  ");
                            m_usb_tx_int(OCR1B);
                            m_usb_tx_string("\n");
                        }
                    }
                break;
        
            case 2: // turn right
                m_usb_tx_string("Case 2: \n");
                if (diff_12 > 0) {
                    if (diff_12 < threshold_com1) {
                        OCR1C = 0.8 * OCR1A;
                        OCR1B = 0.8 * OCR1A + Kp_com1 * (threshold_com1 - diff_12) + Kd_com1 * (diff_12 - diff_12_pre);
                        set(PORTB, 2);  // left
                        set(PORTB, 3);// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    } else {
                        OCR1C = Kp_forward_rev * diff_12 - Kd_forward_rev * (diff_12 - diff_12_pre);
                        if (ADC_V[1] < threshold_front) {
                            OCR1C += Kp_threshold_front * (threshold_front - ADC_V[1]) + Kd_threshold_front * (ADC_V[1] - adc1_pre);
                        }
                        OCR1B = OCR1C;
                        set(PORTB, 2);
                        clear(PORTB, 3);
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Backward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    }
                } else {
                    if (diff_12 > - threshold_com1) {            // turn right for compensation
                        OCR1C = 0.8 * OCR1A;
                        OCR1B = 0.8 * OCR1A - Kp_com1 * (threshold_com1 + diff_12) + Kd_com1 * (diff_12 - diff_12_pre);
                        set(PORTB, 2);  // left
                        set(PORTB, 3);// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    } else if (diff_12 < - threshold_com2) {                                    // turn left for compensation
                        OCR1C = 0.8 * OCR1A + Kp_com2 * (threshold_com2 + diff_12) - Kd_com2 * (diff_12 - diff_12_pre);
                        OCR1B = 0.8 * OCR1A;
                        set(PORTB, 2);	// left
                        set(PORTB, 3);	// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    }
                    else {
                        OCR1C = 0.8 * OCR1A;
                        OCR1B = 0.8 * OCR1A;
                        set(PORTB, 2);	// left
                        set(PORTB, 3);	// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    }
                }
                break;
                
            case 3: // turn left
                m_usb_tx_string("Case 3: \n");
                if (diff_12 < 0) {
                    if (diff_12 > -threshold_com1) {
                        OCR1C = 0.8 * OCR1A + Kp_com1 * (threshold_com1 + diff_12) - Kd_com1 * (diff_12 - diff_12_pre);
                        OCR1B = 0.8 * OCR1A;
                        set(PORTB, 2);// left
                        set(PORTB, 3);  // right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    } else {
                        OCR1C = - Kp_forward_rev * diff_12 + Kd_forward_rev * (diff_12 - diff_12_pre);
                        if (ADC_V[2] < threshold_front) {
                            OCR1C += Kp_threshold_front * (threshold_front - ADC_V[2]) + Kp_threshold_front * (ADC_V[2] - adc2_pre);
                        }
                        OCR1B = OCR1C;
                        clear(PORTB, 2);
                        set(PORTB, 3);
                        m_usb_tx_string("Left Backward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    }
                } else {
                    if (diff_12 < threshold_com1) {              // turn left for compensation
                        OCR1C = 0.8 * OCR1A - Kp_com1 * (threshold_com1 - diff_12) - Kd_com1 * (diff_12 - diff_12_pre);
                        OCR1B = 0.8 * OCR1A;
                        set(PORTB, 2);// left
                        set(PORTB, 3);  // right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    } else if (diff_12 > threshold_com2) {                                    // turn right for compensation
                        OCR1C = 0.8 * OCR1A;
                        OCR1B = 0.8 * OCR1A - Kp_com2 * (diff_12 - threshold_com2) + Kd_com2 * (diff_12 - diff_12_pre);
                        set(PORTB, 2);	// left
                        set(PORTB, 3);	// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    }
                    else {
                        OCR1C = 0.8 * OCR1A;
                        OCR1B = 0.8 * OCR1A;
                        set(PORTB, 2);	// left
                        set(PORTB, 3);	// right
                        m_usb_tx_string("Left Forward:  ");
                        m_usb_tx_int(OCR1C);
                        m_usb_tx_string("   Right Forward:  ");
                        m_usb_tx_int(OCR1B);
                        m_usb_tx_string("\n");
                    }
                }
                break;
                
            default:
                break;
            }
    } else {
        if (diff_34 > 0) {                                  // turn right
            OCR1C = 1 * 0.8 * OCR1A - Kp_rev * diff_34 - Kd_rev * (diff_34 - diff_34_pre);
            OCR1B = OCR1C;
            set(PORTB, 2);              // left forward
            clear(PORTB, 3);            // right backward
            m_usb_tx_string("Left Forward:  ");
            m_usb_tx_int(OCR1C);
            m_usb_tx_string("   Right Backward:  ");
            m_usb_tx_int(OCR1B);
            m_usb_tx_string("\n");
        } else {                                            // turn left
            OCR1C = 1 * 0.8 * OCR1A + Kp_rev * diff_34  + Kd_rev * (diff_34 - diff_34_pre);
            OCR1B = OCR1C;
            clear(PORTB, 2);            // left backward
            set(PORTB, 3);              // right forward
            m_usb_tx_string("Left Backward:  ");
            m_usb_tx_int(OCR1C);
            m_usb_tx_string("   Right Forward:  ");
            m_usb_tx_int(OCR1B);
            m_usb_tx_string("\n");
        }
    }
    diff_12_pre = diff_12;
    diff_34_pre = diff_34;
    adc1_pre = ADC_V[1];
    adc2_pre = ADC_V[2];
}
