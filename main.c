/*
 * irc.c
 *
 * Created: 14.10.2016 0:00:13
 * Author : MrQark
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>   // http://www.atmel.com/webdoc/AVRLibcReferenceManual/group__avr__string.html

#define LED PORTB5
// Schematic https://www.arduino.cc/en/uploads/Main/ArduinoNano30Schematic.pdf
// MCU datasheet	http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_datasheet.pdf
#define LEDP PORTB
#define LED_ON LEDP  |= (1<<LED)
#define LED_OFF LEDP &= ~(1<<LED)
#define LED_TGL LEDP ^= (1<<LED)
#define LED_INI DDRB |= (1<<LED); LED_OFF
volatile uint16_t TMR_TST = 0;
ISR(TIMER0_OVF_vect) {  // tick 1ms
		TCNT0 = 256-250;
	    
		if (TMR_TST++ > 500) {
			LED_TGL;
			TMR_TST =  0;
			PORTB^= (1<<PORTB1);
		}
}
int main(void)
{
	PORTB|= (1<<PORTB1);
	DDRB |= (1<<DDB1);
	LED_INI;

	TCCR0B = 3;  // 160000000/64 = 250 000 Hz 
	TCCR0A = 0;
	TIMSK0 = (1<<TOIE0);

	sei();


    /* Replace with your application code */
    while (1) 
    {

	
    }
}

