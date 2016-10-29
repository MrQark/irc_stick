/*
 * irc.c
 *
 * Created: 14.10.2016 0:00:13
 * Author : MrQark
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <string.h>   // http://www.atmel.com/webdoc/AVRLibcReferenceManual/group__avr__string.html
#include <uart.h>

/* define CPU frequency in Hz in Makefile */
//#ifndef F_CPU
//#error "F_CPU undefined, please define CPU frequency in Hz in Makefile"
//#endif

/* Define UART buad rate here */

#define UART_BAUD_RATE      115200
#ifndef F_CPU
#define F_CPU      16000000  // 16Mhz
#endif


// Schematic https://www.arduino.cc/en/uploads/Main/ArduinoNano30Schematic.pdf
// MCU datasheet	http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_datasheet.pdf
#define IRLED PORTB1
#define IRLEDP PORTB
#define IRLED_ON IRLEDP  &= ~(1<<IRLED)
#define IRLED_OFF IRLEDP |= (1<<IRLED)
#define IRLED_TGL IRLEDP ^= (1<<IRLED)
#define IRLED_INI DDRB |= (1<<IRLED); IRLED_OFF

#define C38KHZP PORTD
#define C38KHZ PORTD3
#define C38KHZD DDRD
#define C38KHZ_ON C38KHZP  |= (1<<C38KHZ)
#define C38KHZ_OFF C38KHZP &= ~(1<<C38KHZ)
#define C38KHZ_TGL C38KHZP ^= (1<<C38KHZ)
// OCR2 = (16 000 000 / 38 000)/2 - 1 = 209.526  == 209
// Fout = 16 000 000 / (2 * (209 +1)) = 38095 Hz (ideal freq is 38222 Hz; div = -0.25%)
#define C38KHZ_INI C38KHZD |= (1<<C38KHZ); C38KHZ_OFF; TCCR2B = 1; TCCR2A = ((1 << WGM21) | (1 << COM2B0)); OCR2A = 210; OCR2B = 210  // DIV (1*210)  - CTC mode - toggle OC2B 


#define LED PORTB5 
#define LEDP PORTB
#define LED_ON LEDP  |= (1<<LED)
#define LED_OFF LEDP &= ~(1<<LED)
#define LED_TGL LEDP ^= (1<<LED)
#define LED_INI DDRB |= (1<<LED); LED_OFF
#define TMR0_INT_EN TIMSK0 |= (1<<TOIE0)
#define TMR0_INT_DE TIMSK0 &= ~(1<<TOIE0)
#define TMR1_INT_EN TIMSK1 |= (1<<TOIE1)
#define TMR1_INT_DE TIMSK1 &= ~(1<<TOIE1)
#define ICP_FILTERING_EN TCCR1B |= (1<<ICNC1)  //ENABLE FILTERING AT ICP1
#define ICP_FILTERING_DE TCCR1B &= ~(1<<ICNC1) //DISABLE FILTERING AT ICP1
#define ICP_RISING_EDGE TCCR1B |= (1<<ICES1) //ENABLE RISING EDGE DETECTING AT ICP1
#define ICP_FALLING_EDGE TCCR1B &= ~(1<<ICES1) //ENABLE FALLING EDGE DETECTING AT ICP1
#define ICP_TOOGLE_EDGE  TCCR1B ^= (1<<ICES1) //TOOGLE EDGE DETECTING AT ICP1
#define ICP_TIMER_OCR1A_TOGGLE_CONFIG TCCR1A |= (1<<COM1A0); TCCR1A &= ~(1<<COM1A1); TCCR1A &= ~3; TCCR1B |= (1 << WGM12); TCCR1B &= ~(1 << WGM13); PORTB |= (1<<PORTB1); DDRB |= (1<<DDB1)
//#define ICP_TIMER_OCR1A_CLEAR_CONFIG  TCCR1A |= (1<<OCR1A1); TCCR1A &= ~(1<<OCR1A0) 
//#define ICP_TIMER_OCR1A_SET_CONFIG	  TCCR1A |= ((1<<OCR1A1) | (1<<OCR1A0))
#define ICP_TIMER_OCR1A_OFF_CONFIG	  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0)); TCCR1B &= ~(1 << WGM12) 
#define ICP_TIMER_CONFIG TCCR1B &= ~7; TCCR1B |= 2 // 0 - STOP / 1 - DIV 1 / 2 - DIV 8 / 3 - DIV 64 / 4 - DIV 256 / 5 - DIV 1024 / 6,7- T1 PIN SOURCE
#define ICP_RESET TIFR1 |= (1<<ICF1) 


#define RX_STRING_LENGHT 256


volatile uint8_t rx_timer = 0;
volatile uint16_t TMR_TST = 0;
uint8_t temp, rx_flag = 0, rx_data =0;
uint16_t rx_string_index =0, temp16;
uint8_t rx_state = 0, rx_string_ready = 0;

#define ICP_MAX_PULSES_NUM   200
#define ICP_SCAN_ERROR_OK 0
#define ICP_SCAN_ERROR_NMAX 1
#define ICP_SCAN_ERROR_NONE 2
volatile uint16_t IcpScanResult[2*ICP_MAX_PULSES_NUM];
volatile uint8_t IcpScanIndex = 0, IcpScanError, IcpScanState, IcpTxIndex;
volatile uint16_t IcpScanTimer = 0, IcpScanTimeout = 0;

uint8_t IRC_MODE = 0;
#define IRC_MODE_IDLE 0
#define IRC_MODE_SCAN 1
#define IRC_MODE_SEND 2

char rx_string[RX_STRING_LENGHT+1];
char *rx_ptr = rx_string;
char DevType[] = "IRC STICK V0.1";
char *DevTypeP = DevType;
#define RX_FLUSH memset (rx_string, 0, sizeof(rx_string))


 unsigned int c;
 char buffer[7];
 int  num=-1974;
/*
ISR(TIMER1_OVF_vect) {  // tick 0.5us
uint16_t tempo1;
	//if (IcpTxIndex & 1) IRLED_OFF; else IRLED_ON;
	IRLED_TGL;
	tempo1 = ~IcpScanResult[IcpTxIndex];
	tempo1++;
	TCNT1 = tempo1;
	if (IcpTxIndex++ >= 67) {IcpTxIndex = 200; TCNT1 = 0;}
	
}
*/
ISR(TIMER0_OVF_vect) {  // tick 1ms


		TCNT0 += 256-250;
	    if (rx_timer < 200) rx_timer++; // 1ms tick
		if (IcpScanTimer<60000) IcpScanTimer++; // 1ms tick 60sec limit
		if (TMR_TST++ > 500) {
			LED_TGL;
			TMR_TST =  0;
			//PORTB^= (1<<PORTB1);
		}
}
int main(void)
{

	LED_INI;
	IRLED_INI;//
	TCCR0B = 3;  // 160000000/64 = 250 000 Hz 
	TCCR0A = 0;
	//TCCR2B = 7;  // 160000000/1024 = 15 625 Hz  / 64 us  * 256 = 16.3ms
	//TCCR2A = 0;
	//ICP_TIMER_OCR1A_TOGGLE_CONFIG;
	TMR0_INT_EN;
    C38KHZ_INI;
    /*
     *  Initialize UART library, pass baudrate and AVR cpu clock
     *  with the macro 
     *  UART_BAUD_SELECT() (normal speed mode )
     *  or 
     *  UART_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)
     */
    uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 

  /*
     *  Transmit string to UART
     *  The string is buffered by the uart library in a circular buffer
     *  and one character at a time is transmitted to the UART using interrupts.
     *  uart_puts() blocks if it can not write the whole string to the circular 
     *  buffer
     */
    uart_puts("IRC was powered on.\0");
    
    /*
     * Transmit string from program memory to UART
     */
    //uart_puts_P("!!!\n");


	 /* 
     * Use standard avr-libc functions to convert numbers into string
     * before transmitting via UART
     */  
	//itoa(num, buffer, 10);   // convert interger into string (decimal format)
	//uart_puts(buffer);        // and transmit string to UART
    /*
     * Transmit single character to UART
     */
   // uart_putc('Q');
	//uart_putc('/r');
	//temp=0;
	//for (temp = 0; temp++; temp<255) rx_string[temp] = 0;  // flush rx buff 
	RX_FLUSH;
	OCR1A = 100;//IcpScanResult[0];
	sei();


    /* Replace with your application code */
    while (1) 
    {
        /*
         * Get received character from ringbuffer
         * uart_getc() returns in the lower byte the received character and 
         * in the higher byte (bitmask) the last receive error
         * UART_NO_DATA is returned when no data is available.
         *
         */
        c = uart_getc();
        if ( c & UART_NO_DATA )
        {
            /* 
             * no data available from UART 
             */
        }
        else
        {
            /*
             * new data available from UART
             * check for Frame or Overrun error
             */
            if ( c & UART_FRAME_ERROR )
            {
                /* Framing Error detected, i.e no stop bit detected */
                uart_puts_P("UART Frame Error: ");
            }
            if ( c & UART_OVERRUN_ERROR )
            {
                /* 
                 * Overrun, a character already present in the UART UDR register was 
                 * not read by the interrupt handler before the next character arrived,
                 * one or more received characters have been dropped
                 */
                uart_puts_P("UART Overrun Error: ");
            }
            if ( c & UART_BUFFER_OVERFLOW )
            {
                /* 
                 * We are not reading the receive buffer fast enough,
                 * one or more received character have been dropped 
                 */
                uart_puts_P("Buffer overflow error: ");
            }
            //received valid data
			rx_timer = 0;
			rx_flag = 1;
			rx_data =(unsigned char)c;
			
            //uart_putc( (unsigned char)c );
        }
		// s1
		//itoa(rx_state, buffer, 10);   // convert interger into string (decimal format)
		//uart_puts(buffer);
		switch (rx_state) {
			case 0:   // search the pause and switch listen
			if (rx_timer >20) rx_state = 1;
			break;
			case 1:   // fill the buffer and check timeout
			if (rx_flag) {
				rx_flag = 0;
				//uart_putc( rx_data );
				rx_string_index++;
				if (rx_string_index>=RX_STRING_LENGHT-1) rx_string_index = RX_STRING_LENGHT-1;
				if (rx_string_index <= RX_STRING_LENGHT-1) {
					rx_string[rx_string_index-1] = rx_data;
					if (rx_string[rx_string_index-1] == 13) rx_string[rx_string_index-1] = 0;
					rx_string[rx_string_index] = 0;
				}
				else {
					uart_puts_P("The string received is too long");
				}
			}
			if ((rx_string_index) && (rx_timer >20)) rx_state = 2;
			break;
			case 2:   // processing and switch next round
			//uart_puts(rx_ptr);
			rx_string_index=0;
			rx_string_ready = 1;
			rx_state = 0;
			break;   

		}
		//s1

		//s2
		if (rx_string_ready) {
		rx_string_ready = 0;
		
		strcpy(rx_ptr,strlwr(rx_ptr));
		//uart_puts(rx_ptr);

		//uart_puts("type");
		//itoa(rx_string[4], buffer, 10);   // convert interger into string (decimal format)
		//uart_puts(buffer);
		IRC_MODE = 0;
			if (!(strcmp(rx_ptr,"type")))  if (!IRC_MODE) uart_puts(DevTypeP);
			if (!(strcmp(rx_ptr,"echo")))  if (!IRC_MODE) uart_puts_P("echo\n");
			if (!(strcmp(rx_ptr,"scan")))  {if (!IRC_MODE) uart_puts_P("Press single key for capture (you have 10s)\0"); if (!IRC_MODE) IRC_MODE = IRC_MODE_SCAN;}
			if (!(strcmp(rx_ptr,"send")))  if (!IRC_MODE) IRC_MODE = IRC_MODE_SEND;
		}
		//s2	
		
		//s3
		if (IRC_MODE == IRC_MODE_SCAN) {
		//cli();
			ICP_FILTERING_EN;  //ENABLE FILTERING AT ICP1
			ICP_TIMER_CONFIG; // 0 - STOP / 1 - DIV 1 / 2 - DIV 8 / 3 - DIV 64 / 4 - DIV 256 / 5 - DIV 1024 / 6,7- T1 PIN SOURCE
			ICP_FALLING_EDGE;	
			IcpScanIndex = 0;
			IcpScanState = 0;
			IcpScanError = ICP_SCAN_ERROR_OK;
			IcpScanTimeout = 10000;
			IcpScanTimer = 0;
			while (IcpScanTimer < IcpScanTimeout) {
				switch (IcpScanState) {
					case 0: // wait for edge
					if (TIFR1 & (1<<ICF1)) IcpScanState = 1;
					break;
					case 1: 
					ICP_RESET;
					IcpScanTimer=0;
					IcpScanTimeout = 100;
					IcpScanResult[IcpScanIndex] = ICR1;
					IcpScanState = 0;
					//ICP_TOOGLE_EDGE;
					if ((IcpScanIndex & 1)) ICP_FALLING_EDGE; else ICP_RISING_EDGE;
					IcpScanIndex++;
					if (IcpScanIndex >= ICP_MAX_PULSES_NUM) {IcpScanError = ICP_SCAN_ERROR_NMAX; IcpScanTimeout = 0;IcpScanState = 2;}
					break;
					default:
					break;
				} 
			}
				if (IcpScanIndex == 0) IcpScanError = ICP_SCAN_ERROR_NONE; else IcpScanIndex--;
			
			
		IRC_MODE = IRC_MODE_IDLE;
		for (uint8_t i=1; i<=IcpScanIndex; i++) {
			if (IcpScanResult[i]<IcpScanResult[i-1]) {
				IcpScanResult[i-1]=!(IcpScanResult[i-1]);
				IcpScanResult[i-1]++;
				IcpScanResult[i-1]+=IcpScanResult[i];
			} 	else IcpScanResult[i-1]=IcpScanResult[i]-IcpScanResult[i-1];
		}
		IcpScanResult[IcpScanIndex]=0;
		//sei();  
		if (IcpScanError == ICP_SCAN_ERROR_NMAX) uart_puts_P ("Error detected: too many pulses");
		if (IcpScanError == ICP_SCAN_ERROR_NONE) uart_puts_P ("Scan state timeout");
		if (IcpScanError == ICP_SCAN_ERROR_OK) {
			for (uint8_t i=0;i<=IcpScanIndex; i++) {
			itoa(IcpScanResult[i], buffer, 10);   // convert integer into string (decimal format)
			uart_puts(buffer);  
			uart_puts_P ("; ");
			}  
		itoa(IcpScanIndex, buffer, 10);   // convert integer into string (decimal format)
		uart_puts(buffer);
		}
	}
		//s3
		//s4
		if (IRC_MODE == IRC_MODE_SEND) {
		//itoa(IcpScanIndex, buffer, 10);   // convert integer into string (decimal format)
		//uart_puts(buffer);
		cli();
		ICP_TIMER_CONFIG;
		TCNT1 = 0;
		//IcpTxIndex = 0;
		//TMR0_INT_DE;
		//TMR1_INT_EN;
	
		//sei();
		//while (IcpTxIndex <200);
		//cli();
		//TMR0_INT_EN;
		//TMR1_INT_DE;
		//uart_puts_P("Done");
		//sei();
		//IRC_MODE = IRC_MODE_IDLE;
		temp16 = IcpScanResult[0];
		OCR1A = temp16;
		ICP_TIMER_OCR1A_TOGGLE_CONFIG;
		TIFR1 |= (1 << OCF1A);
		while(TIFR1 & (1 << OCF1A));
		TIFR1 |= (1 << OCF1A);
			for (uint8_t i=1; i<IcpScanIndex; i++) {
			
			//itoa(i, buffer, 10);   // convert integer into string (decimal format)
			//uart_puts(buffer);
			temp16 = IcpScanResult[i];
			OCR1A = temp16;
			TIFR1 |= (1 << OCF1A);
			while(TIFR1 & (1 << OCF1A));
			TIFR1 |= (1 << OCF1A);
			}
		ICP_TIMER_OCR1A_OFF_CONFIG;
		IRC_MODE = IRC_MODE_IDLE;
		sei();
		uart_puts_P("Done");
		//for (uint8_t i=0; i<IcpScanIndex; i++) {
		//	itoa(IcpScanResult[i], buffer, 10);   // convert integer into string (decimal format)
		//	uart_puts(buffer);
		//}
		}
		//s4
    }

}

