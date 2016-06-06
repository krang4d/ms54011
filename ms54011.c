/******************************
 * ms54011.c
 * Created: 03.06.2016 9:24:06
 * ATMega162-16pu
 ******************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000UL;

unsigned int timer;
unsigned short countInt0 = 0;
unsigned short countInt1 = 0;

//-----INTERROPT----//
ISR(INT0_vect){
	countInt0++;	//External Interrupt INT0 handler function
	_delay_ms(1);
}
ISR(INT2_vect){
	countInt1++;	//External Interrupt INT2 handler function
	_delay_ms(1);
}
ISR (TIMER1_OVF_vect){
	;
}
//------------------//
void setupInterrupt(void){
	GICR = (1<<INT2) | (1<<INT0);	/*GICR=0b11000000;	General Interrupt Control Register - GICR Enable INT1, INT0*/
	MCUCR = (1<<ISC11) | 1<<(ISC01);/*MCUCR=0b00001010;	The falling edge of INT1, INT2 generates an interrupt request*/
	sei();							/*SREG=(1<<7); Status Register Global Interrupt Enable*/
}
void setupTimer1(void){
	//With 16 MHz clock and 65536 times counting T/C1 overflow interrupt
	// will occur every:
	//   1<<CS10                  4096 mkS  (no prescale Fclk/1)
	//   1<<CS11                  32.768 mS (Fclk/8)
	//  (1<<CS11)|(1<<CS10)       262.144 mS (Fclk/64)
	
	TCCR1B = (1<<CS12); //1048.576 mS (Fclk/256)
	TCNT1 = 65536-62439;		//1s
	/* Enable timer 1 overflow interrupt. */
	TIMSK = (1<<TOIE1);
}

void setupPorts(void){
	SFIOR |= ~(1<<PUD); // Pull-up enabled
	
	DDRA = 0x00;	//Port A Initialization to read,
	PORTA = 0xFF;	//pull-up resistor ON.
	
	DDRB = 0x00;	//Port B Initialization to read,
	PORTB = 0xFF;	//pull-up resistor ON.
	
	DDRC = 0xFF;	//Port C Initialization to read,
	PORTC = 0xFF;	//pull-up resistor ON.
	
	//DDRC = (1<<INT1) | (1<<PD4) | (1<<PD5) | (0<<PD);    //Port D Initialization  
	DDRD = 0xFF;
	PORTD = 0xFF;
	
	DDRE = 0xFF;
	PORTE = 0xff;
}
//-------------------------------------------------//
void setDirect(void){
	if ( PORTD & (1<<PD5) )
		PORTD |= (1<<PD7); // Set true to  bit1 in port C
	else
		PORTD &= ~(1<<PD7); // Set false to  bit1 in port C
}
//--------------------------------------------------//
void start(void){
	setDirect();
	if ( PORTD & (1<<PD4) )
		PORTD |= (1<<PD6); // Set true to  bit1 in port C
	else
		PORTD &= ~(1<<PD6); // Set false to  bit1 in port C
}
//-------------------------------------------------//
void setGTS(void){
	if ( PORTB & (1<<PB7)){
		if ( (PORTB & (1<<PB1)) || (PORTB & (1<<PB4)) || (PORTB & (1<<PB5)) || (PORTB & (1<<PB6)) )
			PORTD |= (1<<PD3);
	}
}
//-------------------------------------------------//
void setDTO(void){
	if ( (countInt0 > 0) && (countInt1 > 0)){
//-------------DTO-to-PC----------------//
		PORTC |= (1<<PC2); 
		_delay_ms(10);
		PORTC &= ~(1<<PC2);
//-------------DTO-to-BSP----------------//
		PORTE |= (1<<PE1);
		_delay_ms(10);
		PORTE &= ~(1<<PE1);
//--------------------------------------//
		_delay_ms(1000);
		countInt0 = 0;
		countInt1 = 0;
	}
	else (PORTD &= ~(1<<PD2));
}
//---------------------------------------------------//
int main(void){
	setupInterrupt();
	setupPorts();
	setupTimer1();
	
    while(1){
		start();	
		setGTS();	
		setDTO();	
    }
}