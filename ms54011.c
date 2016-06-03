/*
 * ms54011.c
 *
 * Created: 03.06.2016 9:24:06
 * 
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>

unsigned int timer;
unsigned short countInt0 = 0;
unsigned short countInt1 = 0;

ISR(INT0_vect){
	countInt0++; //External Interrupt INT0 handler function
}

ISR(INT2_vect){
	countInt1++; //External Interrupt INT2 handler function
}

ISR (TIMER1_OVF_vect)
{
	//теперь прерывание будет происходить через 62439 тиков
	// таймера 1, что на частоте 16 МГц составит 1 сек.
	TCNT1 = 65536-62439;
	//Далее идет код, который будет работать каждую секунду.
	//Желательно, чтобы этот код был короче.
	if (timer)
	timer--;
}

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
	//   1<<CS12                  1048.576 mS (Fclk/256)
	TCCR1B = (1<<CS12);
	TCNT1 = 65536-62439;		//1s
	/* Enable timer 1 overflow interrupt. */
	TIMSK = (1<<TOIE1);
}

void setupPorts(void){
	DDRA = 0xFF;
	PORTA = 0xFF;	//Port A Initialization
	DDRB = 0xFF;
	PORTB = 0xFF;	//Port B Initialization
	DDRC = 0xFF;
	PORTC = 0xFF;	//Port C Initialization
	DDRD = 0xFF;    //(1<<PD4) | 1<<PD5) | (0<<PD
	PORTD = 0xFF;	//Port D Initialization 
}

void setDirect(void){
	if ( PORTD & (1<<PD5) )
		PORTD |= (1<<PD7); // Set true to  bit1 in port C
	else
		PORTD &= ~(1<<PD7); // Set false to  bit1 in port C
}


void start(void){
	setDirect();
	if ( PORTD & (1<<PD4) )
		PORTD |= (1<<PD6); // Set true to  bit1 in port C
	else
		PORTD &= ~(1<<PD6); // Set false to  bit1 in port C
}

void setGTS(void){
	if ( PORTB & (1<<PB7)){
		if ( (PORTB & (1<<PB1)) || (PORTB & (1<<PB4)) || (PORTB & (1<<PB5)) || (PORTB & (1<<PB6)) )
			PORTD |= (1<<PD3);
	}
}
void setDTO(void){
	if ( (countInt0 > 0) && (countInt1 > 0){
		PORTD |= (1<<PD2)
		countInt0 = 0;
		countInt1 = 0;
		//wait(100mc)?
	}
	else PORTD &= ~(1<<PD2);
}

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