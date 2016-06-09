/*------------------------------
--Board:--ms54011.c-------------
--Created:-03.06.2016-9:24:06---
--Controller:-ATMega162-16pu----
--Author:-Golovkin-P-G----------
--------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 8000000UL
#include <util/delay.h>

unsigned char countInt0 = 0;
unsigned char countInt1 = 0;

//--External-Interrupt-Procedure-------------------------------------------------------//
ISR(INT0_vect){
	countInt0++;
	PORTB ^= (1<<PB0);	
}
ISR(INT2_vect){
	countInt1++;
	PORTB ^= (1<<PB0);	
}
ISR(TIMER1_OVF_vect){
	//TCNT1H = 0;
	//TCNT1L = 0;
	TCNT1 = 57723;
	PORTB ^= (1<<PB0);
}
//--Main-Functions---------------------------------------------------------------------//
void setupInterrupt(void){
	cli();
//--MCUCR=0b00000011--The rising edge of INT0, INT2 generates an interrupt request-----//
	MCUCR = (1<<ISC00) | 1<<(ISC01);
	EMCUCR = (1<<ISC2);
//--GICR=0b11000000---General Interrupt Control Register - GICR Enable INT0, INT2------//
	GICR = (1<<INT2) | (1<<INT0);
//--SREG=(1<<7)-------Status Register Global Interrupt Enable--------------------------//
	sei();
}
void setupTimer1(void){
	cli();
	TCCR1B = (1<<CS12) | (1<<CS10);	//128 mkS (F_CPU/1028)
	TIMSK |= (1<<TOIE1);		//Enable timer 1 overflow interrupt
	TCNT1 = 57723;	
	sei();
}
void setupPorts(void){
	cli();
	SFIOR &= ~(1<<PUD); //Pull-up enabled
//---------Port A Initialization----------------------------------------------------//
	DDRA = 0x00;    //to read,
	PORTA = 0xFF;	//pull-up resistor
	
//---------Port B Initialization----------------------------------------------------//	
	DDRB = (1<<PB0); //PB0-TESTER
	PORTB = ~(1<<PB0);
	
//---------Port C Initialization----------------------------------------------------//	
	DDRC = (1<<PC2);
	PORTC = ~(1<<PC2);	//pull-up resistor
	
//---------Port D Initialization----------------------------------------------------//
//------------RX---------TX--------INT0------GTS_MK---------------------------------//
	DDRD = (0<<PD0) | (1<<PD1) | (0<<PD2) | (1<<PD3) |
//----------Start_In--Direct_In--Start_Out--Direct_Out------------------------------//
			(0<<PD4) | (0<<PD5) | (1<<PD6) | (1<<PD7);
			
	PORTD = (1<<PD4) | (1<<PD5); //pull-up resistor
	
//---------Port E Initialization----------------------------------------------------//
//--------DTO_to_BSP-----------------------------------------------------//
	DDRE = (1<<PE1);
//----------Pull-up----DTO_to_BSP---------------------------------------------------//
	PORTE = (1<<PE0) | (0<<PE1);
	sei();
}
//----------------------------------------------------------------------------------//
void setDirect(void){
	if ( PIND & (1<<PD5) )
		PORTD |= (1<<PD7); //Forward
	else
		PORTD &= ~(1<<PD7); //Backward
}
//----------------------------------------------------------------------------------//
void start(void){
	if ( PIND & (1<<PD4) ){
		setDirect();
		PORTD |= (1<<PD6); //Start
	}
	else
		PORTD &= ~(1<<PD6); //Stop
}
//---------------------------------------------------------------------------------//
void setGTS(void){
	if ( PINB & (1<<PB7) ){
//----------------G1-------------------G2-------------------G3-------------------G4----------//
		if ( (PINB & (1<<PB1)) || (PINB & (1<<PB4)) || (PINB & (1<<PB5)) || (PINB & (1<<PB6)) )
			PORTD |= (1<<PD3);
		else PORTD &= ~(1<<PD3);
	}
	else PORTD &= ~(1<<PD3);
}
//---------------------------------------------------------------------------------//
void setDTO(void){
	if ( (countInt0 > 0) && (countInt1 > 0)){
		PORTC |= (1<<PC2); //to-PC
		PORTE |= (1<<PE1); //to-BSP
		_delay_ms(10);
		PORTC &= ~(1<<PC2);
		PORTE &= ~(1<<PE1);
		//_delay_ms(1000);
		countInt0 = 0;
		countInt1 = 0;
	}
	else{
		PORTC &= ~(1<<PC2);
		PORTE &= ~(1<<PE1);
	}
}
//---------------------------------------------------------------------------------//
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