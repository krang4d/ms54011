﻿/*------------------------------
--Board:--ms54011.c-------------
--Created:-03.06.2296-9:24:06---
--Controller:-ATMega162-16pu----
--Author:-Golovkin-P-G---------- и стросс в части уарта
--------------------------------*/
#define F_CPU 2000000UL // 2 МГЦ БЛЕАТЬ!!!! Потому что фьюз деления на 8 стоит

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

unsigned char countInt0 = 0;
unsigned char countInt1 = 0;

unsigned char wasDTO1 = 0;
unsigned char wasDTO2 = 0;

unsigned char USART_Receive(void)
{
	
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}

unsigned char c = 'E';
//--External-Interrupt-Procedure-------------------------------------------------------//
ISR(INT0_vect){
	countInt0++;
	wasDTO2 = 0b00001000;
	PORTB ^= (1<<PB0);	
}
ISR(INT2_vect){
	countInt1++;
	wasDTO1 = 0b00000100;
	PORTB ^= (1<<PB0);	
}
ISR(TIMER1_OVF_vect){
	TCNT1 = 63582;
	
	PORTB ^= (1<<PB0);
}
ISR(USART0_RXC_vect)
{
	c = USART_Receive();
			switch (c)
			{
				case '0':
				PORTD &= ~(1<<PD7); //Stop
				PORTD &= ~(1<<PD6); //Release
				break;
				case '1':
				PORTD |= (1<<PD7);  //Start
				PORTD |= (1<<PD6); //Forward
				break;
				case '2':
				PORTD |= (1<<PD7);  //Start
				PORTD &= ~(1<<PD6); //Backward
				break;
				default:
				break;
			}
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
	TCNT1 = 63582;	
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
		PORTD |= (1<<PD6); //Forward
	else
		PORTD &= ~(1<<PD6); //Backward
}
//----------------------------------------------------------------------------------//
void start(void){
	if ( PIND & (1<<PD4) ) //Если НЕ пуск (управляем нулями! Т.к. подтянуто к 1)
	{
		PORTD &= ~(1<<PD7); //Stop
	}
	else
	{
		setDirect();
		PORTD |= (1<<PD7); //Start
	}
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
//govnocode start

void UART_Init(unsigned int ubrr)
{
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;

	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1<<URSEL0)|(0<<USBS0)|(3<<UCSZ00);
}

void UART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}


//govnocode end




//---------------------------------------------------------------------------------//
int main(void){
	//govnocode start
	UART_Init(MYUBRR);
	int divider = 0;

	unsigned char ByteToSend = 0;
	//govnocode end
	setupInterrupt();
	setupPorts();
	setupTimer1();
	

		
    while(1){
		//start();
		
		setGTS();
		setDTO();
		//govnocode start
		

		
		if (divider == 0)
		{
			ByteToSend =
			((PINB & (1 << PINB1)) >> 0)| //Получается такой байт:
			((PINB & (1 << PINB4)) >> 0)| //(П)(Г4)(Г3)(Г2)(ДТО2)(ДТО1)(Г1)(не задействован)
			((PINB & (1 << PINB5)) >> 0)|
			((PINB & (1 << PINB6)) >> 0)|
			((PINB & (1 << PINB7)) >> 0)|
			((PINA & (1 << PINA0)) << 2)|
			((PINA & (1 << PINA1)) << 2)|
			wasDTO1|wasDTO2;
			UART_Transmit(ByteToSend);
			wasDTO1 = 0;
			wasDTO2 = 0;
		}
		divider++;
		if (divider >= 30000)
		{
			divider = 0;
		}
		
		
		//govnocode end
		
		
    }
}