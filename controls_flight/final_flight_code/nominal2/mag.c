/*
 * mag.c
 *
 * Created: 31-03-2012 15:15:08
 *  Author: Hussain
 */ 

/**
 * @file mag.c
 * @brief Read the magnetic field vector from the Magnetometer
 */
#include "common.h"
#include "mag.h"
#include "peripherals.h"
#include "uart.h"
uint8_t mag_count = 0;
uint8_t mag_data[7];

void init_UART_MM(void )
{

  UCSR1A = 0;
  UCSR1B = 0;
  UCSR1C = 0;
  
  ///Double Baud Rate
  UCSR1A |= _BV(U2X1);
  ///Enable Transmission and Reception
  UCSR1B |= _BV(TXEN1) | _BV(RXEN1) |  _BV(RXCIE1);
  ///8-bit Data Byte
  UCSR1C |= _BV(UCSZ11) | _BV(UCSZ10);
  ///Set Baud Rate to 9600
  UBRR1L = 103;
  UBRR1H = 0;
}

static volatile uint8_t t;
void poll_MM1(void)
{
	 char sx[20];
	 char sy[20];
	 char sz[20];
	 int16_t Bx1;
	 int16_t By1;
	 int16_t Bz1;
	 
	 uint16_t Bx;
	 uint16_t By;
	 uint16_t Bz;
	 uint8_t c= 80;
	 
	 transmit_UART0(c);
	 Bx=(uint16_t)receive_UART0();
	 Bx=(Bx<<8);
	 Bx &= 0xFF00;
	 Bx|=(uint16_t)receive_UART0();
	 Current_state.mm.B_x = ((float) Bx) / 15000;
	 // sprintf(sx,"x is %f",Current_state.mm.B_x);
	 
	 //transmit_UART0('x');
	 By=(uint16_t)receive_UART0();
	 By=(By<<8);
	 By &= 0xFF00;
	 By|=(uint16_t)receive_UART0();
	 Current_state.mm.B_y = ((float) By) / 15000;
	 //sprintf(sx,"x is %f",Current_state.mm.B_y);
	 
	 //transmit_UART0('y');
	 Bz=(uint16_t)receive_UART0();
	 Bz=(Bz<<8);
	 Bz &= 0xFF00;
	 Bz|=(uint16_t)receive_UART0();
	 Current_state.mm.B_z = ((float) Bz) / 15000;
	 //sprintf(sx,"x is %f",Current_state.mm.B_z);
	 
	 //transmit_UART0('cr');
	 //receive_MM();
	 
	 if(Bx > 30000)
	 {   Bx1 = Bx - 65536;
		 Current_state.mm.B_x = ((float) Bx1) / 15000;
		 //sprintf(sx,"x is %f",Current_state.mm.B_x);
	 }
	 
	 if(By > 30000)
	 {   By1 = By - 65536;
		 Current_state.mm.B_y = ((float) By1) / 15000;
		 // sprintf(sy,"y is %f",By1);
	 }
	 
	 if(Bz > 30000)
	 {Bz1 = Bz - 65536;Current_state.mm.B_z = ((float) Bz1) / 15000;
		 // sprintf(sz,"z is %f",Bz1);
	 }
	  /*transmit_string_UART0(sx);
      transmit_UART0(' ');
      transmit_string_UART0(sy);
      transmit_UART0(' ');
      transmit_string_UART0(sz);
      transmit_UART0('\r');
	  */
}
ISR(USART1_RX_vect)//ISR for Magmeter UART
{
	int16_t x, y, z;
	mag_data[mag_count] = UDR1;
	mag_count++;
	if (mag_count == 7)
	{
		mag_count = 0;
		x = ((int16_t)mag_data[0])<<8;
		x &= 0xFF00;
		x |= (int16_t)mag_data[1];
		
		
		y = ((int16_t)mag_data[2]) << 8;
		y &= 0xFF00;
		y |= (int16_t)mag_data[3];
		
		z = ((int16_t)mag_data[4]) << 8;
		z &= 0xFF00;
		z |= (int16_t)mag_data[5];
		
		///Convert the readings to Gauss
		Current_state.mm.B_x = ((float) x) / 15000;
		Current_state.mm.B_y = ((float) y) / 15000;
		Current_state.mm.B_z = ((float) z) / 15000;
	}
}
	
	

void poll_MM(void)
{
  ///Temporary variables for magnetometer readings
  
  uint8_t tmp;
  
  ///Send the poll command
 send_MM_cmd("*00P\r");
  
  
}

uint8_t receive_MM(void)
{
  ///Wait for a Byte to arrive
  while (!(UCSR1A & _BV(RXC1)));
  return UDR1; 
}

void send_MM_cmd(char *data)
{
  ///Send the char array till stop
  while((*data) != '\0')
  {
    ///* Wait for empty transmit buffer 
    while ( !(UCSR1A & (_BV(UDRE1))) );
    ///* Start transmission
    UDR1 = *data;
    data++;
  }
}

/*void configure_MM(void)
{
	/// <b>Initialization commands</b>

	///Baud Rate-> Set baud rate=9600 bps
	send_MM_cmd("*99WE\r", 1);
	send_MM_cmd("*99!BR=S\r", 2);
	///Device ID-> Set device ID=00
	send_MM_cmd("*99WE\r", 1);
	send_MM_cmd("*99ID=00\r", 1);
	///Format-> Turn Binary Mode on mode on
	send_MM_cmd("*00WE\r", 1);
	send_MM_cmd("*00B\r", 1);
	
	///Output-> Turn polling mode on
	send_MM_cmd("*00P\r", 1);
	
	///Turn off Zero reading
	send_MM_cmd("*00WE\r", 1);
	send_MM_cmd("*00ZF\r", 1);
	
	///S/R Mode-> Automatic S/R Pulses
	send_MM_cmd("*00WE\r", 1);
	send_MM_cmd("*00TN\r", 1);
	///Average Readings-> Turn average off
	send_MM_cmd("*00WE\r", 1);
	send_MM_cmd("*00VF\r", 1);
	///Store parameters: Paramters reloaded on restoring power.
	send_MM_cmd("*00WE\r", 1);
	send_MM_cmd("*00SP\r", 2);
}*/

