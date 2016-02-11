/**
 * @file master.c
 * @brief Main file for master uC.
 * 
 * Contains the scheduler, preflight check routines, high level task blocks like power, control and communication.
 */
#define F_CPU 8000000
#include "common.h"
#include "spi.h"
//#include "timer.h"
#include "peripherals.h"
#include "mag.h"
//#include "gps.h"
//#include "hm.h"
//#include "slave_comm.h"
#include "comm.h"
//#include "controller.h"
#include "util/twi.h"
#include "uart.h"
/// @todo Write Error checking codes to ensure the OBC does not go into infinite loops

/**
 * @defgroup global_state Global State parameters
 */
//@{
 
volatile uint8_t GPS_done = -1;
uint8_t Mode = DETUMBLING;
uint8_t Mode_prev = DETUMBLING;
uint64_t Time;
volatile struct state Current_state;
unsigned char write_data=0xCC;
unsigned int CyclesToCollectData = 1;
unsigned char recv_data;
uint8_t address=0x20, read=1, write=0;
unsigned int UniversalCycles = 1;
unsigned int counter1 = 0; //Beacon OverCurrent controller
unsigned int counter2 = 0; //Control OverCurrent controller
unsigned int counter3 = 0; //GPS OverCurrent controller
unsigned int counter4 = 0; //Downlink OverCurrent controller
char HM_Data[7]; //Array for HM Data



//@}

void TWI_init_master(void) // Function to initialize master for I2C
{
	//sei();

	TWSR = 0;
	TWCR = 0;
	TWBR = (F_CPU / 200000UL - 16) / 2; // Bit rate
	//TWSR=(0<<TWPS1)|(0<<TWPS0); // Setting prescalar bits
	// SCL freq= F_CPU/(16+2(TWBR).4^TWPS)

}

void TWI_start(void) //Function to send I2C start command
{
	// Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
	TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT))); // Wait till start condition is transmitted
	while((TWSR & 0xF8)!= 0x08); // Check for the acknowledgement
}

void TWI_repeated_start(void) // Function to send I2C repeated start command. Scarcely used
{
	// Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
	TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT))); // wait till restart condition is transmitted
	while((TWSR & 0xF8)!= 0x10); // Check for the acknoledgement
}

void TWI_write_address(unsigned char data)//Function for Master side to send slave address for I2C
{

	TWDR=data; // Address and write instruction
	TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
	while (!(TWCR & (1<<TWINT)))// Wait till complete TWDR byte transmitted
	while((TWSR & 0xF8)!= 0x18);  // Check for the acknowledgement

}

void TWI_read_address(unsigned char data) //Function for slave side to read address sent by Master
{
	TWDR=data; // Address and read instruction
	TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
	while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte received
	while((TWSR & 0xF8)!= 0x40);  // Check for the acknoledgement
}

void TWI_write_data(unsigned char data)//Function to write data on I2C data line
{
	TWDR=data; // put data in TWDR
	TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
	while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted
	while((TWSR & 0xF8) != 0x28); // Check for the acknoledgement
}

void TWI_read_data(void) //Function to read data from I2C data line
{
	TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
	while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted
	while((TWSR & 0xF8) != 0x58); // Check for the acknoledgement
	recv_data=TWDR;//PORTA=recv_data;
	if(UniversalCycles % CyclesToCollectData == 0){transmit_UART0(recv_data);}
	
}

void TWI_stop(void)//Function to stop data transmission
{
	// Clear TWI interrupt flag, Put stop condition on SDA, Enable TWI
	TWCR= (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	while(!(TWCR & (1<<TWSTO)));  // Wait till stop condition is transmitted
}



/**
 * @brief Communication with Power
 *
 * Obtains the health monitoring data by communcicating with Power microcontroller.
 */
/*void power(void){
  /// Start watchdog for power tasks
  watch_dog(T_POWER);
  
  ///Every 1.5 minutes get health monitoring data from the power uC
  if(Time % 90 == 0)
  {
    //get_HM_data();
  }
}*/

/**
 * @brief Main function
 */

int main(void){
    
  /// Initialise Interfaces - UART of Magnetometer and GPS and the SPI bus
   //0 - no over current; 1-reverse
  //Current_state.gps.gps_OC = 0;
  init_SPI();
  init_UART0();
  init_UART_MM();
  configure_torquer();
  DDRA=0xF0;
  Current_state.gps.gps_OC = 0; // This is important because the default value of OC should be 0
  DDRB |= (1<<PB0)|(1<<PB5);//PB0 and PB5 are slave select pins for Slave OBC and ADC
  int j=1;
  _delay_ms(2000);
  PORTA=0b11010000; //LED indicator for debugging
  TWI_init_master();
    ///Wait for all components to switch on
    _delay_ms(2000);
    
    ///* Switch on Global interrupts
    sei();
    
    ///* Set default mode of Satellite
    //Mode = DETUMBLING;
   // Mode = NOMINAL;
    ///* initialise Timer
    Time = 0;
	 
    ///* * Reset timer for 2 seconds
   // timer_reset_two_sec();
    ///Loop begins
     //while(1){         // while(!(PORT_PF & _BV(PIN_PF))){
  //transmit_UART0(42);
      /**
      * * * * Task 1: Control codes
       * @ref control
       */
	  
  while (1)
  {
  PORTB |= (1<<PB5); //Set slave select of ADC =1
	PORTB &= ~(1<<PB0);//Set slave select of Slave OBC = 0
	//SPCR |= (1<<SPE);
	//write_data&= ~(1<<4);// Turn off downlink
	//write_data&= ~(1<<1);//Turn off uplink
	if((Current_state.gps.gps_power_main==2)&&(Current_state.gps.gps_OC == 0))
	{write_data |= (1<<5);}
		else
		{write_data&= ~(1<<5);}//transmit_UART0('a');
PORTA = 0xAA;
TWI_start(); // Function to send start condition
PORTA=0b11000000;
TWI_write_address(address); // Function to write address and data direction bit(write) on SDA

PORTA=0b01100000;
TWI_write_data(write_data);     // Function to write data in slave
PORTA=0b10100000;
TWI_stop(); // Function to send stop condition
//transmit_UART0('b');

if (UniversalCycles%CyclesToCollectData == 0){transmit_string_UART0("PRA");}
_delay_ms(10); // Delay of 10 mili second
//************************Get HM Data from Power Board*************************************
for(int i=0;i<7;i=i+1) 
{
	TWI_start();
	
	TWI_read_address(address+read); // Function to write address and data direction bit(read) on SDA
	TWI_read_data(); // Function to read data from slave
	HM_Data[i] = recv_data;
	TWI_stop();
}
_delay_ms(10);


	for (uint8_t i =0;i<7;i++) {	
			SPDR = HM_Data[i];
			while(!(SPSR & (1<<SPIF) ));
			PORTA = 0xFF;
			_delay_ms(1);
		}
		PORTB |= (1<<PB0);
		PORTB &= ~(1<<PB5);
     	
     	if(counter1 == 0) //OC Check for Beacon
     	{
	     	if((HM_Data[6]&(0x80)) == 0)
	     	{
		     	counter1 = 1;
		     	write_data &= ~(1<<7);
		     	
	     	}
     	}
     	
     	if((counter1 > 0) && (counter1 < 7))
     	{
	     	counter1 = counter1+1;
     	}
     	
     	if(counter1 == 7)
     	{
	     	counter1 = 0;
	     	write_data |= (1<<7);
     	}
		 
		
		 
		 
		 
			 if((HM_Data[6]&(0x20)) == 0) //OC check for GPS
			 {
				Current_state.gps.gps_OC = 1;
			}
			else
			{
				Current_state.gps.gps_OC = 0;
			}


	
	control();	
	 if((HM_Data[6]&(0x40)) == 0) // OC Check for Torquer
	 {
		 reset_PWM();
		 
	 }		
	 
	 transmit_UART0(Current_state.gps.gps_OC);
     // PORTA=0xf0;
      /**
      * * * * Task 2: Communication with power uC through I2C. @ref power
      */
     // power();

      /**
      * * * * Task 3: Communication check routine;
      * @ref comm
      */
      //comm();
      
      ///* * Increment the Timer
      Time += FRAME_TIME;
      
      ///* * Wait for 2 seconds to get over
     // timer_wait_reset();
      
      _delay_ms(1000);
    //}
  }
  return 0;
}
