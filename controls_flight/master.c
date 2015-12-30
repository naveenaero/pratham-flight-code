/**
 * @file master.c
 * @brief Main file for master uC.
 * 
 * Contains the scheduler, preflight check routines, high level task blocks like power, control and communication.
 */

#include "common.h"
#include "spi.h"
#include "timer.h"
#include "peripherals.h"
#include "mag.h"
#include "gps.h"
#include "hm.h"
#include "slave_comm.h"
#include "comm.h"
#include "controller.h"

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
//@}

/**
 * @brief Communication with Power
 *
 * Obtains the health monitoring data by communcicating with Power microcontroller.
 */
void power(void){
  /// Start watchdog for power tasks
  watch_dog(T_POWER);
  
  ///Every 1.5 minutes get health monitoring data from the power uC
  if(Time % 90 == 0)
  {
    get_HM_data();
  }
}

/**
 * @brief Main function
 */

int main(void){
    
  /// Initialise Interfaces - UART of Magnetometer and GPS and the SPI bus
  init_UART_MM();
  init_UART_GPS();
  init_SPI();
  init_UART0();
  ///Configure the magnetometer
  //configure_MM();
  DDRA=0xf0;
  PORTA = 0xE0;
  
  ///Configure the Torquer
  //configure_torquer();
  
  ///Set Preflight pin as input
  //cbi(DDR_PF, PIN_PF);
  
  ///Check if Preflight Pin is high
  while(1){
    ///* Preflight Checks
    /*if(PORT_PF & _BV(PIN_PF)){
      ///* * Set the mode as preflight
      Mode = PREFLIGHT;
	  
	  ///* * Reset timer for 2 seconds
	  timer_reset_two_sec();

      ///* * GPS test
      read_GPS();
      while(UCSR0B & _BV(RXCIE0));
      send_preflight((char *)&Current_state.gps, sizeof(struct GPS_reading));

      ///* * Magnetometer and Torquer test

      ///* * Reading with no torquers on
      read_MM ();
      send_preflight((char *)&Current_state.mm, sizeof(struct MM_reading));
      
	  ///* * Reading with one torquer on at once
      Current_state.pwm.x_dir = 0;
      Current_state.pwm.x = 32768;
      Current_state.pwm.y_dir = 0;
      Current_state.pwm.y = 0;
      Current_state.pwm.z_dir = 0;
      Current_state.pwm.z = 0;
      set_PWM ();
      read_MM ();
      send_preflight((char *)&Current_state.mm, sizeof(struct MM_reading));
  
      Current_state.pwm.x_dir = 0;
      Current_state.pwm.x = 0;
      Current_state.pwm.y_dir = 0;
      Current_state.pwm.y = 32768;
      Current_state.pwm.z_dir = 0;
      Current_state.pwm.z = 0;
      set_PWM ();
      read_MM ();
      send_preflight((char *)&Current_state.mm, sizeof(struct MM_reading));
      
      Current_state.pwm.x_dir = 0;
      Current_state.pwm.x = 0;
      Current_state.pwm.y_dir = 0;
      Current_state.pwm.y = 0;
      Current_state.pwm.z_dir = 0;
      Current_state.pwm.z = 32768;  
      set_PWM ();
      read_MM ();
      send_preflight((char *)&Current_state.mm, sizeof(struct MM_reading));

	  ///* * Reading with one torquer on at once, in other direction
      Current_state.pwm.x_dir = 1;
      Current_state.pwm.x = 32768;
      Current_state.pwm.y_dir = 0;
      Current_state.pwm.y = 0;
      Current_state.pwm.z_dir = 0;
      Current_state.pwm.z = 0;
      set_PWM ();
      read_MM ();
      send_preflight((char *)&Current_state.mm, sizeof(struct MM_reading));
  
      Current_state.pwm.x_dir = 0;
      Current_state.pwm.x = 0;
      Current_state.pwm.y_dir = 1;
      Current_state.pwm.y = 32768;
      Current_state.pwm.z_dir = 0;
      Current_state.pwm.z = 0;
      set_PWM ();
      read_MM ();
      send_preflight((char *)&Current_state.mm, sizeof(struct MM_reading));
      
      Current_state.pwm.x_dir = 0;
      Current_state.pwm.x = 0;
      Current_state.pwm.y_dir = 0;
      Current_state.pwm.y = 0;
      Current_state.pwm.z_dir = 1;
      Current_state.pwm.z = 32768;  
      set_PWM ();
      read_MM ();
      send_preflight((char *)&Current_state.mm, sizeof(struct MM_reading));
	  
	  ///* * Set Torquer values to zero
      reset_PWM();


      ///* * Sunsensor test
      read_SS();
      send_preflight((char *)&Current_state.ss, sizeof(struct SS_reading));

      ///* Health Montoring
      get_HM_data();
      send_preflight((char *)&Current_state.hm, sizeof(struct HM_data));
	  
	  ///Communication Task
	  comm();
	  
	  ///* * Wait for 2 seconds to get over
      timer_wait_reset();
    }

  ///Normal Mode
  */
  
	  
    ///Wait for all components to switch on
    _delay_ms(300);
    
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
     while(1){         // while(!(PORT_PF & _BV(PIN_PF))){
  
      /**
      * * * * Task 1: Control codes
       * @ref control
       */
	  
      control();
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
      
      
    }
  }
  return 0;
}
