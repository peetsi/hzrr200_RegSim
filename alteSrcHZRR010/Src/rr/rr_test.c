// rr_test.c


#include "rr.h"
#include "rr_io.h"


void led_test_fullcolor( void ) {
  uint8_t led, color;

  for( color = 0; color<8; color++ ) {
    for( led = 0; led<5; led++ ) {
      set_led( leds[led], colors[color] );
      HAL_Delay(150);
    }
  }
}


void test_motor_output( void ) {
  int i;
  int delay = 500;

  // all 1
  for( i=0; i<8; i++ ) {
    set_pin( motors[i], 1 );
  }
  HAL_Delay( delay );

  // valley-test 
  for( i=0; i<8; i++ ) {
    set_pin( motors[i], 0 );
    HAL_Delay( delay );
    set_pin( motors[i], 1 );
  }
}


void rr_test( void ) {
  for( EVER ) {
    // led_test_fullcolor();
    test_motor_output();
    //rr_measure();
  }
}