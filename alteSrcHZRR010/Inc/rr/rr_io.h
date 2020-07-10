<<<<<<< HEAD
// rr_io.h

// Define to prevent recursive inclusion -------------------------------------
#ifndef __RR_IO_H
#define __RR_IO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "adc.h"


// led and color names can be or-ed to set several outputs at the same time
#define LED1  0x01
#define LED2  0x02
#define LED3  0x04
#define LED4  0x08

#define BLACK   0x00   // LED off
#define RED     0x01
#define GREEN   0x02
#define BLUE    0x04
#define YELLOW  RED | GREEN
#define MAGENTA RED | BLUE
#define CYAN    GREEN | BLUE
#define WHITE   RED | GREEN | BLUE



uint8_t eeprom_read( void );
void eeprom_write( void );
void rr_init( void );
void rr_selftest( void );
void rr_measure( void );
void set_pin( uint8_t pin, uint8_t on );
void leds_off( void );
void set_led( uint8_t leds, uint8_t color );
void led_blink_err( uint8_t errNr );
uint8_t motor_start( void );
uint8_t check_motor_off( uint8_t v );
void ser_send_message( uint8_t mtype );
uint16_t get_address( void );
float controller_values( uint8_t v );
void motor_on( int8_t motorNr, int8_t dir );
void motor_stop_all( void );
void motor_openValves( void );
uint8_t check_motor_stop( uint8_t v );
uint8_t keypressed( void );
uint8_t keychange( void );
void param2default( void );


#ifdef __cplusplus
}
#endif

#endif /* __RR_IO_H */
=======
// rr_io.h

// Define to prevent recursive inclusion -------------------------------------
#ifndef __RR_IO_H
#define __RR_IO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "adc.h"


// led and color names can be or-ed to set several outputs at the same time
#define LED1  0x01
#define LED2  0x02
#define LED3  0x04
#define LED4  0x08

#define BLACK   0x00   // LED off
#define RED     0x01
#define GREEN   0x02
#define BLUE    0x04
#define YELLOW  RED | GREEN
#define MAGENTA RED | BLUE
#define CYAN    GREEN | BLUE
#define WHITE   RED | GREEN | BLUE



uint8_t eeprom_read( void );
void eeprom_write( void );
void rr_init( void );
void rr_selftest( void );
void rr_measure( void );
void set_pin( uint8_t pin, uint8_t on );
void leds_off( void );
void set_led( uint8_t leds, uint8_t color );
void led_blink_err( uint8_t errNr );
uint8_t motor_start( void );
uint8_t check_motor_off( uint8_t v );
void ser_send_message( uint8_t mtype );
uint16_t get_address( void );
float controller_values( uint8_t v );
void motor_on( int8_t motorNr, int8_t dir );
void motor_stop_all( void );
void motor_openValves( void );
uint8_t check_motor_stop( uint8_t v );
uint8_t keypressed( void );
uint8_t keychange( void );
void param2default( void );


#ifdef __cplusplus
}
#endif

#endif /* __RR_IO_H */
>>>>>>> fdb118d65f9ba2384457aa1ceb71ee94c0edf102
