<<<<<<< HEAD
// rr_valve.h

// Define to prevent recursive inclusion -------------------------------------
#ifndef __RR_VALVE_H
#define __RR_VALVE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "rr.h"


uint8_t motor_is_active( void );
void stop_activities( void );
uint8_t kennlinie( uint8_t valve );
uint8_t check_mode_summer( uint8_t v );
void install( void );
void move( void );
void control( void );
void led_blink( void );
void mot_pos( void );



#ifdef __cplusplus
}
#endif

#endif /* __RR_VALVE_H */
=======
// rr_valve.h

// Define to prevent recursive inclusion -------------------------------------
#ifndef __RR_VALVE_H
#define __RR_VALVE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "rr.h"


uint8_t motor_is_active( void );
void stop_activities( void );
uint8_t kennlinie( uint8_t valve );
uint8_t check_mode_summer( uint8_t v );
void install( void );
void move( void );
void control( void );
void led_blink( void );
void mot_pos( void );



#ifdef __cplusplus
}
#endif

#endif /* __RR_VALVE_H */
>>>>>>> fdb118d65f9ba2384457aa1ceb71ee94c0edf102
