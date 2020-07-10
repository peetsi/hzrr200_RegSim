<<<<<<< HEAD
// rr_ser.h

// Define to prevent recursive inclusion -------------------------------------
#ifndef __RR_SER_H
#define __RR_SER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "rr.h"

#define TX_BUF_SIZE        256 
#define RX_BUF_SIZE        256
#define RX_CMD_SIZE        256
#define TX_CMD_SIZE        RX_CMD_SIZE
#define RX_BUF_IT_SIZE     20             // >2; have some buffer left in case of problems ;)



// *** ser rx ringbuffer
typedef struct {
  uint16_t   size;             // size of ringbuffer in bytes
  volatile uint8_t    cnt;     // characters available in rb
  volatile uint8_t*   in;      // pointer to next free address for input byte
  volatile uint8_t*   out;     // pointer to next available character
                               // NOTE if in == out the ring buffer is empty
  volatile uint8_t*   buf;     // pointer to base of ring buffer in memory
} ringbuf_t;

extern ringbuf_t rx1b;

extern volatile uint8_t rx1Buf[];
extern volatile uint8_t txStr[TX_BUF_SIZE];
extern volatile uint8_t rxStr[RX_BUF_SIZE];
extern volatile uint8_t rx1BufIT[];




void usart1_irqHandler_callback( void );
void       ringbuf_in( ringbuf_t *rb, uint8_t b );
uint16_t   ringbuf_out( ringbuf_t *rb, uint8_t *b );
void       ser1_init( void );
void       ser1_send_str( char* s );
void       ser1_rx_check(void);
void       ser1_get_str( char *s );
void       command_interpreter( void );





#ifdef __cplusplus
}
#endif

#endif /* __RR_SER_H */
=======
// rr_ser.h

// Define to prevent recursive inclusion -------------------------------------
#ifndef __RR_SER_H
#define __RR_SER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "rr.h"

#define TX_BUF_SIZE        256 
#define RX_BUF_SIZE        256
#define RX_CMD_SIZE        256
#define TX_CMD_SIZE        RX_CMD_SIZE
#define RX_BUF_IT_SIZE     20             // >2; have some buffer left in case of problems ;)



// *** ser rx ringbuffer
typedef struct {
  uint16_t   size;             // size of ringbuffer in bytes
  volatile uint8_t    cnt;     // characters available in rb
  volatile uint8_t*   in;      // pointer to next free address for input byte
  volatile uint8_t*   out;     // pointer to next available character
                               // NOTE if in == out the ring buffer is empty
  volatile uint8_t*   buf;     // pointer to base of ring buffer in memory
} ringbuf_t;

extern ringbuf_t rx1b;

extern volatile uint8_t rx1Buf[];
extern volatile uint8_t txStr[TX_BUF_SIZE];
extern volatile uint8_t rxStr[RX_BUF_SIZE];
extern volatile uint8_t rx1BufIT[];




void usart1_irqHandler_callback( void );
void       ringbuf_in( ringbuf_t *rb, uint8_t b );
uint16_t   ringbuf_out( ringbuf_t *rb, uint8_t *b );
void       ser1_init( void );
void       ser1_send_str( char* s );
void       ser1_rx_check(void);
void       ser1_get_str( char *s );
void       command_interpreter( void );





#ifdef __cplusplus
}
#endif

#endif /* __RR_SER_H */
>>>>>>> fdb118d65f9ba2384457aa1ceb71ee94c0edf102
