// HZ-RR010
// Heizungs Ruecklauf Regler, Version 01.0
//
// Peter Loster (c); 2015, 2016
//
// serial communication task
//

#include "rr.h"
#include "rr_io.h"
#include "rr_ser.h"
#include "rr_ctrl.h"
#include "rr_autogene.h"




// **************************************
// serial communication
// **************************************

// NOTE serial receiving of chars to ring-buffer
// problem:  the ST HAL-library supports only reception of a fixed number of chars.
// solution: add some user-code to function "USART1_IRQHandler()" in file "stm32l0xx_it.c"
//           which gets each received byte and stores it to a ring-buffer.
//           pointers and counters are reset so IT-reception should be active permanently
// how it works:
// - start reception using "HAL_UART_Receive_IT()" to a dummy buffer with some bytes 
// - each rx char is put in the dummy buffer and 
// - is also moved to the ring-buffer 
//   in user-extended IT-function (see below) immediately after reception.
// - the count of received characters is reduced by one
// - the byte pointer to the dummy buffer is set back to the last place
// --> thus "HAL_UART_Receive_IT()" never ends except an error condition is met
//     in this case the function "void HAL_UART_RxCpltCallback()" resets a serial-active flag
//     back, which in turn starts function "HAL_UART_Receive_IT()" from the main loop.
//
// ATTENTION
// insert the following code in file "stm32l0xx_it.c"; find /* */ comments and add:
//
//  /* USER CODE BEGIN 0 */
//  #include "rr.h"
//  /* USER CODE END 0 */
// 
//  /* USER CODE BEGIN USART1_IRQn 1 */
//  usart1_irqHandler_callback();
//  /* USER CODE END USART1_IRQn 1 */
//
// ATTENTION uart1 receive always stays in receive mode; count never reaches an end.
//           receive buffer is kept as a ring buffer




volatile uint8_t txStr[TX_BUF_SIZE];
volatile uint8_t rxStr[RX_BUF_SIZE];
volatile uint8_t rx1BufIT[ RX_BUF_IT_SIZE ] = {0xFE,0xFE,0xFE,0xFE,0xFE,0xFE};


void usart1_irqHandler_callback( void ) {
  // will be called after each character received
  // if this interrupt cannot receive a character in time (overflow) RxXferCount
  // is incremented and pRxBuffPtr is incremented. We will still receive the
  // last character from this point; the others are lost (but could be regained).
  // BUT: some time the rx-it-buffer flows over and then UART1 is started again
  if( huart1.RxXferCount < RX_BUF_IT_SIZE )
  {
    huart1.pRxBuffPtr--;                      // point back to just received character
    uint8_t c = (uint8_t) *huart1.pRxBuffPtr; // newly received character
    ringbuf_in( &rx1b, c );                   // put c in rx ring-buffer
    if( c==0x0A ) {                           // <lf> is last character of a modbus message
      stat.rx1Eol = 1;                        // indicate end of line received
      command_interpreter();
      }
    huart1.RxXferCount++;                     // increment xfer-counter to avoid end of rx IT
  }
  else {
    stat.rx1Running = 0;                      // invoke a restart of UART1 from main loop
  }
}


void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart ) {
  // after starting UART1 receive with 20 bytes we should never get here.
  // except for an overflow when  usart1_irqHandler_callback() was not
  // active in time
  // if so, reset flag to let main loop initialize rx1 reception again.
  stat.rx1Running = 0;
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  // check for parity error and other errors if needed
  stat.errSer1 += 1; 
  huart1.ErrorCode = 0;
}


void ser1_rx_check(void) {
  HAL_StatusTypeDef st;
  // keep serial receive always running
  if( (stat.rx1Running == 0) 
   || ((huart1.RxState & HAL_UART_STATE_BUSY_RX) != HAL_UART_STATE_BUSY_RX)) {
    st = HAL_UART_Receive_IT( &huart1, (uint8_t*)rx1BufIT, RX_BUF_IT_SIZE );
    if( !((st == HAL_OK)||(st == HAL_BUSY)) ) {  // NOTE plpl HAL_BUSY eingefuegt
      stat.rx1Running = 0;
    }
    else stat.rx1Running = 1;
  }
}





// *** ser rx ringbuffer
volatile uint8_t rx1Buf[RX_BUF_SIZE];
ringbuf_t rx1b;

void ringbuf_init( ringbuf_t* rb, volatile uint8_t* buf ) {
  rb->size = RX_BUF_SIZE;
  rb->cnt  = 0;
  rb->in   = buf;     // pointer to next free input address
  rb->out  = buf;     // pointer to next character available
  rb->buf  = buf;     // pointer to beginning of buffer byte array
}

void ringbuf_in( ringbuf_t *rb, uint8_t b ) {
  // put byte b in ring-buffer
  *rb->in    = b;
  rb->in    += 1;
  if( (rb->in - rb->buf) > rb->size - 1 ) rb->in = rb->buf;  // set to begin of rb 
  rb->cnt++;
}

uint16_t ringbuf_out( ringbuf_t *rb, uint8_t *b ) {
  // read byte from ring-buffer into *b
  // return nr of bytes still available in ring-buffer, 0 if none
  uint16_t  z;
  *b = *rb->out;
  rb->out   += 1;
  if( (rb->out - rb->buf) > rb->size - 1 ) rb->out = rb->buf; // set to begin of rb
  z = (uint16_t)( rb->out - rb->in );
  rb->cnt--;
  return z;
}


void ser1_get_str( char *s ) {
  char *sp = s;
  char c=0;
  uint16_t cnt = RX_CMD_SIZE - 1;
  uint16_t bn = cnt;                      // count of remaining chars in ring-buffer

  while( c!=0x0A && bn && cnt-- ) {       // <lf> to terminate message
    bn = ringbuf_out( &rx1b, (uint8_t*)&c );
    *sp++ = c;
    if( c== 0x0D || c==0x0A ) c = 0;     // terminate string
  }
  *(sp-2) = 0;                           // terminate string with 0 remove <cr><lf>
  return;
}





void ser1_init( void ) {
  ringbuf_init( &rx1b, rx1Buf );
}



// *** send string

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart1 ) {
  stat.tx1Busy = 0;
  set_pin( de485, 0 );

}


void ser1_send( void ) {
  // send string in global string txStr;
  HAL_StatusTypeDef hs;
  uint16_t len = strlen( (char*)txStr );
   // enable rs485 tx; will be disabled on tx complete interrupt callback
  set_pin( de485, 1 );
  hs = HAL_UART_Transmit_DMA( &huart1, (uint8_t*)txStr, len );
}


void ser1_send_str( char *s ) {
  while( stat.tx1Busy );   // wait for tx-buffer becoming free
  stat.tx1Busy = 1;
  strcpy( (char*)txStr, (char*)s );
  ser1_send();
}





// *** receive string using interrupt


// *** receive ring buffer

// **********************************************
// modbus protocol
// **********************************************


// unwraps modbus and checks checksum within data 
uint32_t unwrap_modbus( char *sa, char *s ) {
  // s        string pointer with received string
  // sa       stripped command nr and data from modbus message without ':', checksum and lrc-parity
  // return   0 ok; error: 1 too short; 2 ':' missing; 3 lrc parity wrong; 4 cs wrong;
  uint8_t  i, l, py, spy;
  uint16_t cs, scs;
  uint32_t err = 0L;
  char hs[5];

  l = strlen( s );
  if( l < 11 || l > RX_CMD_SIZE-2 ) {
    err = 1;
  }
  else if( s[0] != ':' ) {
    err = 2;
  }
  else {
    // lrc parity calculation without leading ':'
    py = 0;
    for( i=1; i < l-2; i++ ) {
      py ^= s[i];
    }  
    spy = s[l-1];
    strncpy( hs, &s[l-2], 2 ); hs[2]=0;
    spy = (uint8_t)strtol(hs,NULL,16);
    if( py != spy ) {
      err = 3;
    }
    else {
      cs = 0;
      for( i=1; i < l-6; i++ ) {
        cs += s[i];
      }
      strncpy( hs, &s[l-6], 4 ); hs[4]=0;
      scs = (uint16_t)strtol(hs,NULL,16);       
      if( cs == scs ) {
        strncpy( sa, &s[1], l-7 ); sa[l-7] = 0;
      }
      else {
        err = 4;
        sa[0] = 0;
      }
    }
  }
  return err;
}


void wrap_modbus( char *sout, char *sin ) {
  // sin      string input 
  // sout     string output
  char        sh[10];
  uint8_t     i, l, py,c;
  uint16_t    cs;
  float       ftic = 0.001 * (float)stat.tic;

  // "00ccaarV1.1 t122117.3"; 
  // 00 = master address, 
  // cc = command, 
  // aa = module address, 
  // r  =
  // V  = <protocol version>, 
  // t  = timestamp seconds
  snprintf( sout, TX_CMD_SIZE-9, ":00%02X%02X%1X%s t%.1f %s ", com.nr, stat.adr, com.con,\
                                   PROT_REV, ftic, sin );
  l = strlen( sout );
  cs = 0;
  for( i=1; i < l; i++ ) {
    cs += sout[i];
  }
  snprintf( sh,10,"%04X", cs );
  strcat( sout, sh );
  l += 4;
  py = 0;
  for( i=1; i < l; i++ ) {
    c=sout[i];
    py ^= c;
  }
  snprintf( sh, 10, "%02X\r\n", py );
  strcat( sout, sh );
}




// **********************************************
// command handling
// **********************************************


void command_interpreter( void ) {
  char     cmd[TX_CMD_SIZE];
  char     s0[RX_CMD_SIZE];
  char     s1[RX_CMD_SIZE];
  char     ans[TX_CMD_SIZE];
  char     hs[10];
  uint8_t  valve = stat.actValve;
  uint8_t  v = valve-1;              // valve index 0..3
  char     jz;                       // jahreszeit e {'S', 'W'}
  char     *cp, *cp0;
  uint32_t err0 = 0;
  float    f;

  stat.rx1Eol = 0;

  ser1_get_str( cmd );
  err0 = unwrap_modbus( s0, cmd );
  if( err0 ) {
    return;                  // discard; command not readable
  }

  strncpy( hs, &s0[0], 2 ); hs[2] = 0;
  com.adr   = strtol( hs, NULL, 16 );
  if( com.adr != stat.adr ) {
    return;                  // discard; command is for some other module
  }

  strncpy( hs, &s0[2], 2 ); hs[2] = 0;
  com.nr    = strtol( hs, NULL, 16 );
  com.con   = s0[4] - '0';             // 0: module; 1..4: controller
  if( com.con > 4 ) {
    stat.err |= ERR_MODBUS_CON;
  }

  if( stat.err != 0 ) {
    snprintf(s1,TX_CMD_SIZE,"NAK err=%04x",stat.err );
  }
  else {
    HAL_Delay(1);        // wait before answering to make dialog stable
    switch( st.rxReg ) {
      // ************** read commands **************************
      case 1:   // ping command; requires only an ACK to be sent back
        strcpy( s1, "ACK" );
      break;

      case 2:   // send status data
        if( com.con == 0 ) {
          // module status
          strcpy( s1, KENNUNG );
          strcat( s1, " ACK" ); // NOTE may send more module status data in later revision
        }
        else {
          // controller status
          v = com.con - 1;
          if( v > 3 ) {
            strcpy( s1, "NAK" ); 
          }
          else if( par[v].active == 0 ) {
            strcpy( s1, "INACTIVE" );
          }
          else {
            if( stat.summer[v]==1 ) {
              jz = 'S'; 
            }
            else {
              jz = 'W';
            }
            snprintf( s1, TX_CMD_SIZE, \
                    " %c VM%5.1f RM%5.1f VE%5.1f " \
                    "RE%5.1f RS%5.1f P%03.0f E%04X " \
                    "FX%d M%.0f A%d", \
                    jz,          stat.tv[v],     stat.tr[v],       stat.tvr[v], \
                    stat.trr[v], stat.trSoll[v], stat.valvePos[v], stat.errV[v], \
                    stat.fix[v],  stat.tMotTotal[v], stat.nMotLimit[v]); 
          }
        }
      break;

      case 3:   // send parameter data in use
        if( com.con == 0 ) {
          // module parameter: header for parameter values; variable names
          param_var_string( s1 );
        }
        else {
          // parameters for controllers as currently used
          param_var_values( s1 );
          }
      break;

      // ************** set commands **************************
      // command include parameters to set; starting from 0x20 
      case 0x20 :  // set Vorlauf temperature from Zentrale
        cp = strtok( s0, " " );
        cp = strtok( NULL, " " );
        f = strtod( cp, &cp0 );
        if( cp0 == s0 ) err0 = 1;
        else           err0 = 0;
        if( err0 > 0 ) {
          stat.tvzOk = 0;
          strcpy( s1, "NAK" );
        }
        else {
          stat.tvz = f;
          stat.tvzOk = par[v].tvzTValid;
          strcpy( s1, "ACK" );
        }
      break;

      case 0x21 :   // set parameters for a controller
        // s0 contains command string with parameters of one controller
        err0 = parse_param( s0 );
        if( err0 > 0 ) strcpy( s1, "NAK" );
        else {
          strcpy( s1, "ACK" );
          stat.cmdParReady = 1;
        }
      break;

      // ************** sys commands **************************
      // command include system operations;  starting from 0x30 
    
      case 0x30 :   // reset all parameters to factory settings
        // ATTENTION be careful using this command - all changes will be lost
        stat.parReset = 1;
        strcpy( s1, "ACK" );
      break;
    
      case 0x31 :   // open valve of addressed controller
        if( com.con > 0 && com.con < 5 ) {
          stat.cmdVLimit[com.con-1] = V_AUF;
          motor_stop_all();
          stop_activities();
          set_led(0,0);
          stat.actValve = com.con;
          par[com.con-1].vFix = V_AUF;
          stat.fix[com.con-1] = V_AUF;
          strcpy( s1, "ACK" );
        }
        else strcpy( s1, "NAK" );
      break;
    
      case 0x32 :   // close valve of addressed controller
        if( com.con > 0 && com.con < 5 ) {
          stat.cmdVLimit[com.con-1] = V_ZU;
          motor_stop_all();
          stop_activities();
          set_led(0,0);
          stat.actValve = com.con;
          par[com.con-1].vFix = V_ZU;
          stat.fix[com.con-1] = V_ZU;
          strcpy( s1, "ACK" );
        }
        else strcpy( s1, "NAK" );
      break;
    
      case 0x33 :   // set valve back to normal control
        if( com.con > 0 && com.con < 5 ) {
          stat.cmdVLimit[com.con-1] = 0;
          motor_stop_all();
          stop_activities();
          set_led(0,0);
          stat.actValve = com.con;
          par[com.con-1].vFix = V_HALT;
          stat.fix[com.con-1] = V_HALT;
          strcpy( s1, "ACK" );
        }
        else strcpy( s1, "NAK" );
      break;
    
      case 0x34 :   // set controller to inactive
        if( com.con > 0 && com.con < 5 ) {
          motor_stop_all();
          stop_activities();
          set_led(0,0);
          stat.actValve = com.con;
          par[com.con-1].active = 0;
          strcpy( s1, "ACK" );
        }
        else strcpy( s1, "NAK" );
      break;
    
      case 0x35 :   // set controller to active
        if( com.con > 0 && com.con < 5 ) {
          motor_stop_all();
          stop_activities();
          set_led(0,0);
          stat.actValve = com.con;
          par[com.con-1].active = 1;
          strcpy( s1, "ACK" );
        }
        else strcpy( s1, "NAK" );
      break;
    
      case 0x36 :   // fast mode on
        if( com.con > 0 && com.con < 5 ) {
          motor_stop_all();
          stop_activities();
          set_led(0,0);
          stat.actValve = com.con;
          par[com.con-1].fast = 255;
          strcpy( s1, "ACK" );
        }
        else strcpy( s1, "NAK" );
      break;
    
      case 0x37 :   // fast mode off
        if( com.con > 0 && com.con < 5 ) {
          motor_stop_all();
          stop_activities();
          set_led(0,0);
          stat.actValve = com.con;
          par[com.con-1].fast = 0;
          strcpy( s1, "ACK" );
        }
        else strcpy( s1, "NAK" );
      break;
    


      case 0x39 :   // write all parameters from RAM to EEPROM;
        stat.par2eeprom = 1;
        strcpy( s1, "ACK" );
      break;

      case 0x3A :   // RESET using watchdog - endless loop;
        strcpy( s1, "ACK" );
        wrap_modbus( ans, s1); // send answer immediately; does not reach end of switch()
        ser1_send_str( ans );
        HAL_Delay(500);        // time to send answer
        while(1);              // wait for watchdog resetting module
      break;

      default:
        strcpy( s1, "NAK" );
      break;
    }
    wrap_modbus( ans, s1);
    ser1_send_str( ans );
    stat.errV[v] = 0L;
  }
}













// ************************************
// RS485 - Modbus communication
// ************************************

// Data communication with RS485 Master
//  1. one MODULE has 4 CONTROLLERS which serve up to 4 valves.
//     each module has a unique 5 bit Address (ADR); jumpers on the printed circuit board (PCB).
//  1.1. the master (e.g. a PC) has the address 0; the modules have addresses e { 1...30 };
//  2. each module works independently; the 4 controllers work independently except for "Vorlauftemperatur"(TV).
//  3. There are 4 parameter sets for each controller.
//  4. There is a set of status variables for the module and for each controller.
//  5. The modules may be connected via RS485 bus using a modbus protocol to one master.
//     A master-slave concept is followed; the modules are slaves; act as servers.
//     for modbus see wikipedia "Modbus", chapter "ASCII protocol".
//  5.1. if ADR == 0 (no jumpers) the module indicates an error blinking all leds red
//  5.2. if ADR e {1, ... 30 } the module listens on the RS-485 bus for commands,
//       while performing control on all valves from parameters stored in static memory.
//  5.3. data transfer via RS485/modbus :
//  5.3.1. listen (on bus) until a data set is received
//  5.3.2. not matching address: do nothing
//  5.3.3. with matching address: check command checksum and modbus control byte
//         if not ok: respond with a "NAK" message
//  5.4.4. if ok: interpret command and send an appropriate answer (see below)
//  5.4.5. if the master sees no answer: after 0.5 sec: the bus is expected to be free

// Protocol string format:
// =======================
// ":<address><command><data><LRC><cr><lf>"
// with:
// ':'          leading character
// <address>    hex address e { '01' .. '1E' }(hex) <-> {1..30}(dez)
// <command>    two hex digits with command e {'01'..'FF'} <-> { 1 .. 255 }
// <data>       ASCII coded data string - see below
// <LRC>        modbus check byte; 2 character hex; see wikipedia link above
// <cr><lf>     carriage return - linefeed signals end ( 0x0D - 0x0A )

// Data content
// ------------
// Data contains the parts:
// "<A><bbb...><CCCC>" with:
// <A>        '0' for module specific commands
//            '1'...'4' for controller specific commands
// <bbb...>   more command specific data, e.g. parameters to be set
//            NOTE: if answer from slave the fist two chars represent the modules hex Address 
// <CCCC>     four hex-digit 16bit checksum including <address> up to last character from <bbb...>

// example:
// --------
// ":0101000F241.."
// ':'      header
// "01"     first 01 is the protocol
// "01"     second 01 is the command number 1
// data contains:
// '0'      for module related command
// "00F2"   is the checksum, a 16 bit hex string
// "41"     the LRC from modbus protocol
// ".."     the two not printable characters for <cr> and <lf>

// commands:
// ---------
// 01       ping command; the master wants an answer with "ACK" for acknowledge, "NAK" if not
//          or a timeout informs the master that the module did not receive the packet
//          this command addresses only the module; 
//    TODO  answer if controller is addressed is ignored and answered as above
// 02       send status data of module or selected controller; normal data set (short)
// 03       same as 02 but: send all status data (long)




 
