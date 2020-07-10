// HZ-RR010
// Heizungs Ruecklauf Regler, Version 01.0
//
// Peter Loster (c); 2015, 2016
//

#include "iwdg.h"
#include "rr.h"
#include "rr_io.h"
#include "rr_ser.h"
#include "rr_ctrl.h"
#include "rr_test.h"
#include "rr_autogene.h"

// ----- variables
uint32_t  tic;            // is set once in mainloop; used for whole timeslice
float     ticSec;         // tick in seconds
parameter_t par[4];       // 4 parameter sets for 4 regulators
parameter_t parx;         // 1 parameter set as received from serial; temporarily
parameter_t pl[2];        // parameter limits; [0]min., [1] max. values
volatile status_t stat;
command_t com;            // serial communication / modbus

uint8_t leds[]   = { LED1, LED2, LED3, LED4 };
uint8_t colors[] = { BLACK, RED, MAGENTA, BLUE, CYAN, GREEN, YELLOW, WHITE };
uint8_t motors[] = { m1l, m1r, m2l, m2r, m3l, m3r, m4l, m4r };
uint8_t motorNr[]= { 1, 2, 3, 4 };




// clear all error flags
void err_clear( void ) {
  uint_fast8_t i;
  stat.err = 0;
  for( i=0; i<4; i++ ) {
    stat.errV[i] = 0;
  }
}


uint32_t check_fast( uint32_t tics, uint8_t v ) {
  // in case of fast switch is set set intervals shorter; 
  // count down fast flag to finally perform normal operation
  if( par[v].fast > 0 ) {
    par[v].fast--;
    tics = tics / 6;
  }
  return tics;
}



// **********************************************
// main loop
// **********************************************
// no function is blocking; the whole system is polling and stat-driven

void rr( void ) {
  int8_t valve, v;
  uint32_t tics = 0;

  rr_init();
  rr_selftest();

  rr_measure();     // remove garbage from input register chains
  rr_measure();
  rr_measure();
  rr_measure();
  valve = stat.actValve;
  v = 0;                    // v is index {0..3} for valves {1..4}

  HAL_StatusTypeDef halstat;          // watchdog
  halstat = HAL_IWDG_Start( &hiwdg );
  if( keypressed() ) {
    // test fuer Inbetriebnahem, TODO
    // rr_test();
    while(1);                         // watchdog test
  }


  for( EVER ) {
    halstat = HAL_IWDG_Refresh(&hiwdg);
    tic = HAL_GetTick();
    stat.tic = tic;
    ticSec = 0.001 * (float)tic;

    ser1_rx_check();        // check if serial 1 read with interrupt is active, if not start it
    err_clear();

    get_address();          // block and signal error via LEDs until valid address is set with jumpers

    if( stat.cmdParReady ) {// take over newly received parameters for a valve  
      memcpy( (uint8_t*)(&par[com.con - 1]), (uint8_t*)&parx, sizeof( parameter_t ) );
      stat.cmdParReady = 0;
    }

    if( stat.parReset ) {   // set parameters back to factory setting
      stat.parReset = 0;
      set_param_limits();   // reset limits just to be sure - should never change
      if( com.con == 0 ) {  // module address
        param2default();    // reset parameters of all valves
      }
      else if( com.con < 5 ) { // only one controller
        param2default_valve( &par[com.con-1] );
      }
    }

    if( stat.par2eeprom ) {  // write all parameters to EEPROM
      stat.par2eeprom = 0;
      eeprom_write();
    }

    // *** perform current movements, else change active valve or start new process

    rr_measure();

    // check if key was pressed -> open all valves for installation
    if( keychange() == 1 ) {         // key status changed from off to on
      if( stat.motInstall == 0 ) {
        // stop other processes
        stop_activities();
        stat.motInstall = 1;
      }
      else {
        // stop opening valves for installation
        motor_stop_all();
        stat.motInstall = 9;     // terminate function
      }
    }

    check_mode_summer( v );


    // *** action - perform movements set before

    if(      stat.led        > 0 ) led_blink();           // flash led
    else if( stat.motControl > 0 ) control();          // control active valve
    else if( stat.motMove    > 0 ) move();             // move active valve
    else if( stat.motInstall > 0 ) install();          // all valves in install position
    else if( stat.motPos     > 0 ) mot_pos();          // position motor valve for some time

    if( motor_is_active() == 0 ) {
      // *** program gets here only if no motor or LED process is pending
      // --- immediate reaction
      valve = stat.actValve;                           // verify proper valve
      v = valve -1;                                    // valve index
      if( stat.cmdVLimit[v] != V_HALT ) {
        // position valve to extreme position
        if( stat.cmdVLimit[v] == V_AUF ) {
          stat.dir[v] = V_AUF;
          stat.dtMot[v] = par[v].dtMMx;         // time
          stat.motPos = 1;                      // start positioning thread
        }
        if( stat.cmdVLimit[v] == V_ZU ) {
          stat.dir[v] = V_ZU;
          stat.dtMot[v] = par[v].dtMMx;         // time
          stat.motPos = 1;                      // start positioning thread
        }
        stat.cmdVLimit[v] = V_HALT;
      }
      else {
        // next valve:
        valve++;
        if( valve > 4 ) valve = 1;                       // start from first valve again
        stat.actValve = valve;
        v = valve -1;                                    // valve index

        if( par[v].vFix != 0 ) {
          // keep valve in fixed position
          if( par[v].vFix == V_AUF ) {
            if(stat.valvePos[v] < 95.0 ) {
              stat.dir[v] = V_AUF;
              stat.dtMot[v] = par[v].dtMMx;         // time
              stat.motPos = 1;                      // start positioning         
              stat.fix[v] = par[v].vFix;
            }
          }
          else if( par[v].vFix == V_ZU ) {
            if(stat.valvePos[v] > 5.0 ) {
              stat.dir[v] = V_ZU;
              stat.dtMot[v] = par[v].dtMMx;         // time
              stat.motPos = 1;                      // start positioning thread
              stat.fix[v] = par[v].vFix;
            }
          }
          else {
            // par[v].vFix was not V_AUF or V_ZU -> deactivate fixed valve position
            par[v].vFix = 0;
            stat.fix[v] = 0;
          }
        }

        if( (int32_t)(tic - stat.ticIntNew[v]) >= 0 ) {
          // --- interval has expired

          if( par[v].vFix > 0 ) {
            // valve is in fixed position
            tics = (uint32_t)( 1000.0 * par[v].dtw );  // use winter intervals
            tics = check_fast( tics, v );
            stat.ledColor = WHITE;       // preset - will be changed below
            if( par[v].vFix == V_AUF ) {
              // valve is in fixed position
              stat.ledColor = RED;
            }
            else if( par[v].vFix == V_ZU ) {
              stat.ledColor = BLUE;
            }
            stat.led = 1;  // blink led
          }

          else if( stat.errV[v] > 0 ) {
            // error measured; e.g. no sensor for ruecklauf, no value for vorlauf
            // blink led to indicate scan of channel
            stat.ledColor = MAGENTA;
            stat.led = 1;
          }
          
          else if( stat.summer[v]==1 ) {
            // summer mode
            if( (int32_t)(tic - stat.ticVSummer[v]) >= 0 ) {
              // valve movement against seizing
              if( stat.motMove > 0 ) {          // motor is busy:
                stat.ticVSummer[v] += 60000;    // try again in 1 minute
              }
              else {
                stat.motMove = 1;
              }
            }
            else {
              // normal summer check-interval
              tics = (uint32_t)( 1000.0 * par[v].dts );
              tics = check_fast( tics, v );
              stat.ledColor = YELLOW;
              stat.led = 1;                          // blink LED
            }
          }
          else {
            // winter mode
            tics = (uint32_t)( 1000.0 * par[v].dtw );
            tics = check_fast( tics, v );
            if( (stat.valvePos[v] == 0.0)&&(stat.fix[v]==0) ) {
              // if valve is closed completely and not in fixed position, 
              // open it for some seconds to let water flow
              stat.dir[v] = V_AUF;                  // set direction
              stat.dtMot[v] = par[v].dtWAuf;
              stat.motPos = 1;                      // start positioning thread
            }
            else if( stat.tvr[v] < par[v].tv0 ) {
              // ruecklauf below tv0 degC -> open valve for dtWauf sec to make sure
              // that hot water reaches the vorlauf temperature sensor
              stat.dir[v] = V_AUF;
              stat.dtMot[v] = par[v].dtWAuf;        // end-time
              stat.motPos = 1;                      // start positioning thread
            }
            else stat.motControl = 1;               // start control cycle 
          }

          stat.ticIntNew[v] = tic + tics;                 // set time for next interval
        }
      }
    }
  }  // eo FOREVER
}


