// rr_ctrl.c 
//

#include "rr.h"
#include "rr_io.h"


//
// check if a process is currently running needing motor movement
uint8_t motor_is_active( void ) {
  if( (stat.led          == 0)
    &&(stat.motControl   == 0)
    &&(stat.motMove      == 0)
    &&(stat.motInstall   == 0)
    &&(stat.motPos       == 0)
    ) {
    return 0;     // no process is active
  }
  else {
   return 1;     // some process is active
  }
}


void stop_activities( void ) {
  stat.led          = 0;
  stat.motControl   = 0;
  stat.motMove      = 0;
  stat.motInstall   = 0;
  stat.motPos       = 0;
}

/*
 verwendet Vorlauftemperatur stat.t_vor und berechnet
 anhand einer Kennlinie, ob die zugehoerige Ruecklauftemperatur
 stat.t_reck im Toleranzband pa.ttol liegt. 
 return: Richtung der Ventilbewegung:
   V_AUF  wenn zu niedrig 
   V_ZU   wenn zu hoch
   V_HALT wenn im Toleranzband
 gesetzte Parameter in stat:
   stat.trSoll   mit neuer Solltemperatur
   stat.dtmot    mit neuer Motorlaufzeit
   stat.dir      mit Richtung
 Falls ein Sensorfehler vorliegt erfolgt keine Berechnung;
 Die Ergebnisse werden alle auf 0 gesetzt.

tr1|- - - - - - - o-----
   |             /:
   |           /  :
t_r|- - - - -+    :      t_r = t_rueck
   |       / :    :
tr0|----o/   :    :
   |    :    :    :
   |    :    :    :
   +---------+----------
      tv0  t_vor  tv1
*/
uint8_t kennlinie( uint8_t valve ) {
  // valve e {1,2,3,4}
  // return: Motor direction e {V_ZU, V_HALT, V_AUF}
  uint8_t v = valve-1;     // v e {0,1,2,3} for valve {1,2,3,4}
  parameter_t pa;
  pa = par[v];             // point to parameter set of actual valve
  float dTemp;              

  // *** set time and direction in error case without calculating
  if( stat.errV[v] ) {
    stat.trSoll[v]    = -99.9;
    stat.dtMot[v]     = 0.0;
    stat.dir[v]       = V_HALT;
    return V_HALT;
  }

  // *** calculate Ruecklauftemperatur from Vorlauftemperatur using Kennlinie
  float m = (pa.tr1 - pa.tr0) / (pa.tv1 - pa.tv0);    // Steigung
  float y = m * ( stat.tvr[v] - pa.tv0 ) + pa.tr0;     // Sollwert Ruecklauftemperatur
  if( y < pa.tr0 ) y = pa.tr0;                        // limit to minimum tr0
  if( y > pa.tr1 ) y = pa.tr1;                        // limit to maximum tr1
  stat.trSoll[v] = y;                                 // set result in status
  dTemp = stat.tr[v] - y;                             // temperature difference

  if(stat.summer[v]==1) {
    stat.trSoll[v]=0.0;
    dTemp = 0.0;
  }
  // *** set time and direction for motion if out of tolerance
  if( dTemp > pa.ttol ) {
    stat.dtMot[v] = pa.dtMMn + pa.pFZu * (dTemp - pa.ttol);
    stat.dir[v] = V_ZU;
    return V_ZU;
  }
  else if( dTemp < -pa.ttol ) {
    stat.dtMot[v] = pa.dtMMn + pa.pFAuf * (-dTemp - pa.ttol);
    stat.dir[v] = V_AUF;
    return V_AUF;
  }
  else {
    stat.dtMot[v] = 0.0;
    stat.dir[v]   = V_HALT;
    return V_HALT;
  }
}




// check if winter or summer mode
// depending on valid Vorlauf temperature
uint8_t check_mode_summer( uint8_t v ) {
  float    tv = stat.tv[v];
  uint32_t zeit;

  if( stat.tvzOk > 0.0 ) {
    tv = stat.tvz;
  }

  uint8_t oldSummerMode = stat.summer[v];      // store old summer-flag
  if( tv > par[v].tvs ) stat.summer[v] = 0;    // is summer mode
  if( tv < par[v].tvw ) stat.summer[v] = 1;    // is winter mode
  if( (oldSummerMode==0) && (stat.summer[v]==1) ) {
    // change from winter to summer mode:
    zeit =  tic + par[v].dtS * DAYS2MSEC;
    stat.ticVSummer[v] = zeit;                 // msec; next valve movement in summer
    stat.motMove = 1;                          // move valve and end up in sommer position
  }
  if( (oldSummerMode==1) && (stat.summer[v]==0) ) {
    // change from summer to winter mode:
  }
  return stat.summer[v];
}


void install( void ) {
  // open ALL valves -> retract pin for installation
  static uint8_t valve;
  static uint8_t v;
  uint8_t motorStop;
  uint8_t color;

  valve         = stat.actValve;
  v             = valve - 1;


  // set to 1 for all opened valves (not incl. error/not active ones)
  static uint8_t ledsInstall;

  switch( stat.motInstall ) {
    case 1:                      // go through all valves; start settings
      motor_stop_all();          // all motors stop
      set_led(0,0);              // all leds off
      ledsInstall = 0;           // bit-falg; prepare flag-bits for all opened valves
      stat.actValve = 1;         // begin with first valve
      valve   = stat.actValve;
      v       = valve - 1;
      stat.motInstall++;       // goto next state
   break;
   case 2:
      
      if( (par[v].active==0) ) {
        stat.motInstall = 5;
      }
      else {
        stat.valvePos[v] = 50.0;   // start from any position
        stat.dir[v] = V_AUF;
        stat.dtMot[v] = par[v].dtMMx;
        color = motor_start();
        set_led( leds[v], color );
        ledsInstall |= leds[v];
        stat.motInstall++;
      }
    break;
    case 3:   // wait for motor stop
      motorStop = check_motor_stop( v );
      if( motorStop ) {
        motor_stop_all(); 
        set_led( 0, 0 );
        stat.ticWait = tic + 500; // msec; wait
        stat.motInstall++;
      }
    break;
    case 4:   // delay to let motor current go down
      if( (int32_t)(tic - stat.ticWait) >= 0 ) {
        stat.motInstall++;
      }
    break;
    case 5:   // switch to next valve
      stat.actValve++;
      valve         = stat.actValve;
      v             = valve - 1;
      if( stat.actValve > 4 ) {
        // up to 4 active valves are open (hot) now
        stat.actValve = 1;                               // reasonable value
        // timeout for keeping valves open
        stat.ticWait = tic + (uint32_t)(T_INSTALL*1000); // msec; wait 
        // show LEDs of all opened valves for installation:
        set_led( ledsInstall, WHITE );
        stat.motInstall = 6;                             // goto next state
      }
      else stat.motInstall = 2;                          // open next valve
    break;
    case 6:   // wait installation time; can be interrupted by pressing key again
      if( (int32_t)(tic - stat.ticWait) > 0 ) {
        stat.motInstall = 9;
      }
    break;
    case 9:
    default:
      stat.motInstall = 0;      // stop this sequence
      motor_stop_all();
      set_led(0,0);
      stat.summer[0]=0;         // start again in winter moder
      stat.summer[0]=0;
      stat.summer[0]=0;
      stat.summer[0]=0;
    break;
  }
}


void move( void  ) {
  // move active valve to avoid seizing
  // ->close ->open ->close some seconds again
  uint8_t valve = stat.actValve;
  uint8_t v = valve-1;
  uint8_t color;
  uint8_t motorStop;
  uint32_t msec;

  switch( stat.motMove ) {
    case 1:                      // close valve
      stat.valvePos[v] = 50.0;   // start from any position
      stat.dir[v] = V_ZU;
      stat.dtMot[v] = par[v].dtMMx;
      color = motor_start();
      set_led( leds[v], color );
      stat.motMove++;
    break;
    case 2:                     // wait for motor stop
      motorStop = check_motor_stop( v );
      if( motorStop ) {
        set_led( 0, 0 );
        stat.ticWait = tic + 500; // msec; wait 
        stat.motMove++;
      }
      if( stat.errV[v] > 0 ) {  // terminate - got here to flash LED, motor was off
        stat.motMove = 9; 
      }
    break;
    case 3:                     // delay to let motor current go down
      if( (int32_t)(tic - stat.ticWait) > 0 ) {
        stat.motMove++;
      }
    break;
    case 4:                     // open valve
      stat.dir[v] = V_AUF;
      stat.dtMot[v] = par[v].dtMMx;
      color = motor_start();
      set_led( leds[v], color );
      stat.motMove++;
    break;
    case 5:                     // wait for motor stop
      motorStop = check_motor_stop( v );
      if( motorStop ) {
        set_led( 0, 0 );
        stat.ticWait = tic + 500; // msec; wait 
        stat.motMove++;
      }
    break;
    case 6:                     // delay to let motor current go down
      if( (int32_t)(tic - stat.ticWait) > 0 ) {
        stat.motMove++;
      }
    break;
    case 7:                     // close valve until summer position is reached
      stat.dir[v] = V_ZU;
      stat.dtMot[v] = par[v].dtSZu;
      color = motor_start();
      set_led( leds[v], color );
      stat.motMove++;
    break;
    case 8:                     // wait for motor stop
      motorStop = check_motor_stop( v );
      if( motorStop > 0 ) {
        set_led( 0, 0 );
        stat.motMove++;
      }
    break;
    case 9:
    default:
      stat.motMove = 0;
      stat.motPos  = 0;
      motor_stop_all();
      leds_off();
      msec = tic + (uint32_t)(par[v].dtS * DAYS2MSEC); // msec; next valve movement
      if( par[v].fast > 0 ) {      // fast check
        par[v].fast--;
        msec = tic + 5*60*1000;    // every 5 minutes a valve movement during summer
      }
      stat.ticVSummer[v] = msec;
    break;
  }
}


void control( void ) {
  uint8_t  valve = stat.actValve;   // e  {1,2,3,4}
  uint8_t  v = valve -1;
  uint8_t  color, motorStop;

  switch( stat.motControl ) {
    case 1:   // start control cycle displaying LED
      kennlinie( valve );           // on sensor error parameters are set to 0
      color    = WHITE;             // default color
      if     ( par[v].active == 0 ) { color = CYAN; }              // inactive
      else if( stat.errV[v]   > 0 ) { color = MAGENTA; }           // error 
      else if( stat.summer[v]== 0 ) { color = GREEN; }             // active, winter mode
      else                          { color = YELLOW; }            // active, summer mode
      stat.color[v] = color;
      set_led( leds[v], stat.color[v] );
      stat.ticLedOff = tic + (uint32_t)(par[v].dtLed * 1000.0);  // LED stop time
      stat.motControl++;
    break;
    case 2:   // switch off LED; start motor if applicable
      if( (int32_t)(tic - stat.ticLedOff) >= 0 ) {
        set_led( 0, 0 );

        if( (par[v].active==0)||(stat.dtMot[v])==0.0 ) { stat.motControl = 9; } // no motor -> end
        else if( (stat.dir[v] == V_ZU ) && (stat.valvePos[v] ==  0.0) ) { stat.motControl = 9; } // end
        else if( (stat.dir[v] == V_AUF) && (stat.valvePos[v] ==100.0) ) { stat.motControl = 9; } // end
        else {
          color = motor_start();
          set_led( leds[v], color );
          stat.motControl++;
        }
      }
    break;
    case 3:   // motor over-current or time elapsed -> stop motor movement
      motorStop = check_motor_stop( v );  // 1: timeout; 2: over-current; 3: under-current
      if( motorStop ) {
        set_led( 0, 0 );
        motor_stop_all();
        stat.motControl = 9;
      }
    break;
    case 9:   // end of sequence
    default:
      motor_stop_all();
      set_led( 0, 0 );
      stat.dir[v]        = V_HALT;
      stat.dtMot[v]      = 0.0;
      stat.motControl    = 0;    // reset state for next cycle
    break;
  }
}


// blink LED of current controller to indicate summer check interval
void led_blink( void ) {
  uint8_t valve = stat.actValve;
  uint8_t v = valve - 1;
  switch( stat.led ) {
    case 1:
      if( par[v].active == 0 ) { set_led( leds[v], CYAN   ); }
      else {
        set_led( leds[v], stat.ledColor );
      }
      stat.ticLedOff = tic + par[v].dtLed * 1000.0;
      stat.led++;
    break;
    case 2:
      if( (int32_t)(tic - stat.ticLedOff) > 0 ) {
        stat.led++;
      }
    break;
    case 3:
    default:
      set_led( 0, 0 );
      stat.led = 0;
    break;
  }
}


void mot_pos( void ) {
  // position motor for some time;
  // input via status structure:
  // direction and time are set before starting this process:
  //    stat.dir e {V_AUF, V_ZU}
  //    stat.dtMot[v] = time to move motor sec;

  uint8_t valve = stat.actValve;
  uint8_t v = valve-1;
  uint8_t color;
  uint8_t motorStop;

  switch( stat.motPos ) {
    case 1:                         // start motor
      if(stat.errV[v] > 0 ) {       // some error from measurements -> no movements
        stat.motPos = 9;            // motor positioning not applicable
        stat.fix[v] = stat.dir[v];
        if( stat.dir[v] == V_AUF ) {
          stat.valvePos[v] = 100.0; // emulate open valve
        }
        else {
          stat.valvePos[v] = 0.0;   // emulate closed valve
        }
      }
      color = motor_start();
      set_led( leds[v], color );
      stat.motPos++;
    break;
    case 2:                     // wait for motor stop
      motorStop = check_motor_stop( v );
      if( motorStop ) {
        motor_stop_all();
        leds_off();
        stat.ticWait = tic + 500; // msec; wait 
        stat.motPos++;
      }
      if( stat.errV[v] > 0 ) {
        stat.motPos = 9; 
      }
    break;
    case 3:                     // delay to let motor current go down
      if( (int32_t)(tic - stat.ticWait) > 0 ) {
        stat.motPos++;
      }
    break;
    case 4:
    case 9:
    default:
      stat.motPos = 0;          // stop motor positioning
      motor_stop_all();
      leds_off();
      stat.cmdVLimit[v] = V_HALT; // do not start positioning again
    break;
  }
}