// HZ-RR010
// Heizungs Ruecklauf Regler, Version 01.0
//
// Peter Loster (c); 2015, 2016
//
// peripheral input output functions
//

#include "rr.h"
#include "rr_io.h"
#include "rr_ser.h"
#include "rr_autogene.h"
#include "adc.h"


// **************************
// Error handling 
// **************************
void error_handler( uint32_t errNum ) {
  for( EVER );                          // wait for watchdog
}


// **************************
// initialize variables
// **************************




void param2default( void ) {
  int i;
  for( i=0; i<4; i++ ) {
    param2default_valve( &par[i] );
  }
}


void status2default( void ) {
  uint8_t   v;
  stat.err    = 0;
  stat.adcConvCplt = 0;
  stat.tx1Busy     = 0;
  stat.rx1Eol      = 0;
  stat.cmdParReady = 0;           // nr of command to be executed
  stat.parReset    = 0;           // do not set parameters to factory settings
  stat.adr         = 0xFF;        // address from jumpers: 0x0 break case. 0x1F for RS485 master
  stat.vsup        = 0.0;         // V; supply voltage
  stat.tref        = 0.0;
  stat.vmot        = 0.0;         // V; voltage on motor = vsup - volts(VMhalf)
  stat.imot        = 0.0;         // mA; voltage on Rsense / Rsense
  stat.tempInt     = 0.0;
  stat.tvz         = 0.0;         // deg C; temp. Vorlauf; von Zentrale gesendet
  stat.tvzOk       = 0.0;         // sec;   >0.0 if tvz is still valid; 
  stat.rx1Running  = 0;           // start receiving from uart1 is needed
  stat.actValve    = 1;           // e {1..4} valve number to be next for control
  // *** flags for motor control
  stat.flMotLock   = 0;           //motor nr. e {1,2,3,4} if a motor is active; block other requests
  stat.ticMotStop  = 0L;          // msec; tic count when motor has to be switched off
  stat.ticLedOff   = 0L;          // msec;  tick count when led has to be switched off
  stat.motControl  = 0;           //  0:inactive; else state counter for control cycle
  stat.motInstall  = 0;           //  0:inactive; else state counter for open valve for installation
  stat.motMove     = 0;           //  0:inactive; else state counter for moving valves in summer modee
  stat.led         = 0;           //  0:inactive; else state of led indicator
  stat.ledColor    = 0;           //  color of led to blink / show state
  stat.motPos      = 0;           //  0:inactive; else state of direct motor movement
  // *** for each valve indexed values
  for( v=0; v<4; v++ ) {
    stat.summer[v]     = 0;      // 1;     1: if summer, 0: winter operation
    stat.ticVSummer[v] = par[v].dtS * DAYS2MSEC; // msec; next valve movement in summer
    stat.errV[v]       = 0;      // error values for each valve
    stat.ticIntNew[v]  = 0;      // msec;  tic count when a new interval starts
    stat.flLedOn[v]    = 0;      // 1      if LED is on; else 0
    stat.flMotRequ[v]  = 0;      // 1      if motor has to be switched on
    stat.flMotAct[v]   = 0;      // 1      if motor sequence of current controller is activated
    stat.dir[v]        = V_HALT; //        direction to drive
    stat.cmdVLimit[v]  = V_HALT;
    stat.par2eeprom    = 0;    
    stat.dtMot[v]      = 0;      // sec;   time to drive motor for regulation
    stat.tMotTotal[v]  = 0;      // sec;   total time of all motor movements
    stat.tvr[v]        = 0.0;    // deg C; temperature Vorlauf, used for control
    stat.sVStat[v]     = T_SENSOR_OK;   // sensor Vorlauf status
    stat.tr[v]         = 0.0;    // deg C; temperature Ruecklauf
    stat.trr[v]        = 0.0;    // deg C; temperature Ruecklauf, used for control
    stat.sRStat[v]     = 0;      // sensor Ruecklauf status: {0,1,2} = {in range, not connected, short circuit}
    stat.trSoll[v]     = 0.0;    // deg C; set temperature, Ruecklauf
    stat.valvePos[v]   = 50.0;   // %;     estimated position of motor valve
  }
}


void init_variables( void ) {


  stat.adcConvCplt = 0;
  set_param_limits();
  eeprom_read();        //

  status2default();     // NOTE AFTER parameter settings !!!
  #if TEST_FAST == 1
  int i;
  for( i=0; i<4; i++ ) {
    par[i].fast = 255;
  }
  #endif



}




// **************************
// EEPROM functions
// **************************

uint32_t checksum32( uint8_t *s, uint16_t count ) {
  // checksum over s[], from start index si for count bytes
  uint32_t   sum = 0;
  uint16_t   i;

  for( i=0; i < count; i++ ) {
    sum += s[i];
  }
  return sum;    
}



void eeprom_put( uint8_t v ) {
  // write data with valve index v (0..3) from parx to eeprom
  // data starts at a fixed boundary for every valve
  HAL_StatusTypeDef  est;
  uint32_t           cs;
  uint32_t           eadrBase  = (uint32_t)(DATA_EEPROM_BASE + v * EEPROM_PAR_SIZE );
  uint32_t           eadr      = eadrBase;
  uint32_t*          wptr      = (uint32_t *)(&parx);
  uint16_t           bcount    = sizeof(parameter_t);
  uint16_t           wcount    = bcount / 4 + 1;      // can only read 32bit words

  // add checksum to parx
  cs = checksum32( (uint8_t*)(&parx), bcount - 12 );
  parx.checksum = cs;

  // now write data 32bit word wise
  est = HAL_FLASHEx_DATAEEPROM_Unlock();
  if( est != HAL_OK ) error_handler( ERR_EEPROM_UNLOCK );
  while( wcount ) {
    est = HAL_FLASHEx_DATAEEPROM_Program(  FLASH_TYPEPROGRAMDATA_WORD, eadr, *wptr);
    if( est != HAL_OK ) error_handler( ERR_EEPROM_WRITE );
    eadr+=4;
    wptr++;
    wcount--;
  }
  est = HAL_FLASHEx_DATAEEPROM_Lock();
  if( est != HAL_OK ) error_handler( ERR_EEPROM_WRITE );
}


void eeprom_write( void ) {
  uint8_t v;

  // fill in all checksums
  for( v=0; v<4; v++ ) {
    memcpy( (uint8_t*)&parx, (uint8_t*)&par[v], sizeof(parx) );
    eeprom_put(v);
  }
}


//
// read eeprom data to parameters
//
uint16_t eeprom_get( uint8_t v) {
  // read one parameter set v e{0..3} from eeprom to parx
  uint32_t           cs, csStored;
  uint16_t           err    = 0;
  uint32_t*          eadr   = (uint32_t*)(DATA_EEPROM_BASE + v * EEPROM_PAR_SIZE );
  uint32_t*          eptr   = eadr;
  uint32_t*          wPtr   = (uint32_t *)(&parx);
  uint16_t           bcount = sizeof(parameter_t);
  uint16_t           wcount = bcount / 4 + 1;      // can only read 32bit words

  if( wcount > EEPROM_PAR_SIZE ) err++;
  else {
    while( wcount ) {
      *wPtr++ = *eptr++;
      wcount--;
    }
    cs       = checksum32( (uint8_t*)(&parx), bcount - 12 );
    csStored = parx.checksum;
    if( cs != csStored ) err++;
    if( strcmp( parx.kenn, KENNUNG ) != 0 ) err++;
    if( err ) {
      param2default_valve( & parx );  // read factory settings
      eeprom_put( v );
    }
    memcpy( (uint8_t*)&par[v], (uint8_t*)&parx, bcount );
  }
  return err;
}


uint8_t eeprom_read( void ) {
  // read parameter sets from eeprom
  // if there is an error (checksum) parameters are set
  // to factory settings; factory settings are written to EEPROM
  uint16_t  err;
  uint8_t   v;
  for( v=0; v<4; v++ ) {
    err = eeprom_get( v );
  }
  return err;
}









// ****************************************
// analog digital converter (adc) functions
// ****************************************

//  ADC GPIO Configuration    
//    PA0     ------> ADC_IN0  - TV1
//    PA1     ------> ADC_IN1  - TR1
//    PA2     ------> ADC_IN2  - TV2
//    PA3     ------> ADC_IN3  - TR2
//    PA4     ------> ADC_IN4  - TV3
//    PA5     ------> ADC_IN5  - TR3
//    PA6     ------> ADC_IN6  - TV4
//    PA7     ------> ADC_IN7  - TR4
//    PC1     ------> ADC_IN11 - VShalf
//    PC2     ------> ADC_IN12 - I_Motor
//    PC3     ------> ADC_IN13 - TREF

enum {
  adc_tv1, adc_tr1, adc_tv2, adc_tr2, adc_tv3, adc_tr3, adc_tv4, adc_tr4,
  adc_vshalf, adc_imotor, adc_tref, adc_intVRef, adc_intTemp };
#define ADC_CHANNEL_CNT  13
int16_t adc_raw[ ADC_CHANNEL_CNT ];
float   adcMeanSum[ ADC_CHANNEL_CNT ];  // sum of several adc_raw values

void adc_fehler( int16_t fehlerNr ) {
  for( EVER );
}


void adc_calib( void ) {
  HAL_StatusTypeDef st;
  if( (st=HAL_ADCEx_Calibration_Start( &hadc, ADC_SINGLE_ENDED)) != HAL_OK ) {
    adc_fehler(0);
  }
  return;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  stat.adcConvCplt = 1;
}


void adc_readAllDMA( void ) {
  HAL_StatusTypeDef st;
  adc_calib();
  stat.adcConvCplt = 0;
  st = HAL_ADC_Start_DMA( &hadc,(uint32_t *) adc_raw, (uint32_t)ADC_CHANNEL_CNT );
  if( st != HAL_OK ) adc_fehler( 10 );
  while( !stat.adcConvCplt );  // wait for adc conversion ready
  st = HAL_ADC_Stop_DMA( &hadc );
  if( st != HAL_OK ) adc_fehler( 10 );
}


void adc_init( void ) {
  stat.adcConvCplt = 0;
  adc_calib();
}



// *****************************
// GPIO digital I/O functions
// *****************************


#define NVTemp_Pin GPIO_PIN_0
#define NVTemp_GPIO_Port GPIOC


void set_pin( uint8_t pin, uint8_t on ) {
  // pin nr from enum
  // on = 0 -> set output low; 1 -> set output high
  switch( pin ) {
    case m1l:
      if( on ) HAL_GPIO_WritePin( M1L_GPIO_Port, M1L_Pin, 1 );
      else     HAL_GPIO_WritePin( M1L_GPIO_Port, M1L_Pin, 0 );
      break; 
    case m1r:
      if( on ) HAL_GPIO_WritePin( M1R_GPIO_Port, M1R_Pin, 1 );
      else     HAL_GPIO_WritePin( M1R_GPIO_Port, M1R_Pin, 0 );
      break;
    case m2l:
      if( on ) HAL_GPIO_WritePin( M2L_GPIO_Port, M2L_Pin, 1 );
      else     HAL_GPIO_WritePin( M2L_GPIO_Port, M2L_Pin, 0 );
      break;
    case m2r:
      if( on ) HAL_GPIO_WritePin( M2R_GPIO_Port, M2R_Pin, 1 );
      else     HAL_GPIO_WritePin( M2R_GPIO_Port, M2R_Pin, 0 );
      break;
    case m3l:
      if( on ) HAL_GPIO_WritePin( M3L_GPIO_Port, M3L_Pin, 1 );
      else     HAL_GPIO_WritePin( M3L_GPIO_Port, M3L_Pin, 0 );
      break;
    case m3r:
      if( on ) HAL_GPIO_WritePin( M3R_GPIO_Port, M3R_Pin, 1 );
      else     HAL_GPIO_WritePin( M3R_GPIO_Port, M3R_Pin, 0 );
      break;
    case m4l:
      if( on ) HAL_GPIO_WritePin( M4L_GPIO_Port, M4L_Pin, 1 );
      else     HAL_GPIO_WritePin( M4L_GPIO_Port, M4L_Pin, 0 );
      break;
    case m4r:
      if( on ) HAL_GPIO_WritePin( M4R_GPIO_Port, M4R_Pin, 1 );
      else     HAL_GPIO_WritePin( M4R_GPIO_Port, M4R_Pin, 0 );
      break;
    case red:
      if( on ) HAL_GPIO_WritePin( NRed_GPIO_Port, NRed_Pin, 1 );
      else     HAL_GPIO_WritePin( NRed_GPIO_Port, NRed_Pin, 0 );
      break;
    case green:
      if( on ) HAL_GPIO_WritePin( NGreen_GPIO_Port, NGreen_Pin, 1 );
      else     HAL_GPIO_WritePin( NGreen_GPIO_Port, NGreen_Pin, 0 );
      break;
    case blue:
      if( on ) HAL_GPIO_WritePin( NBlue_GPIO_Port, NBlue_Pin, 1 );
      else     HAL_GPIO_WritePin( NBlue_GPIO_Port, NBlue_Pin, 0 );
      break;
    case led1:
      if( on ) HAL_GPIO_WritePin( NLED1_GPIO_Port, NLED1_Pin, 1 );
      else     HAL_GPIO_WritePin( NLED1_GPIO_Port, NLED1_Pin, 0 );
      break;
    case led2:
      if( on ) HAL_GPIO_WritePin( NLED2_GPIO_Port, NLED2_Pin, 1 );
      else     HAL_GPIO_WritePin( NLED2_GPIO_Port, NLED2_Pin, 0 );
      break;
    case led3:
      if( on ) HAL_GPIO_WritePin( NLED3_GPIO_Port, NLED3_Pin, 1 );
      else     HAL_GPIO_WritePin( NLED3_GPIO_Port, NLED3_Pin, 0 );
      break;
    case led4:
      if( on ) HAL_GPIO_WritePin( NLED4_GPIO_Port, NLED4_Pin, 1 );
      else     HAL_GPIO_WritePin( NLED4_GPIO_Port, NLED4_Pin, 0 );
    break;
    case de485:
      if( on ) HAL_GPIO_WritePin( RS485_DE_GPIO_Port, RS485_DE_Pin, 1 );
      else     HAL_GPIO_WritePin( RS485_DE_GPIO_Port, RS485_DE_Pin, 0 );
      break;
    case vtemp:
      if( on ) HAL_GPIO_WritePin( NVTemp_GPIO_Port, NVTemp_Pin, 1 );
      else     HAL_GPIO_WritePin( NVTemp_GPIO_Port, NVTemp_Pin, 0 );
      break;
    default:
      break;
  }
}


void set_VTemp( uint8_t on ) {
  if( on == 1 ) set_pin( vtemp, 0 );
  else          set_pin( vtemp, 1 );
}



void set_pins2default( void ) {
  set_pin( m1l, 1 );
  set_pin( m1r, 1 );
  set_pin( m2l, 1 );
  set_pin( m2r, 1 );
  set_pin( m3l, 1 );
  set_pin( m3r, 1 );
  set_pin( m4l, 1 );
  set_pin( m4r, 1 );
  set_pin( red, 1 );
  set_pin( green, 1 );
  set_pin( blue, 1 );
  set_pin( led1, 0 );
  set_pin( led2, 0 );
  set_pin( led3, 0 );
  set_pin( led4, 0 );
  set_pin( de485, 0 );
  set_pin( vtemp, 1 );
}


// gpio input pin numbers
enum{ adr0, adr1, adr2, adr3, adr4 };

uint8_t get_pin( uint8_t pin ) {
  GPIO_PinState ps=0;
  switch( pin ) {
    case adr0:
      ps = HAL_GPIO_ReadPin( NADR0_GPIO_Port, NADR0_Pin);
    break;
    case adr1:
      ps = HAL_GPIO_ReadPin( NADR1_GPIO_Port, NADR1_Pin);
    break;
    case adr2:
      ps = HAL_GPIO_ReadPin( NADR2_GPIO_Port, NADR2_Pin);
    break;
    case adr3:
      ps = HAL_GPIO_ReadPin( NADR3_GPIO_Port, NADR3_Pin);
    break;
    case adr4:
      ps = HAL_GPIO_ReadPin( NADR4_GPIO_Port, NADR4_Pin);
    break;
    default:
    break;
  }
  if( ps ) return 1;
  else     return 0;
}


// *** RS485 address from 5 jumpers

// ATTENTION on error: blocking
uint16_t get_address( void ) {
  uint16_t b0,b1,b2,b3,b4;
  uint16_t a=0;

  do {
    b0 = get_pin(adr0);
    b1 = get_pin(adr1);
    b2 = get_pin(adr2);
    b3 = get_pin(adr3);
    b4 = get_pin(adr4);
    a = ~(b4<<4 | b3<<3 | b2<<2 | b1<<1 | b0) & 0x1F;
    if( a > 0x1E ) a=0;          // max 30=0x1E addresses valid from 1 to 30 dez.
    if( a== 0 ) {
      led_blink_err( 1 );
    }
  } while( a==0 );
  stat.adr = a;
  return a;
}





// *** switch on LEDs
// 4 red-green-blue (rgb) LEDs are positioned next to each regulator terminal block
// each LED can be activated separately; 
// all activated LEDs show the same color because the r, g and b lines are connected together
//
// during regulation there are two phases indicating the status of each channel
//   1. for a short time status 
//      CYAN     not active
//      YELLOW   summer mode
//      GREEN    winter mode
//      MAGENTA  sensor error      is displayed
//   2. during motor movement 
//      RED      for opening valve - retracting motor pin
//      BLUE     for closing valve - extracting motor pin  is displayed
//   3. if install mode was selected pressing the install button:
//      WHITE    after all active motor-drives / valves are open for installation
//
// The following colors for each LED are defined:
// BLACK   rgb leds off
// BLUE    valve closing (colder)
// GREEN   winter mode
// YELLOW  summer mode
// RED     valve opening (hotter)
// CYAN    inactive
// MAGENTA error
// WHITE   open for installation
//
void set_led( uint8_t leds, uint8_t color ) {
  // activate / deactivate LEDs; negative logic
  if( leds & LED1 ) HAL_GPIO_WritePin( NLED1_GPIO_Port, NLED1_Pin, 1 );
  else              HAL_GPIO_WritePin( NLED1_GPIO_Port, NLED1_Pin, 0 );
  if( leds & LED2 ) HAL_GPIO_WritePin( NLED2_GPIO_Port, NLED2_Pin, 1 );
  else              HAL_GPIO_WritePin( NLED2_GPIO_Port, NLED2_Pin, 0 );
  if( leds & LED3 ) HAL_GPIO_WritePin( NLED3_GPIO_Port, NLED3_Pin, 1 );
  else              HAL_GPIO_WritePin( NLED3_GPIO_Port, NLED3_Pin, 0 );
  if( leds & LED4 ) HAL_GPIO_WritePin( NLED4_GPIO_Port, NLED4_Pin, 1 );
  else              HAL_GPIO_WritePin( NLED4_GPIO_Port, NLED4_Pin, 0 );
  // switch on color; negative logic
  if( color & RED   ) HAL_GPIO_WritePin( NRed_GPIO_Port,   NRed_Pin,   0 );
  else                HAL_GPIO_WritePin( NRed_GPIO_Port,   NRed_Pin,   1 );
  if( color & GREEN ) HAL_GPIO_WritePin( NGreen_GPIO_Port, NGreen_Pin, 0 );
  else                HAL_GPIO_WritePin( NGreen_GPIO_Port, NGreen_Pin, 1 );
  if( color & BLUE  ) HAL_GPIO_WritePin( NBlue_GPIO_Port,  NBlue_Pin,  0 );
  else                HAL_GPIO_WritePin( NBlue_GPIO_Port,  NBlue_Pin,  1 );
}


void leds_off( void ) {
  HAL_GPIO_WritePin( NLED1_GPIO_Port, NLED1_Pin, 1 );
  HAL_GPIO_WritePin( NLED2_GPIO_Port, NLED2_Pin, 1 );
  HAL_GPIO_WritePin( NLED3_GPIO_Port, NLED3_Pin, 1 );
  HAL_GPIO_WritePin( NLED4_GPIO_Port, NLED4_Pin, 1 );
  HAL_GPIO_WritePin( NRed_GPIO_Port,   NRed_Pin,   1 );
  HAL_GPIO_WritePin( NGreen_GPIO_Port, NGreen_Pin, 1 );
  HAL_GPIO_WritePin( NBlue_GPIO_Port,  NBlue_Pin,  1 );
}


// ATTENTION blocking !
void led_blink_err( uint8_t errNr ) {
  switch( errNr ) {
    case 1:
      set_led( LED1 | LED2 | LED3 | LED4, RED );
    break;
      set_led( LED1 | LED2 | LED3 | LED4, MAGENTA );
    case 2:
    break;
    case 3:
    break;
    default:
    break;
  }
  HAL_Delay( 100 );
  leds_off();
  HAL_Delay( 400 );
}


void test_led( void ) {
  set_led( LED1 | LED2 | LED3 | LED4, RED );
  HAL_Delay(500);
  set_led( LED1 | LED2 | LED3 | LED4, GREEN );
  HAL_Delay(500);
  set_led( LED1 | LED2 | LED3 | LED4, BLUE );
  HAL_Delay(500);
  set_led( LED1 | LED2 | LED3 | LED4, 0 );
}










// *** Ventil Motorantrieb

//   motor bridge
// * ============
// * the motor is connected to a bridge to enable forward and backward rotation
// * 
// *             +U.suppy, 3.3V typ
// *                     |
// *          +----------o----------+
// *          |                     |
// *     +--Pch MOSFET P1         Pch MOSFET P2--+
// *     |    |                     |            |
// * mxl-o    o------- MOTOR -------o            o-mxr
// *     |    |                     |            |
// *     +--Nch MOSFET N1         Nch MOSFET N2--+
// *          |                     |
// *          +----------o----------+----> U.current
// *                     |
// *                  Rmot 3.3 Ohm(current sense)
// *                     |
// *                    GND (0V)
// *             

// ATTENTION 
// N1 and N2 are the gates of the N-channel MOS Transistors; P1 and P2 for the P-channel MOS Tr.
// mxl = P1 o N1 and mxr = P2 o N2 with x e {1,2,3,4} are connected together to one GPIO output each.
// in idle mode mxl and mxr are high level output connecting the motor via N1 and N2 to GND
// to move the motor to the left mxl gets low; to the right mxr gets low. If both are low motor stops.
// ATTENTION
// all 4 motor bridges are connected to the same U.current line. Thus
// ONLY ONE MOTOR SHALL BE SWITCHED ON AT THE SAME TIME !!!
// while the motor is running the current is permanently monitored via U.current.
// if current is greater than a predefined value the motor is stopped by software.
// usually this indicates that the valve drive has reached an endpoint.

// *  Measured currents and times during motor movement:
// *  --------------------------------------------------
// *  ca. 12 break case.break case 15 mA without load
// *  ca. 17 break case.break case 20 mA pusshing the valve pin into the valve, closing it
// *  > 40 mA arriving at mechanical endpoint -> stop immediately
// *  time from open to closed valve: ca. 30sec
// *  time for the motor running without pushing the valve pin: ca. 8sec
// 

void motor_stop( uint8_t valve ) {
  // valve    e   {1,2,3,4}
  int8_t v = valve - 1;
  int8_t i = v * 2;

  if( (v < 0) || (v > 3) ) return;
  set_pin( motors[i], 1 );      // 1. half-bridge to low output
  set_pin( motors[i+1], 1 );    // 2. half-bridge to low output
  stat.flMotAct[v] = 0;         // motor nolonger active
}


void motor_stop_all( void ) {
  uint8_t valve;

  // set all motor output pins to idle state
  for( valve=0; valve<5; valve++ ) { 
    motor_stop( valve );
  }
}


void motor_on( int8_t motorNr, int8_t dir ) {
  // switch on motor
  // motNr:   motor number e {0,1,2,3,4}
  // dir:     direction e { V_ZU, V_HALT, V_AUF }
  // NOTE if a motor is switched on, all other motors are stopped
  // NOTE motor number 0 stops all motors, also all other numbers not in set above

  // stop all motors 
  motor_stop_all();

  switch( motorNr ) {
    case 1:
      if( dir == V_AUF  ) set_pin( m1l, 0 ); 
      if( dir == V_ZU   ) set_pin( m1r, 0 ); 
      break;
    case 2:
      if( dir == V_AUF  ) set_pin( m2l, 0 ); 
      if( dir == V_ZU   ) set_pin( m2r, 0 ); 
      break;
    case 3:
      if( dir == V_AUF  ) set_pin( m3l, 0 ); 
      if( dir == V_ZU   ) set_pin( m3r, 0 ); 
      break;
    case 4:
      if( dir == V_AUF  ) set_pin( m4l, 0 ); 
      if( dir == V_ZU   ) set_pin( m4r, 0 ); 
      break;
    default:
      // all motors stay off
    break;
  }
  stat.flMotAct[ motorNr - 1] = 1;
}


// start motor during normal control
uint8_t motor_start() {
  // returns the color of the LED to indicate kind of movement
  uint8_t   valve = stat.actValve;
  uint8_t   v     = valve-1;  // index 0.break case.3 for valve 1..4
  uint8_t   dir   = stat.dir[v];  
  float     sec   = stat.dtMot[v];
  uint8_t   color;
  uint32_t  msec;
  parameter_t pa = par[v];

  msec = 0;
  if( (valve<1) || (valve>4)       // wrong valve number 
   || ((dir!=V_AUF)&&(dir!=V_ZU))  // wrong direction
   || (stat.errV[v] > 0 )          // some error from measurements
    ) {
    color = MAGENTA;
    sec   = par[v].dtLed;          // display only short time
  }
  else if( pa.active==0 ) {
    color = CYAN;
    sec   = par[v].dtLed;          // display only short time
  }
  else {
    if( sec > pa.dtMMx ) {
      sec = pa.dtMMx;
    }
    if( (pa.active) && (dir == V_ZU) ) {
      color = BLUE;              // close valve; getting colder
    }
    else if( (pa.active) && (dir == V_AUF) ) {
      color = RED;               // open valve; getting hotter
    }
    else {
      color = WHITE;             // should never get here
    }
  }
  msec = (uint32_t)(sec * 1000.0);            // motor on time
  stat.ticMotStop = tic + msec;

  if( msec > 0 ) {
    // start motor
    motor_on( valve, dir );
    HAL_Delay( 100 );             // wait for decay of motor start current
  }
  return color;
}


// 
uint8_t check_motor_stop( uint8_t v ) {
  // v   is valve index e {0,1,2,3}
  // return 0: motor is running with normal current consumption
  //        1: if motor on time has elapsed
  //        2: if motor current is higher than stop current, else 0
  //        3: if no valve is attached
  parameter_t p   = par[v];

  rr_measure();                    // get all analog values
  // NOTE minimum motor current can not be evaluated, because the motor switches off
  // NOTE it it reaches an end-position. Then the line is open like in a cable break.
  // NOTE this is why we cannot use the following code:
  //  if( stat.imot < p.IMin ) {    // no motor attached -> current too low
  //    stat.errV[v] |= ERR_MOTOR_CBR; // cable break / no motor attached
  //    stat.valvePos[v] = 50.0;       // assume 50%
  //    return 3;
  //  }

  if( (stat.imot > MOTOR_IMAX_STOP) ||   // absolute maximum current
      (stat.imot > p.IStop) ) {
    motor_stop_all();              // switch off motor immediately
    stat.nMotLimit[v]++;
    if( stat.dir[v] == V_ZU ) {
      stat.valvePos[v] =   0.0;
    }
    else {
      stat.valvePos[v] = 100.0;
    }
    return 2;
  }

  if( (int32_t)(tic - stat.ticMotStop) >= 0 ) {
    motor_stop_all();
    // motor on-time expired; set rough value for valve position in percent
    float percent = stat.dtMot[v] / par[v].dtVoc * 100.0;   // 100 (%)
    if( stat.dir[v] == V_ZU ) {
      stat.tMotSum[v]  -= stat.dtMot[v];
      stat.tMotTotal[v]+= stat.dtMot[v];
      stat.valvePos[v] -= percent;
      // limit to 1degC to start motor next time if no limit actually reached
      if( stat.valvePos[v] < 0.0 ) stat.valvePos[v] = 1.0;
    }
    else {
      stat.tMotSum[v]  += stat.dtMot[v];
      stat.tMotTotal[v]+= stat.dtMot[v];
      stat.valvePos[v] += percent;
      // limit to 99degC to start motor next time if no limit actaully reached
      if( stat.valvePos[v] > 100.0 ) stat.valvePos[v] = 99.0;
    }
    return 1;
  }
  else return 0;
}




// *****************************
// calculations
// *****************************

// check if temperature is in a valid range to detect
//   missing sensor ->   >150degC
//   short circuit  ->   <-50degC
uint8_t sensor_check_range( float temp ) {
  // v        valve index {0..3}
  // temp     temperature to be checked
  uint8_t r;

  if( temp>150.0 ) {
    r = T_SENSOR_OPEN;     // not connected, cable break or wrong sensor over-range
  }
  else if( temp<-50.0 ) {
    r = T_SENSOR_SHORT;    // short circuit or wrong sensor under-range
  }
  else r = T_SENSOR_OK;    // sensor value in range
  return r;
}


// determine Temperature of Vorlauf - at least one sensor connected
// 0. if a valid (tvzOk > 0) value tvz from Zentrale is avalilable use tvz 
// 1. if a sensor for Vorlauf (tv) is installed, this value us used
// 2. if no sensor for Vorlauf is installed, take temperature of the first channel with a sensor instaled
// 3. if no channel has a Vorlauf Sensor installed, set error flag of regulator
float controller_values( uint8_t v ) {
  // v  e  {0,1,2,3} valve index
  // return:
  //   -99.9  degC if no valid temperature is measured at any channel
  //   Vorlauf temperature of valve Nr = v+1 if a sensor is connected
  //   Vorlauf temperature of first active valve with a sensor connected
  uint8_t w;
  uint8_t f;
  float tVorlauf;    // measured value or -99.9 or 999.9 for errors

  // *** check if there is a valid Vorlauf temperature from Zentrale
  if( stat.tvzOk > 0.0 ) {
    f = sensor_check_range( stat.tvz );
    if( f != T_SENSOR_OK ) {
      tVorlauf = -99.9;
      stat.tvzOk = 0.0;
    }
    else {
      tVorlauf = stat.tvz;
      stat.tvr[v] = tVorlauf;
    }
  }

  // *** check and define Vorlauf temperature for valve nr = v+1
  if( stat.tvzOk == 0.0 ) {
    f = sensor_check_range( stat.tv[v] );
    stat.sVStat[v] = f;              // set sensor status for Vorlauf
    if( f == T_SENSOR_OK ) {
      tVorlauf    = stat.tv[v];
      stat.tvr[v] = tVorlauf;
    }
    else {
      for( w=0; w<4; w++ ) {
        f = sensor_check_range( stat.tv[w] );
        if( f == T_SENSOR_OK ) {
          tVorlauf = stat.tv[w];     // use the first value with a Vorlauf sensor installed
          stat.tvr[v] = tVorlauf;
          break;
        }
      }
    }
  }

  if( (tVorlauf == -99.9)||(tVorlauf == 999.9) ) {
    stat.errV[v] |= ERR_NO_VALID_TV;
    stat.err     |= ERR_NO_VALID_TV;
    stat.tvr[v] = -99.9;
  }

  // *** check and define Ruecklauf temperature
  f = sensor_check_range( stat.tr[v] );
  stat.sRStat[v] = f;
  if( f == T_SENSOR_OK )  { 
    stat.trr[v] = stat.tr[v];
  }
  else {
    stat.errV[v] |= ERR_NO_VALID_TR;
    stat.trr[v] = -99.9;
  }
  return tVorlauf;
}



// ****************************
// *** Temperature measurements

//   Schematic:   
// *     Vs (3.3V) <=> 4095digits (12bit resolution) <=> 2 * <avs> (adc raw reference)
// *           |
// *          switch  <- set_VTemp
// *           |
// *          <rv>       <Rv = 1200 Ohm/0.1%>
// *           |
// *           +---- ADC value <avt> for temperature
// *           |
// *          <rt>       <R.T = KTY81-110; 1209 Ohm @ 50°C>
// *           |
// *          GND - 0V
// *
// *    for reference one channel has a 1200 Ohm/0.1% resistor in place of R.T: -> avs         
// * (!) Ratiometric measurement
 

typedef struct { float t; float r; } KTY81_RTable_t;
KTY81_RTable_t r2t[] = {
  // degC    Ohm
  {-55.0,  490.0 },
  {-50.0,  515.0 },
  {-40.0,  567.0 },
  {-30.0,  624.0 },
  {-20.0,  684.0 },
  {-10.0,  747.0 },
  {  0.0,  815.0 },
  { 10.0,  886.0 },
  { 20.0,  961.0 },
  { 25.0, 1000.0 },
  { 30.0, 1040.0 },
  { 40.0, 1122.0 },
  { 50.0, 1209.0 },
  { 60.0, 1299.0 },
  { 70.0, 1392.0 },
  { 80.0, 1490.0 },
  { 90.0, 1591.0 },
  {100.0, 1696.0 },
  {110.0, 1805.0 },
  {120.0, 1915.0 },
  {130.0, 2023.0 },
  {140.0, 2124.0 },
  {150.0, 2211.0 }
};

char nr2t = sizeof( r2t ) / 8;


float interpol( float x, float x0, float y0, float x1, float y1 ) {
  //  suche den y-Wert zu x auf der Geraden von (x0,y0) nach (x1,y1)
  float m = (y1 - y0 ) / ( x1 - x0 );
  float y = m * ( x - x0 ) + y0;
  return y;
}


float raw2temp( int32_t avt ) {
  // avt :                                      dig;    adc-value from ADC (12bit)
  // return temp in deg C; underrange = -99.9 (short-); overrange = 999.9 (open-circuit) 
  uint8_t i;
  float rt;                                  // Ohm;    resistance of temperature sensor
  float rv = 1200.0;                         // Ohm;    resistance of upper resistor in voltage-divider
  float avs = 2.0 * adcMeanSum[ adc_tref ];  // dig;    adc value of supply voltage (2*half_voltage) 
  float temp;                                // deg C;  calculated temperature

  rt = rv * (float)avt / (avs-(float)avt);   // Ohm;    resistance of KTY81 Sensor
  if( rt < r2t[0].r ) {
    temp = -99.9;
  }
  else if( rt > r2t[nr2t-1].r ) {
    temp = 999.9;
  }
  else {
    for( i=0; i < nr2t-1; i++ ) {
      if( r2t[i].r >= rt ) 
        break;
    } 
    temp = interpol( rt, r2t[i].r, r2t[i].t, r2t[i+1].r, r2t[i+1].t );
  }
  return temp;
}


// dC := degree Celsius
//                         130 dC - 30 dC
// Temperature (in dC) = ------------------- x (TS_DATA - TS_CAL1) + 30 dC
//                        TS_CAL2 - TS_CAL1
// calibration values for temperature sensors and their addresses
// program from STM32L0x1 reference manual, chapter A.8.17
#define TEMP130_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007A))
#define VREFINT_CAL ((uint16_t*) ((uint32_t) 0x1FF80078))
float ComputeTemperature(float tValue) {
  float t;
//  temperature = (((float)tValue * 330.0 / 300.0) - (float) *TEMP30_CAL_ADDR ); 
//  temperature = temperature * (130.0 - 30.0);
//  temperature = temperature / (int32_t)(*TEMP130_CAL_ADDR - *TEMP30_CAL_ADDR);
//  temperature = temperature + 30.0;
  float tcal1 = (float)(*TEMP30_CAL_ADDR);
  float tcal2 = (float)(*TEMP130_CAL_ADDR);
  float ufakt = (float)(*VREFINT_CAL) / (adcMeanSum[adc_intVRef]);
  
  t = ufakt * tValue;  // NOTE do not use; stimmt was nicht; kommt 35deg statt 25deg raus
  t = (130.0-30.0)/(tcal2-tcal1) * (t - tcal1) + 30.0;
  return t; // in degree Celsius
}

// VDDA is used as reference for AD-conversions
float vdda( uint32_t adcValue ) {
  uint32_t vrefIntCal = *VREFINT_CAL;
  float vdda = 3.0 * (float)vrefIntCal / (float)adcValue;
  return vdda;
}


void calc_raw2phys( void ) {
  float uhelp;

  stat.vdda = vdda( adcMeanSum[ adc_intVRef ] );

  uhelp     = adcMeanSum[ adc_vshalf ] / (2047.5) * stat.vdda;   
  stat.vsup = uhelp;

  uhelp     = adcMeanSum[ adc_imotor ] / (4095.0) * stat.vdda;
  stat.imot = uhelp * 1000.0 / R_SHUNT;
  stat.vmot = stat.vdda - uhelp;

  stat.tv[0]  = raw2temp( adcMeanSum[ adc_tv1 ] );
  stat.tr[0]  = raw2temp( adcMeanSum[ adc_tr1 ] );
  stat.tv[1]  = raw2temp( adcMeanSum[ adc_tv2 ] );
  stat.tr[1]  = raw2temp( adcMeanSum[ adc_tr2 ] );
  stat.tv[2]  = raw2temp( adcMeanSum[ adc_tv3 ] );
  stat.tr[2]  = raw2temp( adcMeanSum[ adc_tr3 ] );
  stat.tv[3]  = raw2temp( adcMeanSum[ adc_tv4 ] );
  stat.tr[3]  = raw2temp( adcMeanSum[ adc_tr4 ] );
  stat.tref   = raw2temp( adcMeanSum[ adc_tref] );
  stat.tempInt = (float)ComputeTemperature( adcMeanSum[adc_intTemp] );
}





// *****************************
// global functions
// *****************************



void rr_init( void ) {
  set_pins2default();
  init_variables();
  set_led( 0, 0 );
  get_address();
  adc_init();
  ser1_init();
}


void rr_selftest( void ) {
  test_led();
}



void rr_measure( void ) {
  uint8_t i, j, v;
  int32_t sum[ADC_CHANNEL_CNT];

  set_VTemp( 1 );                // switch on voltage for temperature sensors
  HAL_Delay( 10 );               // wait for voltages getting stable
  for( j=0; j<ADC_CHANNEL_CNT; j++ ) {
    sum[j] = 0L;
  }
  for( i=0; i<ADC_MEAN_CNT; i++ ) {
    adc_readAllDMA();
    for( j=0; j<ADC_CHANNEL_CNT; j++ ) {
      sum[j] += adc_raw[j];
    }
  }
  for( j=0; j<ADC_CHANNEL_CNT; j++ ) {
    adcMeanSum[j] = (float)sum[j] / (float)ADC_MEAN_CNT;
  }
  // NOTE  do not switch off; easier to measure currents in field
  //set_VTemp( 0 );                // switch off voltage for sensors
  calc_raw2phys();
  for( v=0; v<4; v++ ) {
    controller_values( v );      // prepare values for controllers
  }
}


// *** check if button is pressed

uint8_t keypressed( void ) {
  uint16_t key;
  key = HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_11 );
  if( key ) {
    key = 0;       
  }
  else {
    key = 1;         // button pressed
    HAL_Delay( 5 );  // debounce
  }
  stat.key = (uint8_t)key;
  return (uint8_t)key;
}


uint8_t keychange( void ) {
  uint8_t keyold = stat.key;
  uint8_t key = keypressed();
  if( keyold==0 && key==1 ) return 1;                   // key pressed
  if( keyold==1 && key==0 ) return 2;                   // key released
  return 0;                                             // key not changed
}

