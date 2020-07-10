<<<<<<< HEAD
// rr.h

// Define to prevent recursive inclusion -------------------------------------
#ifndef __RR_H
#define __RR_H

#ifdef __cplusplus
 extern "C" {
#endif

// *******************************
// ATTENTION CHANGE HERE FOR TESTS
// *******************************

#define TEST_FAST 0





// c
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
// stm32 hal (hardware abstraction layer) system; used by "CubeMX" generator
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_adc.h"
#include "usart.h"
#include "mxconstants.h"
// from application
#include "rr_autogene.h"
#include "rr_ser.h"

#define FW_NAME       "HZ-RR"   // project name
#define FW_REV        "11d"     // revision of firmware
#define PROT_REV      "a"       // revision of protocol in case of later changes


#define EVER ;;

// ERRORS
#define ERR_NO_VALID_TV     0x0001     // no sensor for "T-Vorlauf" in all 4 valves
#define ERR_NO_VALID_TR     0x0002     // no sensor for "T-Ruecklauf" for actual valve
#define ERR_EEPROM_READ     0x0004     // cannot read eeprom data after writing default values
#define ERR_EEPROM_UNLOCK   0x0008     // cannot unlock eeprom before write
#define ERR_EEPROM_WRITE    0x0010     // cannot write data to eeprom
#define ERR_NO_VALID_MOT    0x0020     // direction or motor on time are wrong
#define ERR_MODBUS_RX       0x0040     // modbus packet was wrong
#define ERR_MODBUS_CON      0x0080     // modbus module/regulator=controller number out of range
#define ERR_MOTOR_CBR       0x0100     // motor cable break, Motor nicht angeschlossen, Strom zu gering

// eeprom data related
#define KENNUNG         FW_NAME FW_REV // NOTE max. 9 char + 0, no blank; Kennung der Datensaetze incl. Version
#define EEPROM_PAR_SIZE      256       // number of bytes reserved for one parameter set
// adc and process related
#define U_SUPPLY          3.3          // volts; may differ in reality; no problem - ratiometric values
#define R_SHUNT           3.3          // Ohm;   shunt resistor to measure current of motor
#define ADC_MEAN_CNT      8            // number of repeated adc-cycles for mean value

#define T_LED_MIN  500            // msec for minimum LED display time

#define MOTOR_IMAX_STOP    60.0   // mA;   absolute maximum motor current to stop if limit is reached
#define   V_HALT            0     // Motor bleibt stehen
#define   V_AUF             1     // Motor faehrt zurueck -> Stift eingezogen
#define   V_ZU              2     // Motor und Stift faehrt aus
#define MOTOR_RANGE_TIME 30.0     // sec;  time for positioning from end-to-end
#define T_INSTALL       300.0     // sec;  time to keep active valves open for installation
// bits for motor-control flags
#define MOTOR1  0x01
#define MOTOR2  0x02
#define MOTOR3  0x04
#define MOTOR4  0x08
// sensor status
#define T_SENSOR_OK       0
#define T_SENSOR_SHORT    1
#define T_SENSOR_OPEN     2
// conversions
#define DAYS2MSEC         (24.0 * 3600000.0)


// **************************
// parameter_t structure 
// is in file "rr_autogene.h"


// status information for whole process
typedef struct {
  // *** common values:
  // general
  uint32_t    tic;           // timer tick of last main loop = actual time for status
  uint32_t    ticWait;       // timer ticks; end time of a wait condition somewhere
  // error conditions
  uint32_t    err;           // global errors
  uint16_t    errSer1;       //  >0 if an error callback on serial1 was invoked
  // *** flags:
  // serial communication
  uint8_t     adcConvCplt;
  uint8_t     tx1Busy;       //  1 if serial transmission is in progress
  uint8_t     rx1Running;    //  1 if serial IT reception is active, if 0 must restart IT reception 
  uint8_t     rx1Eol;        //  1 if serial <cr> or <lf> was received
  uint8_t     cmdParReady;   //  >0 : a command was received and waits for execution
  uint8_t     parReset;      //  >0 : set all parameters to factory settings
  uint8_t     par2eeprom;    //  1 : write all parameters to eeprom
  uint8_t     modbusSent;    //  >0 count for number of tries to send data via modbus; 0 if ready for new data to send
  uint8_t     modubsReceived;//  1 a command was received via RS-485 modbus
  uint16_t    adr;          // jumper set address: 0x0 .. 0x1F for RS485 slave; 0xFF: no init
  uint8_t     txStrReady;   // send-string is ready to be sent; will be sent if communication is ready

  uint8_t     actValve;      //  actual valve Number used for control
  uint8_t     key;           //  1 if key is pressed
  // analog values
  float       vsup;         // V; supply voltage after protection Schottky diode from supply voltage
  float       tref;         // degC; reference temperature of 1/2 VCC of sensor supply (typ. 3.3V / 2)
  float       vmot;         // V; voltage on motor = vsup - volts(VMhalf)
  float       imot;         // mA; voltage on Rsense / Rsense
  float       vdda;         // VDDA calculated from internal ref. voltage
  float       tempInt;      // internal temperature
  float       tvz;          // deg C; temp. Vorlauf; von Zentrale gesendet
  float       tvzOk    ;    // sec;   Endzeit ab wann tvz nicht mehr gilt
  // *** flags for motor and led control
  // motor state control for movements 
  uint8_t     motControl;    //  0:inactive; else state counter for control cycle
  uint8_t     motInstall;    //  0:inactive; else state counter for open valve for installation
  uint8_t     motMove;       //  0:inactive; else state counter for moving valves in summer modee
  uint8_t     led;           //  0:inactive; else state of led indicator
  uint8_t     ledColor;      //  color of led to blink / show state
  uint8_t     motPos;        //  0:inactive; else state of direct motor movement
  uint8_t     flMotLock;     //  block other requests; motor nr. e {1,2,3,4} if a motor is active else 0;
  // motor and led end-times
  uint32_t    ticMotStop;    // msec; tic count when motor has to be switched off
  uint32_t    ticLedOff;     // msec; tic count when led has to be switched off
  // *** for each valve indexed values
  uint8_t     summer[4];     // 1: if summer, 0: winter operation
  uint8_t     vMoveSummer[4]; // 1: valve movement during summer period has to be performed
  uint32_t    ticVSummer[4]; // msec; tic count when next valve movement has to be applied in summer mode
  uint16_t    errV[4];       // error values for each valve

  uint32_t    ticIntNew[4];  // msec;  tic count when a new interval starts
  uint8_t     flLedOn[4];    // 1      if LED is on; else 0

  uint8_t     flMotRequ[4];  // 1      if motor has to be switched on
  uint8_t     flMotAct[4];   // 1      if motor of current controller is activated

  int8_t      dir[4];        //        direction to drive motor e {V_HALT, V_AUF, V_ZU}
  float       dtMot[4];      // sec;   time to drive motor
  uint8_t     color[4];      //        color of led to indicate status
  float       tMotSum[4];    // sec;   sum of motor runtime; set to 0 each time the valve raches an end
  float       tMotTotal[4];  // sec;   total time of all motor movements
  uint32_t    nMotLimit[4];  // 1;     count nr. of limits reached by valve
  float       tv[4];         // deg C; temperature Vorlauf, gemessen
  float       tvr[4];        // deg C; temperature Vorlauf, used for control
  uint8_t     sVStat[4];     // flag;  sensor Vorlauf status: {0,1,2} = {in range, not connected, short circuit}
  float       tr[4];         // deg C; temperature Ruecklauf, gemessen
  float       trr[4];        // deg C; temperature Ruecklauf, used for control
  uint8_t     sRStat[4];     // flag;  sensor Ruecklauf status: {0,1,2} = {in range, not connected, short circuit}
  float       trSoll[4];     // deg C; set temperature, Ruecklauf
  uint8_t     cmdVLimit[4];  // 1   e {V_HALT, V_AUF, V_ZU}; drive valve to limit 
  uint8_t     fix[4];        // flag;  valve is in fixed position
  // motor - valve status
    // valve position for motor 1,2,3,4 
    //   0.0 : valve closed; pin extracted to maximum current
    // 100.0 : valve opened; pin retracted to max. current; installation pos.
    //  -1.0 : not yet initialized
  float       valvePos[4];
} status_t;


// for serial communication / modbus
typedef struct {
  uint8_t        adr;         // address in command string (NOT jumper set address ! )
  uint8_t        nr;          // command number
  uint8_t        con;         // controller nr. 1,2,3,4; 0 is module
} command_t;


// enum GPIO pins:
enum { m1l,   m1r,  m2l,  m2r, m3l,   m3r,  m4l,   m4r, 
       led1, led2, led3, led4, red, green, blue, de485, vtemp };
enum { mot1 = 1, mot2, mot3, mot4 };

extern parameter_t        par[]; // all parameters determining function; stored in eeprom
extern parameter_t pl[];
extern parameter_t parx;

extern volatile status_t  stat;  // all variables defining the status of the regulators 
extern command_t          com;

extern uint32_t  tic;
extern float     ticSec;
extern uint8_t   motors[];
extern uint8_t   leds[];
extern uint8_t    colors[];

void rr( void );



#ifdef __cplusplus
}
#endif

#endif /* __RR_H */
=======
// rr.h

// Define to prevent recursive inclusion -------------------------------------
#ifndef __RR_H
#define __RR_H

#ifdef __cplusplus
 extern "C" {
#endif

// *******************************
// ATTENTION CHANGE HERE FOR TESTS
// *******************************

#define TEST_FAST 0





// c
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
// stm32 hal (hardware abstraction layer) system; used by "CubeMX" generator
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_adc.h"
#include "usart.h"
#include "mxconstants.h"
// from application
#include "rr_autogene.h"
#include "rr_ser.h"

#define FW_NAME       "HZ-RR"   // project name
#define FW_REV        "11d"     // revision of firmware
#define PROT_REV      "a"       // revision of protocol in case of later changes


#define EVER ;;

// ERRORS
#define ERR_NO_VALID_TV     0x0001     // no sensor for "T-Vorlauf" in all 4 valves
#define ERR_NO_VALID_TR     0x0002     // no sensor for "T-Ruecklauf" for actual valve
#define ERR_EEPROM_READ     0x0004     // cannot read eeprom data after writing default values
#define ERR_EEPROM_UNLOCK   0x0008     // cannot unlock eeprom before write
#define ERR_EEPROM_WRITE    0x0010     // cannot write data to eeprom
#define ERR_NO_VALID_MOT    0x0020     // direction or motor on time are wrong
#define ERR_MODBUS_RX       0x0040     // modbus packet was wrong
#define ERR_MODBUS_CON      0x0080     // modbus module/regulator=controller number out of range
#define ERR_MOTOR_CBR       0x0100     // motor cable break, Motor nicht angeschlossen, Strom zu gering

// eeprom data related
#define KENNUNG         FW_NAME FW_REV // NOTE max. 9 char + 0, no blank; Kennung der Datensaetze incl. Version
#define EEPROM_PAR_SIZE      256       // number of bytes reserved for one parameter set
// adc and process related
#define U_SUPPLY          3.3          // volts; may differ in reality; no problem - ratiometric values
#define R_SHUNT           3.3          // Ohm;   shunt resistor to measure current of motor
#define ADC_MEAN_CNT      8            // number of repeated adc-cycles for mean value

#define T_LED_MIN  500            // msec for minimum LED display time

#define MOTOR_IMAX_STOP    60.0   // mA;   absolute maximum motor current to stop if limit is reached
#define   V_HALT            0     // Motor bleibt stehen
#define   V_AUF             1     // Motor faehrt zurueck -> Stift eingezogen
#define   V_ZU              2     // Motor und Stift faehrt aus
#define MOTOR_RANGE_TIME 30.0     // sec;  time for positioning from end-to-end
#define T_INSTALL       300.0     // sec;  time to keep active valves open for installation
// bits for motor-control flags
#define MOTOR1  0x01
#define MOTOR2  0x02
#define MOTOR3  0x04
#define MOTOR4  0x08
// sensor status
#define T_SENSOR_OK       0
#define T_SENSOR_SHORT    1
#define T_SENSOR_OPEN     2
// conversions
#define DAYS2MSEC         (24.0 * 3600000.0)


// **************************
// parameter_t structure 
// is in file "rr_autogene.h"


// status information for whole process
typedef struct {
  // *** common values:
  // general
  uint32_t    tic;           // timer tick of last main loop = actual time for status
  uint32_t    ticWait;       // timer ticks; end time of a wait condition somewhere
  // error conditions
  uint32_t    err;           // global errors
  uint16_t    errSer1;       //  >0 if an error callback on serial1 was invoked
  // *** flags:
  // serial communication
  uint8_t     adcConvCplt;
  uint8_t     tx1Busy;       //  1 if serial transmission is in progress
  uint8_t     rx1Running;    //  1 if serial IT reception is active, if 0 must restart IT reception 
  uint8_t     rx1Eol;        //  1 if serial <cr> or <lf> was received
  uint8_t     cmdParReady;   //  >0 : a command was received and waits for execution
  uint8_t     parReset;      //  >0 : set all parameters to factory settings
  uint8_t     par2eeprom;    //  1 : write all parameters to eeprom
  uint8_t     modbusSent;    //  >0 count for number of tries to send data via modbus; 0 if ready for new data to send
  uint8_t     modubsReceived;//  1 a command was received via RS-485 modbus
  uint16_t    adr;          // jumper set address: 0x0 .. 0x1F for RS485 slave; 0xFF: no init
  uint8_t     txStrReady;   // send-string is ready to be sent; will be sent if communication is ready

  uint8_t     actValve;      //  actual valve Number used for control
  uint8_t     key;           //  1 if key is pressed
  // analog values
  float       vsup;         // V; supply voltage after protection Schottky diode from supply voltage
  float       tref;         // degC; reference temperature of 1/2 VCC of sensor supply (typ. 3.3V / 2)
  float       vmot;         // V; voltage on motor = vsup - volts(VMhalf)
  float       imot;         // mA; voltage on Rsense / Rsense
  float       vdda;         // VDDA calculated from internal ref. voltage
  float       tempInt;      // internal temperature
  float       tvz;          // deg C; temp. Vorlauf; von Zentrale gesendet
  float       tvzOk    ;    // sec;   Endzeit ab wann tvz nicht mehr gilt
  // *** flags for motor and led control
  // motor state control for movements 
  uint8_t     motControl;    //  0:inactive; else state counter for control cycle
  uint8_t     motInstall;    //  0:inactive; else state counter for open valve for installation
  uint8_t     motMove;       //  0:inactive; else state counter for moving valves in summer modee
  uint8_t     led;           //  0:inactive; else state of led indicator
  uint8_t     ledColor;      //  color of led to blink / show state
  uint8_t     motPos;        //  0:inactive; else state of direct motor movement
  uint8_t     flMotLock;     //  block other requests; motor nr. e {1,2,3,4} if a motor is active else 0;
  // motor and led end-times
  uint32_t    ticMotStop;    // msec; tic count when motor has to be switched off
  uint32_t    ticLedOff;     // msec; tic count when led has to be switched off
  // *** for each valve indexed values
  uint8_t     summer[4];     // 1: if summer, 0: winter operation
  uint8_t     vMoveSummer[4]; // 1: valve movement during summer period has to be performed
  uint32_t    ticVSummer[4]; // msec; tic count when next valve movement has to be applied in summer mode
  uint16_t    errV[4];       // error values for each valve

  uint32_t    ticIntNew[4];  // msec;  tic count when a new interval starts
  uint8_t     flLedOn[4];    // 1      if LED is on; else 0

  uint8_t     flMotRequ[4];  // 1      if motor has to be switched on
  uint8_t     flMotAct[4];   // 1      if motor of current controller is activated

  int8_t      dir[4];        //        direction to drive motor e {V_HALT, V_AUF, V_ZU}
  float       dtMot[4];      // sec;   time to drive motor
  uint8_t     color[4];      //        color of led to indicate status
  float       tMotSum[4];    // sec;   sum of motor runtime; set to 0 each time the valve raches an end
  float       tMotTotal[4];  // sec;   total time of all motor movements
  uint32_t    nMotLimit[4];  // 1;     count nr. of limits reached by valve
  float       tv[4];         // deg C; temperature Vorlauf, gemessen
  float       tvr[4];        // deg C; temperature Vorlauf, used for control
  uint8_t     sVStat[4];     // flag;  sensor Vorlauf status: {0,1,2} = {in range, not connected, short circuit}
  float       tr[4];         // deg C; temperature Ruecklauf, gemessen
  float       trr[4];        // deg C; temperature Ruecklauf, used for control
  uint8_t     sRStat[4];     // flag;  sensor Ruecklauf status: {0,1,2} = {in range, not connected, short circuit}
  float       trSoll[4];     // deg C; set temperature, Ruecklauf
  uint8_t     cmdVLimit[4];  // 1   e {V_HALT, V_AUF, V_ZU}; drive valve to limit 
  uint8_t     fix[4];        // flag;  valve is in fixed position
  // motor - valve status
    // valve position for motor 1,2,3,4 
    //   0.0 : valve closed; pin extracted to maximum current
    // 100.0 : valve opened; pin retracted to max. current; installation pos.
    //  -1.0 : not yet initialized
  float       valvePos[4];
} status_t;


// for serial communication / modbus
typedef struct {
  uint8_t        adr;         // address in command string (NOT jumper set address ! )
  uint8_t        nr;          // command number
  uint8_t        con;         // controller nr. 1,2,3,4; 0 is module
} command_t;


// enum GPIO pins:
enum { m1l,   m1r,  m2l,  m2r, m3l,   m3r,  m4l,   m4r, 
       led1, led2, led3, led4, red, green, blue, de485, vtemp };
enum { mot1 = 1, mot2, mot3, mot4 };

extern parameter_t        par[]; // all parameters determining function; stored in eeprom
extern parameter_t pl[];
extern parameter_t parx;

extern volatile status_t  stat;  // all variables defining the status of the regulators 
extern command_t          com;

extern uint32_t  tic;
extern float     ticSec;
extern uint8_t   motors[];
extern uint8_t   leds[];
extern uint8_t    colors[];

void rr( void );



#ifdef __cplusplus
}
#endif

#endif /* __RR_H */
>>>>>>> fdb118d65f9ba2384457aa1ceb71ee94c0edf102
