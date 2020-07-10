<<<<<<< HEAD
// rr.h

// Define to prevent recursive inclusion -------------------------------------
#ifndef __RR_H
#define __RR_H

#ifdef __cplusplus
 extern "C" {
#endif

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
// from our application
#include "rr_io.h"
#include "rr_ser.h"
#include "rr_ctrl.h"
#include "rr_test.h"

#define FW_NAME       "HZ-RR"
#define FW_REV        "1.1"
#define PROT_REV      "a"       // revision of protocol in case of later changes

#define TEST_FAST   1
#define TEST_PL     1     // TODO only for debugging - remove later


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

// COMMANDS
#define CMD_NULL            0x00       // no command pending
#define CMD_SET_PARAM1      0x01       // parameters received to replace current parameters of valve 1
#define CMD_SET_PARAM2      0x02       // parameters received to replace current parameters of valve 2
#define CMD_SET_PARAM3      0x03       // parameters received to replace current parameters of valve 3
#define CMD_SET_PARAM4      0x04       // parameters received to replace current parameters of valve 4
#define CMD_PROG_EEPROM     0x05       // save current parameter set to EEPROM


// eeprom data related
#define KENNUNG    "HZ-RR.011"          // NOTE max. 9 char + 0, no blank; Kennung der Datensaetze incl. Version

// adc and process related
#define U_SUPPLY          3.3          // volts; may differ in reality; no problem - ratiometric values
#define R_SHUNT           3.3          // Ohm;   shunt resistor to measure current of motor
#define ADC_MEAN_CNT      8            // number of repeated adc-cycles for mean value

#define T_LED_MIN  500     // msec for minimum LED display time

#define MOTOR_IMAX_STOP    50.0   // mA;   absolute maximum motor current to stop if limit is reached
#define MOTOR_IMIN          2.0   // mA;   assume cable break if below
#define MOTOR_MAX_TIME     50.0   // sec;  maximum motor on time
#define MOTOR_MIN_TIME      0.5   // sec;  minimal motor on time
#define   V_HALT            0     // Motor bleibt stehen
#define   V_AUF             1     // Motor faehrt zurueck -> Stift eingezogen
#define   V_ZU              2     // Motor und Stift faehrt aus
#define MOTOR_RANGE_TIME 30.0     // sec;  timefor positioning from end-to-end
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
#define DAYS2MSEC (24.0 * 3600000.0)


// parameters will be stored in eeprom
typedef struct {
  char        kenn[10];     // Kennung; always in the first place
  uint8_t     active;       // 1: regulator is active / used; 0: not used
  // Betriebsart
  float       tvs;          // degC; Vorlauf, switch to summer if below
  float       tvw;          // degC; Vorlauf, switch to winter if above
  float       dts;          // sec;  Intervall fuer Regelzyklus Sommerbetrieb
  float       dtw;          // sec;  Intervall fuer Regelzyklus Winterbetrieb
  float       dtv;          // tage; Ruhezeit nach der eine Ventilbewegung ausgeführt wird
  float       dtvc;         // sec;  time to close valve from open position for summer operation
  // Kennlinie
  float       tv0;          // degC; x-Achse 1. Punkt der Kennlinie
  float       tr0;          // degC; y-Achse 1. Punkt der Kennlinie
  float       tv1;          // degC; x-Achse 2. Punkt der Kennlinie
  float       tr1;          // degC; y-Achse 2. Punkt der Kennlinie
  // LED Anzeige
  float       dtLed;        // sec;  Einschaltdauer für LEDs
  // Regler
  float       ttol;         // degC; Toleranz; unter +/-ttol wird nicht geregelt
  float       pfakt;        // proportionalfaktor fuer Regelabweichung
  float       aufZuFakt;    // V_AUF geht um diesen Faktor schneller als V_ZU; bei 1.0 gleich
  float       dtMotMin;     // minimale Motorlaufzeit fuer Regeln
  float       dtMotMax;     // maximale Motorlaufzeit fuer zum auf Anschlag fahren
  float       motIStop;     // stop motor if current exceeds this value; limited to MOTOR_I_STOP
  // Ventil
  float       dtValveTravel;// time for a complete valve travel from open to closed or vice versa
  // testbetrieb
  uint8_t     testsActive;  // 0: alle Tests sind deaktiviert
  uint8_t     zeitraffer;   // anz. schnellerer Zyklen f. Test; zaehlt auf 0 runter
  uint8_t     testMode;     // sets test mode; if it has changed-> set to standard and write eeprom
  uint32_t    test;         // used temporarily for debugging
  uint8_t     testFast;     // shorter delay times
  // letzter Eintrag ist die Checksum
  uint32_t    checksum;
} parameter_t;
extern parameter_t par[4];     // all parameters determining function; stored in eeprom


typedef struct {
  // *** common values:
  // general
  uint32_t    tic;           // timer tick of last main loop = actual time for status
  // error conditions
  uint32_t    err;           // global errors
  uint16_t    errSer1;       //  >0 if an error callback on serial1 was invoked
  // *** flags:
  // serial communication
  uint8_t     adcConvCplt;
  uint8_t     tx1Busy;       //  1 if serial transmission is in progress
  uint8_t     rx1Running;    //  1 if serial IT reception is active, if 0 must restart IT reception 
  uint8_t     rx1Eol;        //  1 if serial <cr> or <lf> was received
  uint8_t     cmdReady;      //  >0 : a command was received and waits for execution
  uint8_t     modbusSent;    //  >0 count for number of tries to send data via modbus; 0 if ready for new data to send
  uint8_t     modubsReceived;//  1 a command was received via RS-485 modbus
  uint16_t    adr;          // jumper set address: 0x0 .. 0x1F for RS485 slave; 0xFF: no init
  uint8_t     txStrReady;   // send-string is ready to be sent; will be sent if communication is ready

  uint8_t     actValve;      //  actual valve Number used for control
  uint8_t     key;           //  1 if key is pressed
  // analog values
  float       vsup;         // V; supply voltage after protection Schottky diode from supply voltage
  float       tref;
  float       vmot;         // V; voltage on motor = vsup - volts(VMhalf)
  float       imot;         // mA; voltage on Rsense / Rsense
  float       vdda;         // VDDA calculated from internal ref. voltage
  float       tempInt;      // internal temperature
  // *** flags for motor and led control
  // motor state control
  uint8_t     motControl;    //  0:inactive; else state counter for control cycle
  uint8_t     motInstal;     //  0:inactive; else state counter for open valve for installation
  uint8_t     motMove;       //  0:inactive; else state counter for moving valves in summer modee
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

  int8_t      dir[4];        //        direction to drive e {-1, 0, +1}
  float       dtMot[4];      // sec;   time to drive motor for regulation
  uint8_t     color[4];      //        color of led to indicate status
  float       tMotRun[4];    // sec;   sum of motor runtime; set to 0 each time the valve raches an end

  float       tv[4];        // deg C; temperature Vorlauf, gemessen
  float       tvr[4];       // deg C; temperature Vorlauf, used for control
  uint8_t     sVStat[4];     // sensor Vorlauf status: {0,1,2} = {in range, not connected, short circuit}
  float       tr[4];        // deg C; temperature Ruecklauf, gemessen
  float       trr[4];       // deg C; temperature Ruecklauf, used for control
  uint8_t     sRStat[4];    // sensor Ruecklauf status: {0,1,2} = {in range, not connected, short circuit}
  float       trSoll[4];    // deg C; set temperature, Ruecklauf
  // motor - valve status
    // valve position for motor 1,2,3,4 
    //   0.0 : valve closed; pin extracted to maximum current
    // 100.0 : valve opened; pin retracted to max. current; installation pos.
    //  -1.0 : not yet initialized
  float       valvePos[4];
  // TODO   remove  for test only
  uint32_t    msg_tic;      // msec;   tics until next test message is sent
} status_t;
extern volatile status_t stat; // all variables defining the status of the regulators 


typedef struct {
  uint8_t        adr;         // address in command string (NOT jumper set address ! )
  uint8_t        nr;          // command number
  uint8_t        con;         // controller nr. 1,2,3,4; 0 is module
} command_t;


// enum GPIO pins:
enum { m1l,   m1r,  m2l,  m2r, m3l,   m3r,  m4l,   m4r, 
       led1, led2, led3, led4, red, green, blue, de485, vtemp };
enum { mot1 = 1, mot2, mot3, mot4 };

extern uint32_t tic;
extern uint8_t motors[];
extern uint8_t leds[];
extern uint8_t colors[];
extern command_t com;

void rr( void );
uint16_t command_interpreter( void );



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
// from our application
#include "rr_io.h"
#include "rr_ser.h"
#include "rr_ctrl.h"
#include "rr_test.h"

#define FW_NAME       "HZ-RR"
#define FW_REV        "1.1"
#define PROT_REV      "a"       // revision of protocol in case of later changes

#define TEST_FAST   1
#define TEST_PL     1     // TODO only for debugging - remove later


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

// COMMANDS
#define CMD_NULL            0x00       // no command pending
#define CMD_SET_PARAM1      0x01       // parameters received to replace current parameters of valve 1
#define CMD_SET_PARAM2      0x02       // parameters received to replace current parameters of valve 2
#define CMD_SET_PARAM3      0x03       // parameters received to replace current parameters of valve 3
#define CMD_SET_PARAM4      0x04       // parameters received to replace current parameters of valve 4
#define CMD_PROG_EEPROM     0x05       // save current parameter set to EEPROM


// eeprom data related
#define KENNUNG    "HZ-RR.011"          // NOTE max. 9 char + 0, no blank; Kennung der Datensaetze incl. Version

// adc and process related
#define U_SUPPLY          3.3          // volts; may differ in reality; no problem - ratiometric values
#define R_SHUNT           3.3          // Ohm;   shunt resistor to measure current of motor
#define ADC_MEAN_CNT      8            // number of repeated adc-cycles for mean value

#define T_LED_MIN  500     // msec for minimum LED display time

#define MOTOR_IMAX_STOP    50.0   // mA;   absolute maximum motor current to stop if limit is reached
#define MOTOR_IMIN          2.0   // mA;   assume cable break if below
#define MOTOR_MAX_TIME     50.0   // sec;  maximum motor on time
#define MOTOR_MIN_TIME      0.5   // sec;  minimal motor on time
#define   V_HALT            0     // Motor bleibt stehen
#define   V_AUF             1     // Motor faehrt zurueck -> Stift eingezogen
#define   V_ZU              2     // Motor und Stift faehrt aus
#define MOTOR_RANGE_TIME 30.0     // sec;  timefor positioning from end-to-end
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
#define DAYS2MSEC (24.0 * 3600000.0)


// parameters will be stored in eeprom
typedef struct {
  char        kenn[10];     // Kennung; always in the first place
  uint8_t     active;       // 1: regulator is active / used; 0: not used
  // Betriebsart
  float       tvs;          // degC; Vorlauf, switch to summer if below
  float       tvw;          // degC; Vorlauf, switch to winter if above
  float       dts;          // sec;  Intervall fuer Regelzyklus Sommerbetrieb
  float       dtw;          // sec;  Intervall fuer Regelzyklus Winterbetrieb
  float       dtv;          // tage; Ruhezeit nach der eine Ventilbewegung ausgeführt wird
  float       dtvc;         // sec;  time to close valve from open position for summer operation
  // Kennlinie
  float       tv0;          // degC; x-Achse 1. Punkt der Kennlinie
  float       tr0;          // degC; y-Achse 1. Punkt der Kennlinie
  float       tv1;          // degC; x-Achse 2. Punkt der Kennlinie
  float       tr1;          // degC; y-Achse 2. Punkt der Kennlinie
  // LED Anzeige
  float       dtLed;        // sec;  Einschaltdauer für LEDs
  // Regler
  float       ttol;         // degC; Toleranz; unter +/-ttol wird nicht geregelt
  float       pfakt;        // proportionalfaktor fuer Regelabweichung
  float       aufZuFakt;    // V_AUF geht um diesen Faktor schneller als V_ZU; bei 1.0 gleich
  float       dtMotMin;     // minimale Motorlaufzeit fuer Regeln
  float       dtMotMax;     // maximale Motorlaufzeit fuer zum auf Anschlag fahren
  float       motIStop;     // stop motor if current exceeds this value; limited to MOTOR_I_STOP
  // Ventil
  float       dtValveTravel;// time for a complete valve travel from open to closed or vice versa
  // testbetrieb
  uint8_t     testsActive;  // 0: alle Tests sind deaktiviert
  uint8_t     zeitraffer;   // anz. schnellerer Zyklen f. Test; zaehlt auf 0 runter
  uint8_t     testMode;     // sets test mode; if it has changed-> set to standard and write eeprom
  uint32_t    test;         // used temporarily for debugging
  uint8_t     testFast;     // shorter delay times
  // letzter Eintrag ist die Checksum
  uint32_t    checksum;
} parameter_t;
extern parameter_t par[4];     // all parameters determining function; stored in eeprom


typedef struct {
  // *** common values:
  // general
  uint32_t    tic;           // timer tick of last main loop = actual time for status
  // error conditions
  uint32_t    err;           // global errors
  uint16_t    errSer1;       //  >0 if an error callback on serial1 was invoked
  // *** flags:
  // serial communication
  uint8_t     adcConvCplt;
  uint8_t     tx1Busy;       //  1 if serial transmission is in progress
  uint8_t     rx1Running;    //  1 if serial IT reception is active, if 0 must restart IT reception 
  uint8_t     rx1Eol;        //  1 if serial <cr> or <lf> was received
  uint8_t     cmdReady;      //  >0 : a command was received and waits for execution
  uint8_t     modbusSent;    //  >0 count for number of tries to send data via modbus; 0 if ready for new data to send
  uint8_t     modubsReceived;//  1 a command was received via RS-485 modbus
  uint16_t    adr;          // jumper set address: 0x0 .. 0x1F for RS485 slave; 0xFF: no init
  uint8_t     txStrReady;   // send-string is ready to be sent; will be sent if communication is ready

  uint8_t     actValve;      //  actual valve Number used for control
  uint8_t     key;           //  1 if key is pressed
  // analog values
  float       vsup;         // V; supply voltage after protection Schottky diode from supply voltage
  float       tref;
  float       vmot;         // V; voltage on motor = vsup - volts(VMhalf)
  float       imot;         // mA; voltage on Rsense / Rsense
  float       vdda;         // VDDA calculated from internal ref. voltage
  float       tempInt;      // internal temperature
  // *** flags for motor and led control
  // motor state control
  uint8_t     motControl;    //  0:inactive; else state counter for control cycle
  uint8_t     motInstal;     //  0:inactive; else state counter for open valve for installation
  uint8_t     motMove;       //  0:inactive; else state counter for moving valves in summer modee
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

  int8_t      dir[4];        //        direction to drive e {-1, 0, +1}
  float       dtMot[4];      // sec;   time to drive motor for regulation
  uint8_t     color[4];      //        color of led to indicate status
  float       tMotRun[4];    // sec;   sum of motor runtime; set to 0 each time the valve raches an end

  float       tv[4];        // deg C; temperature Vorlauf, gemessen
  float       tvr[4];       // deg C; temperature Vorlauf, used for control
  uint8_t     sVStat[4];     // sensor Vorlauf status: {0,1,2} = {in range, not connected, short circuit}
  float       tr[4];        // deg C; temperature Ruecklauf, gemessen
  float       trr[4];       // deg C; temperature Ruecklauf, used for control
  uint8_t     sRStat[4];    // sensor Ruecklauf status: {0,1,2} = {in range, not connected, short circuit}
  float       trSoll[4];    // deg C; set temperature, Ruecklauf
  // motor - valve status
    // valve position for motor 1,2,3,4 
    //   0.0 : valve closed; pin extracted to maximum current
    // 100.0 : valve opened; pin retracted to max. current; installation pos.
    //  -1.0 : not yet initialized
  float       valvePos[4];
  // TODO   remove  for test only
  uint32_t    msg_tic;      // msec;   tics until next test message is sent
} status_t;
extern volatile status_t stat; // all variables defining the status of the regulators 


typedef struct {
  uint8_t        adr;         // address in command string (NOT jumper set address ! )
  uint8_t        nr;          // command number
  uint8_t        con;         // controller nr. 1,2,3,4; 0 is module
} command_t;


// enum GPIO pins:
enum { m1l,   m1r,  m2l,  m2r, m3l,   m3r,  m4l,   m4r, 
       led1, led2, led3, led4, red, green, blue, de485, vtemp };
enum { mot1 = 1, mot2, mot3, mot4 };

extern uint32_t tic;
extern uint8_t motors[];
extern uint8_t leds[];
extern uint8_t colors[];
extern command_t com;

void rr( void );
uint16_t command_interpreter( void );



#ifdef __cplusplus
}
#endif

#endif /* __RR_H */
>>>>>>> fdb118d65f9ba2384457aa1ceb71ee94c0edf102
