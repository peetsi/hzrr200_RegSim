/*
  rr_autogene.h
  Project:   HZ-RR
  AUTOMATICALLY GENERATED CODE FROM PYTHON SCRIPT(s):
      "rr_autogene.py"
  */
// NOTE  !!! DO NOT CHANGE HERE !!! CHANGE IN PYTHON SCRIPT !!!



// Define to prevent recursive inclusion
#ifndef __RR_AUTOGENE_H
#define __RR_AUTOGENE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "rr.h"

// parameters will be stored in eeprom
typedef struct {
  char        kenn[10];             // Kennung; always in the first place
  uint8_t  active; // d;    00 regulator is active / used; 0: not used
  float    tvs;    // degC; 01 Vorlauf, switch to summer if below
  float    tvw;    // degC; 02 Vorlauf, switch to winter if above
  float    dts;    // sec;  03 Intervall fuer Regelzyklus Sommerbetrieb
  float    dtw;    // sec;  04 Intervall fuer Regelzyklus Winterbetrieb
  float    dtS;    // days; 05 <20.0 !!! Ruhezeit nach der eine Ventilbewegung ausgefuehrt wird
  float    dtSZu;  // sec;  06 time to close valve from open position for summer operation
  float    dtWAuf; // sec;  07 time to open valve from close position in winter operation
  float    dtInst; // sec;  08 time to hold valves open until normal operation continues
  float    tv0;    // degC; 09 x-Achse 1. Punkt der Kennlinie
  float    tr0;    // degC; 10 y-Achse 1. Punkt der Kennlinie
  float    tv1;    // degC; 11 x-Achse 2. Punkt der Kennlinie
  float    tr1;    // degC; 12 y-Achse 2. Punkt der Kennlinie
  float    dtLed;  // sec;  13 Einschaltdauer fuer LEDs
  float    ttol;   // degC; 14 Toleranz; unter +/-ttol wird nicht geregelt
  float    pFZu;   // d;    15 proportionalfaktor fuer Regelabweichung Ventil Zu
  float    pFAuf;  // d;    16 proportionalfaktor fuer Regelabweichung Ventil Auf
  float    dtMMn;  // sec;  17 minimale Motorlaufzeit fuer Regeln
  float    dtMMx;  // sec;  18 maximale Motorlaufzeit fuer zum auf Anschlag fahren
  float    IStop;  // mA;   19 stop motor if current exceeds this value; limited to MOTOR_I_STOP
  float    IMn;    // mA;   20 stop motor if current is below -> motor not connected
  float    dtVoc;  // sec;  21 time for a valve travel from open to closed or vice versa
  uint8_t  vFix;   // d;    22 0 regulate normal; 1 if valve kept open; 2 if closed;
  uint8_t  test;   // d;    23 testMode;
  uint8_t  fast;   // d;    24 anz. schnellerer Zyklen f. Test; zaehlt auf 0 runter
  float    tvzTValid;// degC; 25 Gueltigkeit der Vorlauf Temp. von Zentrale; seriell empfangen
  // !!! ALSWAYS IN LAST PLACE !!!
  uint32_t    checksum;
  uint32_t    buffer1;  // some space to compensate byte boundary
  uint32_t    buffer2;  // of eeprom words to read or write
} parameter_t;

// *** global pre-declarations
void set_param_limits( void );
void param2default_valve( parameter_t* p );
int16_t parse_param( char *s0 );
void param_var_string( char *s );
void param_var_values( char *s );

#ifdef __cplusplus
}
#endif

#endif /* __RR_AUTOGENE_H */


