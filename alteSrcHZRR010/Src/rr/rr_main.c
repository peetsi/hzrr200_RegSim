// HZ-RR010
// Heizungs Ruecklauf Regler, Version 01.0
//
// Peter Loster (c); 2015, 2016
//

#define EVER ;;
#include "rr.h"

// ----- variables
volatile flags_t flags;
parameter_t par[4];     // 4 parameter sets for 4 regulators



// TODO only used for test
//      finally the functions work in freeRTOS tasks
void rr( void ) {
  rr_init();
  rr_measure();     // measure all input values
}

