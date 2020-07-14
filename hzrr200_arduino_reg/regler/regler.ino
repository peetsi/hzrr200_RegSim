/* ===========================================================
Regler Entwurf

--------------------------------------------------------------
*/





// Status data structure:
// (partly: contains only varibles for the regulator) 
typedef struct {
    // see function characteristic() for details:
    float tRSet;        // set value of Ruecklauf temperature (Sollwert)
} controllerStatus_t;

typedef struct {
    float tVorMeas;     // locally measured Vorlauf temperature
    float tVorCentral;  // Vorlauf temperature received from master via network
    float tVor;         // actually used Vorlauf temperature
    controllerStatus_t cs[3];       // cs <-> "controller status" for controllers 0...3
} status_t;
status_t st;




// Parameter data structure:
// (partly: contains only varibles for the regulator) 
typedef struct {
    float tv0,tv1;     // see function characteristic() for details
    float tr0,tr1;     // see function characteristic() for details
} controllerParameter_t;

typedef struct {
    controllerParameter_t cp[3];       // cp <-> "controller parameter" for controllers 0...3
} parameter_t;
parameter_t par;




/* ------------------------
    calculate temperature 
    Ruecklauf from Vorlauf
    -----------------------
*/    
void init_characteristic(void) {
    byte i;
    for(i=0;i<3;i++){
        par.cp[i].tv0 = 40.0;   // degC; min. Vorlauf
        par.cp[i].tv1 = 75.0;   // degC; max. Vorlauf
        par.cp[i].tr0 = 32.0;   // degC; min. Ruecklauf
        par.cp[i].tr1 = 46.0;   // degC; max. Ruecklauf
    }
}


/*
 verwendet Vorlauftemperatur tv und berechnet
 anhand einer Kennlinie von (tv0,tr0) - (tv1,tr1) 
 die zugehoerige Ruecklauftemperatur y

tr1|- - - - - - - +-----
   |             /:
   |           /  :
 y |- - - - -+    :
   |       / :    :
tr0|----+/   :    :
   |    :    :    :
   |    :    :    :
   +---------+----------
      tv0   tv   tv1
*/
uint8_t characteristic( uint8_t valve, float tv ) {   
    // input:
    //      valve e {0,1,2}
    //      tv Vorlauf temperature to be used
    //      status_t st        (global)
    //      parameter_t par    (global)
    // return: 
    //      set value of Ruecklauf temperature
    //      set st.cs[valve].tRSet (global)
    controllerParameter_t cp;     // for less typing work in the following :c)      
    cp = par.cp[valve];           // controller parameters of actual valve
    float m,y;
      
    // *** calculate Ruecklauftemperatur from Vorlauftemperatur using characteristic curve
    if     ( tv <= cp.tv0 ) y = cp.tr0;                     // minimum tr0
    else if( tv >= cp.tv1 ) y = cp.tr1;                     // maximum tr1
    else {
        m = (cp.tr1 - cp.tr0) / (cp.tv1 - cp.tv0);    // slope of line (Steigung)
        y = m * ( tv - cp.tv0 ) + cp.tr0;             // Sollwert Ruecklauftemperatur
        st.cs[valve].tRSet = y;                             // set result in status
     }
     return y;
}



// ================================================
// End functions
// ================================================

// TEST functions above
void test_regulator(void) {
    float tempVL[]={30.0, 40.0, 52.5, 75.0, 80.0};  // test temperatures for Vorlauf
    byte reg,i;
    float tr;
    
    init_characteristic();
    for(reg=0;reg<3;reg++){
        Serial.print("Vorl. Rueckl. Regler");
        Serial.println(reg);
        for(i=0;i<5;i++){
            tr=characteristic( reg, tempVL[i] );
            Serial.print(tempVL[i]);
            Serial.print("  ");
            Serial.print(tr);
            Serial.print("  ");
            Serial.println(tr);
        };
    };
};


void setup( void ) {
    Serial.begin(9600);
    test_regulator();
}

void loop(void){

}






/*
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

*/
