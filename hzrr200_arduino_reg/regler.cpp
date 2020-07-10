/* ===========================================================
Regler Entwurf

--------------------------------------------------------------
*/





// Status data structure:
// (partly: contains only varibles for the regulator) 
typedef struct {

}


typedef struct {

} status_t;
status_t st;




// Parameter data structure:
// (partly: contains only varibles for the regulator) 
typedef struct {
    

} controllerParameter_t;

typedef struct {
    controllerParameter_t cp[3];       // cp <-> "controller parameter" for controllers 0...3

} parameter_t;
parameter_t par;




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


