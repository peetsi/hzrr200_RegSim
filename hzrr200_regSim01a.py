#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# hzrr200 Rücklauf-Regler Simulation
# Peter Loster, Mai/Juni 2020
#
import time
import numpy as np
import matplotlib.pyplot as plt

# *** constants
VALVE_OPEN = 1
VALVE_CLOSE= 2

# for test
motorPos = 30.0


# *** status variables are changed during operation
# NOTE: values have to be initialized for formal python-reasons
#       they are set in a separate init function
class St:         # Status variables
    # *** common to all regulators
    
    # *** for each regulator
    tRlSet    =0    # set value for ruecklauf regulator
    firstLoop =0    # 1 if first loop is in progress
    tOld      =0    # sec; previous timestamp initial value
    dt        =0    # sec; timeslice, initial value
    tempVlLP  =0    # Temperature Vorlauf low-pass
    tempRlLP  =0    # Temperature Rücklauf, low-pass
    tempRlLP2 =0    # Temp.Rücklauf, low-pass 2.order
    mRL       =0    # degC/sec; RL temperature slope
    motPauseEnd =0  # sec; end-time of valve-motor inactive
# *** parameters are stored in EEPROM; may be changed via network
class Par:                # Parameter variables, in EEPROM
    # *** common to all regulators:
    fMeas = 0.2           # measurments per second
    # Kennlinie, common for all regulators
    # see diagram at function characteristic()
    tv0   = 40.0
    tv1   = 75.0
    tr0   = 32.0
    tr1   = 46.0
    # *** for each regulator:
    mode  = 1             # 0:inactive; 1:ruecklauf regulator; 2:Roomtemp.Reg (for reg.1)
    tauTVL= 10.0*60.0     # sec; tau to reach 1/e amplitude of VL with low-pass filter
    tauTRL= 30.0*60.0     # sec; tau to reach 1/e amplitude of RL with low-pass filter
    mUp   =  7.0          # degC/sec; Steigung fuer Aufwärtsflanke
    mDn   = -8.0          # degC/sec; Steigung fuer Abwärtsflanke
    offtime  = 45.0*60.0  # sec; to stop motor after strong slope
    motDelay  =10*60      # sec; pause motor after steep slope
    dTempMin  = 1         # K; temperature difference to allowed minimum tolerance
    dTempMax  = 1         # K; temperature difference to allowed maximum tolerance
    secPerK   = 0.05      # sec/K; seconds of motor on per Kelvin difference
    tMotMin   = 0.1       # sec;  minimum motor on-time to have an action

    
def init_var_status():
    St.firstLoop = 1;
    St.tOld = -9999.0
    St.dt   = 1.0 / Par.fMeas
    print("St.dt=",St.dt)
    St.tempVlLP = 60.0        # degC; start value -> measured later
    St.tempRlLP = 44.0        # degC; start value -> measured later
    St.mRL      = 0.0         # degC/sec; start value
    motPauseEnd = time.time() # sec; End of pause set to now


'''
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
'''

def characteristic( valve, tv ):
    # calculate set value for Ruecklauf from Vorlauf tv
    # see above
    ''' NOTE: only one regulator - St. and Par. variables are indexed !!! '''
    if tv <= Par.tv0 :
        y = Par.tv0
    elif tv >= Par.tv1 :
        y = Par.tv1
    else:
        m = (Par.tr1 - Par.tr0) / (Par.tv1 - Par.tv0)
        y = m * ( tv - Par.tv0 ) + Par.tr0
        St.tempRlSet = y
    return y


def regler( tn,tempVl,tempRl ):
    # tn       sec;      actual time
    # tempVl   degC;     Vorlauf temperature
    # tempRl   degC;     Rücklauf temperature
    # Rücklauf; fast lowpass filter
    global motorPos
    if Par.mode == 0:
        return                   # inactive - do nothing
    if St.firstLoop > 0:
        St.tempVlLP = tempVl     # init low-pass filter Vorlauf
        St.tempRlLP = tempRl     # init low-pass filter Ruecklauf
        St.tempRlLP2= tempRl     # init low-pass filter 2. order Ruecklauf
        St.mRl = 0.0             # init slope Ruecklauf
        m2high = 0.0             # init slope Ruecklauf too high
        m2low  = 0.0             # init slope Ruecklauf too low
        mPause = tn-1            # endtime for slope-pause
        St.tempRLOld = tempRl
        dTemp=0.0
        diff=0.0
        motorPos=25.0
    else:
        #print(".")
        # *** claculate filter factor for low-pass of VL and RL
        faktVL = 1.0/(Par.tauTVL*Par.fMeas) # 1; filter factor
        faktRL = 1.0/(Par.tauTRL*Par.fMeas) # 1; filter factor
        St.tempVlLP = tempVl * faktVL + St.tempVlLP * (1.0 - faktVL)
        St.tempRlLP = tempRl * faktRL + St.tempRlLP * (1.0 - faktRL)
        St.tempRlLP2= St.tempRlLP * faktRL + St.tempRlLP2 * (1.0 - faktRL)
    
        # *** RL temperature slope evaluation
        # find slope of input tempRl
        dTempRL = (tempRl - St.tempRLOld)*1000   # mK; temp. change
        St.mRL = dTempRL / St.dt;                # mK/sec; slope over time
        #print("dTempRL,St.dt,St.mRL=",dTempRL,St.dt,St.mRL)
        St.tempRLOld = tempRl;
        # find too high slopes
        # if found: stop regulator for moving valve-motor to avoid
        #     resonance effects
        #print(St.mRL,Par.mUp)
        if St.mRL > Par.mUp :
            m2high = 0.9            # m too high
            # start valve-motor delay time
            St.motPauseEnd = tn + Par.motDelay
        else:
            m2high = 0
        if St.mRL < Par.mDn :
            m2low = -0.9            # m too low
            # start valve-motor delay time
            St.motPauseEnd = tn + Par.motDelay
        else:
            m2low = 0

        # for simulation only:
        #print("tn=%f; motPauseEnd=%f"%(tn,St.motPauseEnd))
        if tn < St.motPauseEnd:
            mPause=1
        else:
            mPause=0.0
            
        # *** activate motor if too long disabled
        # TODO parameter in Par for max. idle time of motor
        # TODO switch motor on only for a limited time <-- other parameter
        
        # *** main regulator
        tempSet = characteristic( 0, St.tempVlLP)       # 0 is a dummy, only one regualtor for test
        dTemp = tempSet - St.tempRlLP
        if   dTemp < St.tempRlLP - Par.dTempMin:
            diff = dTemp - Par.dTempMin    # K; difference below lower tolerance
            St.tMot = - Par.secPerK * diff + Par.tMotMin   # motor on-time; diff is negative
            St.tDir = VALVE_OPEN
            motorPos -= St.tMot
        elif dTemp > St.tempRlLP + Par.dTempMax:
            diff = dTemp - Par.dTempMax    # K; difference over upper tolerance
            St.tMot = Par.secPerK * diff + Par.tMotMin   # motor on-time; diff is negative
            St.tDir = VALVE_CLOSE
            motorPos += St.tMot
        else:
            dTemp=0.0
            diff=0.0
    # *** return values for plotting
    St.firstLoop=0
    return (St.tempRlLP, St.tempRlLP2, St.mRL, m2high, m2low, mPause,dTemp,diff,motorPos)


if __name__ == "__main__":


    # **** ATTENTION +++ ATTENTION +++ ATTENTION +++ ATTENTION +++
    ''' FIRST read all measured values THEN initialize variables '''
    # **** ATTENTION +++ ATTENTION +++ ATTENTION +++ ATTENTION +++
    
    init_var_status()
    
    # *** generate a temperature curve
    # TODO alternatively read it from file
    def temp_verlauf( tvon, tbis, dt, T, a0, offset ):
        # tvon, tbis in Minuten
        t = np.arange( tvon, tbis, dt )
        tr = 2.0*np.pi * t / T
        rl = a0 * np.sin(tr) + offset
        return (t,rl)


    # generate changing temperature values over time
    vonSeconds = 0
    bisSeconds = 240*60
    dt         = 1.0/Par.fMeas    # sec;  sample rate
    TMinutes   = 60*60     # sec;  Periode einer Schwingung
    a0         = 5.0    # degC; Amplitude in Grad Celsius
    offset     = 40.0
    (t,tempRL)=temp_verlauf(vonSeconds,bisSeconds,dt,TMinutes,a0,offset)
    tm=t/60.0
    
    rlLP=[]             # degC;  temperature Rücklauf after low-pass
    rlLP2=[]            # degC;  temperature Rücklauf after 2. low-pass
    mRL =[]             # K/min; temperature slope Rücklauf
    mHi =[]             # too high positive RL slope detected
    mLo =[]             # too low negative RL slope detected
    motPause=[]         # 1 if valve-motor shall not work
    dTemp=[]            # regulator difference
    diff=[]             # difference
    motPosA=[]          # relative motor position
    
    St.firstLoop=1      # indicate first loop to all iterations
    for i in range (len(t)):
    #for i in range (5):
        (tempRlLP,tempRlLP2,m,a,b,mp,dt,d,mPos) = regler(t[i],60.0,tempRL[i])
        St.firstloop=0
        # store results for later plotting
        rlLP.append(tempRlLP)
        rlLP2.append(tempRlLP2)
        mRL.append(m)
        mHi.append(a)   # only used for sim-plot
        mLo.append(b)   # only used for sim-plot
        motPause.append(mp) # motor pause active
        dTemp.append(dt)
        diff.append(d)
        motPosA.append(mPos)
        
    # plot results    
    plt.plot(tm,tempRL,label="tempRL")
    plt.plot(tm,rlLP,label="rlLP")
    plt.plot(tm,rlLP2,label="rlLP2")
    plt.plot(tm,mRL,":",label="mRL")
    plt.plot(tm,mHi,label="mHi")
    plt.plot(tm,mLo,label="mLo")
    plt.plot(tm,motPause,":",label="motPause")
    plt.plot(tm,dTemp,".-",label="dTemp")
    plt.plot(tm,diff,".-",label="diff")
    plt.plot(tm,motPosA,".-",label="motPos")
    plt.grid()
    plt.xlabel("Minutes")
    plt.legend()
    plt.show()
        

