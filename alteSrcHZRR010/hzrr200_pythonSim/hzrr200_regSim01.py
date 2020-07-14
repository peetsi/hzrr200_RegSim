#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# hzrr200 Rücklauf-Regler Simulation
# Peter Loster, Mai/Juni 2020
#
import time
import numpy as np
import matplotlib.pyplot as plt

# *** status variables are changed during operation
# NOTE: values have to be initialized for formal reasons
#       they are set in a separate init function
class St:         # Status variables
    firstLoop =1    # 1 if first loop is in progress
    tOld      =0    # sec; previous timestamp initial value
    dt        =0    # sec; timeslice, initial value
    tempVlLP  =0    # Temperature Vorlauf low-pass
    tempRlLP  =0    # Temperature Rücklauf, low-pass
    tempRlLP2 =0    # Temp.Rücklauf, low-pass 2.order
    mRL       =0    # degC/sec; RL temperature slope
    motPauseEnd =0  # sec; end-time of valve-motor inactive
# *** parameters are stored in EEPROM; may be changed via network
class Par:                # Parameter variables, in EEPROM
    fMeas = 0.2           # measurments per second
    tauTVL= 10.0*60.0     # sec; tau to reach 1/e amplitude of VL with low-pass filter
    tauTRL= 30.0*60.0     # sec; tau to reach 1/e amplitude of RL with low-pass filter
    mUp   =  7.0          # degC/sec; Steigung fuer Aufwärtsflanke
    mDn   = -8.0          # degC/sec; Steigung fuer Abwärtsflanke
    offtime  = 45.0*60.0  # sec; to stop motor after strong slope
    motDelay  =10*60    # sec; pause motor after steep slope
    
def init_var_status():
    St.firstLoop = 1;
    St.tOld = -9999.0
    St.dt   = 1.0 / Par.fMeas
    print("St.dt=",St.dt)
    St.tempVlLP = 60.0        # degC; start value -> measured later
    St.tempRlLP = 44.0        # degC; start value -> measured later
    St.mRL      = 0.0         # degC/sec; start value
    motPauseEnd = time.time() # sec; End of pause set to now

def regler( tn,tempVl,tempRl ):
    # tn       sec;      actual time
    # tempVl   degC;     Vorlauf temperature
    # tempRl   degC;     Rücklauf temperature
    # Rücklauf; fast lowpass filter

    if St.firstLoop > 0:
        St.tempVlLP = tempVl     # init low-pass filter Vorlauf
        St.tempRlLP = tempRl     # init low-pass filter Ruecklauf
        St.mRl = 0.0             # init slope Ruecklauf
        m2high = 0.0             # init slope Ruecklauf too high
        m2low  = 0.0             # init slope Ruecklauf too low
        mPause = tn-1            # endtime for slope-pause
        St.tempRLOld = tempRl
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
        # always use mean temperature of RL (not current temp.)
        # do nothing if motor delay is active
        pass

    # *** return values for plotting
    St.firstLoop=0
    return (St.tempRlLP, St.tempRlLP2, St.mRL, m2high, m2low, mPause)


if __name__ == "__main__":


    # **** ATTENTION +++ ATTENTION +++ ATTENTION +++ ATTENTION +++
    ''' FIRST read all measured values THEN initialize variables '''
    # **** ATTENTION +++ ATTENTION +++ ATTENTION +++ ATTENTION +++
    
    init_var_status()
    
    # *** generate a temperature curve
    # TODO alternatively read it from file
    def temp_verlauf( tvon, tbis, dt, T, a0 ):
        # tvon, tbis in Minuten
        t = np.arange( tvon, tbis, dt )
        tr = 2.0*np.pi * t / T
        rl = a0 * np.sin(tr)
        return (t,rl)


    # generate changing temperature values over time
    vonSeconds = 0
    bisSeconds = 240*60
    dt         = 1.0/Par.fMeas    # sec;  sample rate
    TMinutes   = 60*60     # sec;  Periode einer Schwingung
    a0         = 5.0    # degC; Amplitude in Grad Celsius
    (t,tempRL)=temp_verlauf(vonSeconds,bisSeconds,dt,TMinutes,a0)
    tm=t/60.0
    
    rlLP=[]             # degC;  temperature Rücklauf after low-pass
    rlLP2=[]            # degC;  temperature Rücklauf after 2. low-pass
    mRL =[]             # K/min; temperature slope Rücklauf
    mHi =[]             # too high positive RL slope detected
    mLo =[]             # too low negative RL slope detected
    motPause=[]         # 1 if valve-motor shall not work
    
    St.firstLoop=1      # indicate first loop to all iterations
    for i in range (len(t)):
    #for i in range (5):
        (tempRlLP,tempRlLP2,m,a,b,mp) = regler(t[i],60.0,tempRL[i])
        St.firstloop=0
        # store results for later plotting
        rlLP.append(tempRlLP)
        rlLP2.append(tempRlLP2)
        mRL.append(m)
        mHi.append(a)   # only used for sim-plot
        mLo.append(b)   # only used for sim-plot
        motPause.append(mp) # motor pause active
        
    # plot results    
    plt.plot(tm,tempRL,label="tempRL")
    plt.plot(tm,rlLP,label="rlLP")
    plt.plot(tm,rlLP2,label="rlLP2")
    plt.plot(tm,mRL,":",label="mRL")
    plt.plot(tm,mHi,label="mHi")
    plt.plot(tm,mLo,label="mLo")
    plt.plot(tm,motPause,":",label="motPause")
    plt.grid()
    plt.xlabel("Minutes")
    plt.legend()
    plt.show()
        


    '''
    m = [rl[i]-rl[i-1] for i in range(1,len(t)) ]
    m.insert(0,0)
    print(m)
    mUp = [ 1 if m[i] > Par.mUp else 0 for i in range(len(m)) ]
    mDn = [-1 if m[i] < Par.mDn else 0 for i in range(len(m)) ]

    delayUp = []
    cnt=0
    for x in mUp:
        if x != 0:
            delayUp.append(.9)
            cnt = Par.offtime
        else:
            if cnt > 0:
                cnt-=1
                delayUp.append(.9)
            else:
                delayUp.append(0)

    delayDn = []
    cnt=0
    for x in mDn:
        if x != 0:
            delayDn.append(-.9)
            cnt = Par.offtime
        else:
            if cnt > 0:
                cnt-=1
                delayDn.append(-.9)
            else:
                delayDn.append(0)



    print(max(m),min(m))
    print(mUp)
    rlMean = np.zeros(len(m),float)
    rlMean[0] = rl[0]
    rlMean = [rlMean[i-1]*(1-Par.fakt) + rl[i]*Par.fakt for i in range(1,len(rl))]
    rlMean.insert(0,rl[0])
    rlMeanA = np.array(rlMean )
    print(rlMeanA)

    ma = np.array(m)
    plt.plot(t,rl)
    plt.plot(t,ma)
    plt.plot(t,mUp)
    plt.plot(t,mDn)
    plt.plot(t,rlMeanA)
    plt.plot(t,delayUp,":")
    plt.plot(t,delayDn,":")
    plt.show()
'''
    
    
    

        
