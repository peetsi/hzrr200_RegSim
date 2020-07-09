# hzrr200 Rücklauf-Regler Simulation
# Peter Loster, Mai/Juni 2020
#
import time
import numpy as np
import matplotlib.pyplot as plt

class St:                # Status variables
    tOld     = -9999.0   # previous timestamp initial value
    dt       = -9999.0   # timeslice, initial value
    tempVlLP = 60.0      # Temperature Vorlauf low-pass
    tempRlLP =  0.0      # Temperature Rücklauf, low-pass
    tempRLOld=-99.0      # previous Rücklauf value; initial value
    mRL      =  0.0      # degC/min; RL temperature slope
    motPauseEnd =  0.0   # end-time of valve-motor inactive
    
class Par:        # Parameter variables, in EEPROM
    mUp = 0.2     # Steigung fuer Aufwärtsflanke
    mDn = -0.5    # Steigung fuer Abwärtsflanke
    rlLP = 0.01   # Rücklauf Faktor Tiefpass
    offtime = 45  # minutes to stop regulator after strong slope
    motDelay = 10 # min; stop motor action after slope too big detected

def temp_verlauf( tvon, tbis, dt, T, a0 ):
    # tvon, tbis in Minuten
    t = np.arange( tvon, tbis, dt )
    tr = 2.0*np.pi * t / T
    rl = a0 * np.sin(tr)
    return (t,rl)

def regler( tn,tempVl,tempRl ):
    # tn       minutes;  actual time
    # tempVl   degC;     Vorlauf temperature
    # tempRl   degC;     Rücklauf temperature
    # Rücklauf; fast lowpass filter
    St.tempRlLP = tempRl * Par.rlLP + St.tempRlLP * (1.0 - Par.rlLP)
    
    # calculate timeslice
    if St.tOld > 0.0 :      # calculate next timeslice
        St.dt = tn - St.tOld
    St.tOld = tn
    
    # find slope of input tempRl
    if St.tempRLOld > -90.0 :    # calculate slope angle (tangens)
        dTempRL = tempRl - St.tempRLOld
        St.mRL = dTempRL / St.dt;
    St.tempRLOld = tempRl;
            
    # find too high slopes
    # if found: stop regulator for moving valve-motor to avoid
    #     resonance effects
    if St.mRL > Par.mUp :
        m2high = 0.9
        # start valve-motor delay time
        #St.motPauseEnd = time.time() + Par.motDelay * 60.0 # minutes->sec
        St.motPauseEnd = tn + Par.motDelay
    else:
        m2high = 0
    if St.mRL < Par.mDn :
        m2low = -0.9
        # start valve-motor delay time
        #St.motPauseEnd = time.time() + Par.motDelay * 60.0 # minutes->sec
        St.motPauseEnd = tn + Par.motDelay
    else:
        m2low = 0

    # for simulation only:
    #if time.time() < St.motPauseEnd:
    if tn < St.motPauseEnd:
        mp=1
    else:
        mp=0.0
        
    # *** activate motor if too long disabled
    # TODO parameter in Par for max. idle time of motor
    # TODO switch motor on only for a limited time <-- other parameter
    
    # *** main regulator 
    # always use mean temperature of RL (not current temp.)
    # do nothing if motor delay is active
    pass

    # *** return values for plotting
    return (St.tempRlLP, St.mRL, m2high, m2low, mp)


if __name__ == "__main__":

    # generate changing temperature values over time
    vonMinutes = 0
    bisMinutes = 240
    dt         = 1.0    # min;  Taktrate Minuten
    TMinutes   = 60     # min;  Periode einer Schwingung
    a0         = 5.0    # degC; Amplitude in Grad Celsius
    (t,tempRL)=temp_verlauf(vonMinutes,bisMinutes,dt,TMinutes,a0)
    
    rlLP=[]             # degC;  temperature Rücklauf after low-pass
    mRL =[]             # K/min; temperature slope Rücklauf
    mHi =[]             # too high positive RL slope detected
    mLo =[]             # too low negative RL slope detected
    motPause=[]         # 1 if valve-motor shall not work
    
    for i in range (len(t)):
        (tempRlLP,m,a,b,mp) = regler(t[i],60.0,tempRL[i])
        rlLP.append(tempRlLP)
        mRL.append(m)
        mHi.append(a)   # only used for sim-plot
        mLo.append(b)   # only used for sim-plot
        motPause.append(mp) # motor pause active
        
    # plot results    
    plt.plot(t,tempRL,label="tempRL")
    plt.plot(t,rlLP,label="rlLP")
    plt.plot(t,mRL,label="mRL")
    plt.plot(t,mHi,label="mHi")
    plt.plot(t,mLo,label="mLo")
    plt.plot(t,motPause,":",label="motPause")
    plt.grid()
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
    
    
    

        