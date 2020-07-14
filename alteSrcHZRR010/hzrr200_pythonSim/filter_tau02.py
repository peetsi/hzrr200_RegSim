#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
***************************************************
calculate filter constant of simple low-pass filter
***************************************************

calculate a step answer from 1 to 0;
take (1 - factor) * last filtered value + 0

The damping factor to reach 1/e is tau in n-count units
rev. 01: demonstrate time constatn tau as n_tau as number of interations
rev. 02: recalculate with time-units in minutes

'''


import numpy as np

tau = 30.0 * 60.0     # seconds;  to reach 1/e of amplitude
dt  =  2.0            # seconds;  for measured samples
nTau = tau / dt       # muber of iterations to reach tau
fakt = 1.0/nTau
print("fakt=%f"%(fakt))

s=1
i=0
while(1):
    s = (1.0-fakt)*s + 0
    if s <= 1/np.e :
        print("n_tau = ",i)
        break
    i+=1
print("exact value for fakt=%f, 1/fakt=%f "%(fakt, 1/fakt))
print("1/e=%f; iteration value=%f; difference=%f"%(1/np.e, s, 1/np.e - s))
nCalc=np.log(1/np.e)/np.log(1-fakt)
print("calculated n for factor %f is %f"%(fakt,nCalc));

print()
ti = i * dt
tCalc = nCalc * dt
print("time to reach 1/e of amplitude in seconds:")
print("for %d iteratons ti=%f; for %f iterastions tCalc=%f"%(i,ti,nCalc,tCalc))

print()
print("time to reach 1/e of amplitude in minutes:")
ti=ti/60.0
tCalc=tCalc/60.0
print("time to reach 1/e of amplitude in seconds:")
print("for %d iteratons ti=%f; for %f iterastions tCalc=%f"%(i,ti,nCalc,tCalc))



