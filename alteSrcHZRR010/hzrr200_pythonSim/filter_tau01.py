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

'''


import numpy as np


fakt = 0.00001
s=1
i=0
while(1):
    s = (1.0-fakt)*s + 0
    i+=1
    if s <= 1/np.e :
        print("n_tau = ",i)
        break
print("exact value for fakt=%f, 1/fakt=%f "%(fakt, 1/fakt))
print("1/e=%f; iteration value=%f; difference=%f"%(1/np.e, s, 1/np.e - s))
nCalc=np.log(1/np.e)/np.log(1-fakt)
print("calculated n for factor %f is %f"%(fakt,nCalc));


