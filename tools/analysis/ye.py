# -*- coding: utf-8 -*-
"""
Created on Sun Feb 15 13:02:40 2026

@author: gunni
"""
from glob import glob
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt

def load_data(folder):
    data = {}
    for f in glob(os.path.join(folder,"*.csv")):
        data_name = os.path.basename(f).replace(".csv","")
        array = np.genfromtxt(f,delimiter=',',skip_footer=1)
        try:
            shape = array[0,1:3].astype(int)
        except IndexError:
            print(f)
        array =np.delete(array,[1,2],axis=1)
        
        data[data_name] = {"data":array[:,1:],"shape":shape,"time":array[:,0]*1e-6}
    return data


folder=r"C:\Users\gunni\Desktop\6dof\build\logs\2026-02-15_14-12-41"

data=load_data(folder)
print(data.keys())

plt.figure()
plt.plot(data['command_raw']['time'],data['command_raw']['data'])
plt.twinx()
plt.plot(data['omega_raw']['time'],data['omega_raw']['data'])
plt.plot(data['omega_filt']['time'],data['omega_filt']['data'])


plt.figure()
plt.plot(data['Motor1Est']['time'],data['Motor1Est']['data'])

plt.figure()
plt.plot(data['Motor1Cov']['time'],data['Motor1Cov']['data'])

plt.figure()
plt.plot(data['Motor1X']['time'],data['Motor1X']['data'])