# -*- coding: utf-8 -*-
"""
Created on Mon Dec 15 13:03:33 2025

@author: RodriguesAT
"""
from glob import glob
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
%matplotlib inline
# data_cols = {"AccBias":["x","y","z"],
#              "GPS":["lat","lon",'alt','vel_n','vel_e','vel_d'],
#              "Grav":["x","y","z"],
#              "GyroBias":["x","y","z"],
#              "IMU":["acc_x","acc_y","acc_z","ang_x","ang_y","ang_z"],
#              "MAG":["x","y","z"],
#              "Pos":["n","e","d"],
#              "Quat":["w","x","y","z"],
#     }
# default_cols = ['time','rows','cols']

def load_data(folder):
    data = {}
    for f in glob(os.path.join(folder,"*.csv")):
        data_name = os.path.basename(f).replace(".csv","")
        array = np.genfromtxt(f,delimiter=',')
        shape = array[0,1:3].astype(int)
        array =np.delete(array,[1,2],axis=1)
        
        data[data_name] = {"data":array[:,1:],"shape":shape,"time":array[:,0]*1e-6}
    return data

def plot_three(data,label,times=None,components=None,**kwargs):
    if components is None:
        components = 'XYZ'
    if times is None:
        plt.plot(data[:,0],data[:,1],label=f'{label} {components[0]}',**kwargs)
        plt.plot(data[:,0],data[:,2],label=f'{label} {components[1]}',**kwargs)
        plt.plot(data[:,0],data[:,3],label=f'{label} {components[2]}',**kwargs)            
    else:
        plt.plot(times,data[:,0],label=f'{label} {components[0]}',**kwargs)
        plt.plot(times,data[:,1],label=f'{label} {components[1]}',**kwargs)
        plt.plot(times,data[:,2],label=f'{label} {components[2]}',**kwargs)    
        
def quaternion_to_euler(quaternions):
    """
    Converts a Quaternion (Hamilton Convention: [w, x, y, z]) or an 
    nx4 array of Quaternions to Euler angles (roll, pitch, yaw) 
    (intrinsic Z-Y-X sequence).

    Args:
        quaternions (np.array): A 4-element unit quaternion [w, x, y, z] 
                                or an nx4 array of quaternions.

    Returns:
        np.array: A 3-element array [roll, pitch, yaw] in radians, 
                  or an nx3 array of [roll, pitch, yaw] for array input.
    """
    # Ensure input is a NumPy array and determine if it's a single quaternion or an array
    quaternions = np.asarray(quaternions)
    
    # Check if the input is a single 4-element quaternion
    is_single_quaternion = quaternions.ndim == 1 or quaternions.shape[-1] != 4
    if is_single_quaternion:
        # Reshape to a 1x4 array for unified processing
        q = quaternions.reshape(1, 4)
        was_single = True
    else:
        # Input is an nx4 array
        q = quaternions
        was_single = False

    # Separate quaternion components
    # The convention is [w, x, y, z]
    q_w = q[:, 0]
    q_x = q[:, 1]
    q_y = q[:, 2]
    q_z = q[:, 3]
    
    # --- Calculations for Z-Y-X (Roll-Pitch-Yaw) sequence ---
    
    # Roll (Rotation around X-axis)
    # roll = atan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x**2 + q_y**2))
    sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z)
    cosr_cosp = 1.0 - 2.0 * (q_x**2 + q_y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (Rotation around Y-axis)
    # pitch = asin(2*(q_w*q_y - q_z*q_x))
    sinp = 2.0 * (q_w * q_y - q_z * q_x)
    
    # Clamping the argument of arcsin to the range [-1, 1] to prevent
    # domain errors due to floating point inaccuracies
    # The max value for sinp is +/-1.0, but float precision can cause values
    # slightly outside this range (e.g., 1.0000000000000002)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    
    # Yaw (Rotation around Z-axis)
    # yaw = atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y**2 + q_z**2))
    siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1.0 - 2.0 * (q_y**2 + q_z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    # Combine the results into an Nx3 array
    eulers = np.stack((roll, pitch, yaw), axis=-1)
    
    # Return the correct shape based on the input
    if was_single:
        # Return a 1D array if a single quaternion was input
        return eulers.flatten()
    else:
        # Return the Nx3 array for array input
        return eulers
    
if __name__ == "__main__":
    folder=r'C:\Users\gunni\Desktop\quadsim\simulation\build\logs\2025-12-15_19-14-32'
    data = load_data(folder)
    imu_acc = data['IMU']['data'][:,:3]
    
    plt.figure(figsize=(10, 6))
    plt.title("Acc")
    plot_three(data['IMU']['data'][:,:3],times=data['IMU']['time'],label='meas')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("Gyro")
    plot_three(data['IMU']['data'][:,3:],times=data['IMU']['time'],label='meas')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("Bias")
    plot_three(data['GyroBias']['data'],times=data['GyroBias']['time'],label='gyro',ls='--')
    plot_three(data['AccBias']['data'],times=data['AccBias']['time'],label='acc')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("Velocity")
    plot_three(data['Vel']['data'],times=data['Vel']['time'],label='Est',ls='--')
    plot_three(data['GPS']['data'][:,3:],times=data['GPS']['time'],label='Meas')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("Magnetometer")
    norm = data['MAG']['data']/np.linalg.norm(data['MAG']['data'],axis=1)[:,np.newaxis]
    plot_three(norm,times=data['MAG']['time'],label='Meas')
    plot_three(data['MAG_Ref']['data'],times=data['MAG_Ref']['time'],label='Ref',ls='-.')
    plot_three(data['MAG_Pred']['data'],times=data['MAG_Pred']['time'],label='Pred',ls='--')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("orientation")
    est_ori = np.rad2deg(quaternion_to_euler(data['Quat']['data']))
    plot_three(est_ori,'Estimated',times=data['Quat']['time'],ls='--')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("LL")
    plt.plot(data['GPS']['time'],data['GPS']['data'][:,0])
    plt.plot(data['GPS']['time'],data['GPS']['data'][:,1])
    plt.legend()
    plt.grid()