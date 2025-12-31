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
from matplotlib.widgets import Slider
%matplotlib qt5
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
        array = np.genfromtxt(f,delimiter=',',skip_footer=1)
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
def plot_interactive_heatmap(data_tensor, time_array,x_labels, y_labels, text_data_tensor=None, title_prefix="Data", cmap='coolwarm', vmin=-1, vmax=1):
    """
    Args:
        data_tensor: Used for the heatmap COLORS (usually Correlation).
        text_data_tensor: (Optional) Used for the TEXT values (usually Covariance).
                          If None, defaults to using data_tensor.
    """
    rows, cols = data_tensor.shape[1], data_tensor.shape[2]
    
    # If no specific text data is provided, use the color data for text
    if text_data_tensor is None:
        text_data_tensor = data_tensor

    # 1. Setup the Plot
    fig, ax = plt.subplots(figsize=(14, 10)) # Made slightly wider for scientific notation
    plt.subplots_adjust(bottom=0.25) 
    
    # Initial Image (Colors based on data_tensor)
    im = ax.imshow(data_tensor[0], cmap=cmap, vmin=vmin, vmax=vmax)
    
    ax.set_xticks(np.arange(cols))
    ax.set_yticks(np.arange(rows))
    ax.set_xticklabels(x_labels, rotation=90)
    ax.set_yticklabels(y_labels)
    ax.set_title(f"{title_prefix} at t={time_array[0]:.2f}s")
    fig.colorbar(im, ax=ax)

    # 2. Initialize Text Annotations
    text_annotations = []
    
    # Contrast threshold based on the COLOR data (heatmap intensity)
    range_span = max(abs(vmax), abs(vmin))
    contrast_threshold = range_span * 0.6 

    for i in range(rows):
        row_texts = []
        for j in range(cols):
            # Value for Color (Correlation)
            val_color = data_tensor[0, i, j]
            # Value for Text (Covariance)
            val_text = text_data_tensor[0, i, j]
            
            # Determine text color based on the BACKGROUND intensity
            text_color = "white" if abs(val_color) > contrast_threshold else "black"
            
            # Create text using scientific notation for covariance
            text = ax.text(j, i, f"{val_text:.2e}", 
                           ha="center", va="center", 
                           color=text_color, 
                           fontsize=6) 
            row_texts.append(text)
        text_annotations.append(row_texts)

    # 3. Create the Slider
    ax_slider = plt.axes([0.2, 0.1, 0.6, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(
        ax=ax_slider,
        label='Time (s)',
        valmin=time_array[0],
        valmax=time_array[-1],
        valinit=time_array[0],
    )

    # 4. Update Function
    def update(val):
        t_selected = slider.val
        idx = (np.abs(time_array - t_selected)).argmin()
        
        # Get matrices for this timestep
        matrix_color = data_tensor[idx]
        matrix_text = text_data_tensor[idx]
        
        # Update heatmap colors
        im.set_data(matrix_color)
        
        # Update text values and text colors
        for i in range(rows):
            for j in range(cols):
                val_c = matrix_color[i, j]
                val_t = matrix_text[i, j]
                
                # Update number (Un-normalized Covariance)
                text_annotations[i][j].set_text(f"{val_t:.2e}")
                
                # Update visibility (based on Correlation background)
                text_annotations[i][j].set_color("white" if abs(val_c) > contrast_threshold else "black")

        ax.set_title(f"{title_prefix} at t={time_array[idx]:.2f}s")
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.tight_layout()
    print(f"Plotting {title_prefix}...")
    plt.show()

def plot_covariance_heatmap(data_dict):
    # 1. Extract and reshape data
    raw_cov = data_dict['data']
    times = data_dict['time']
    
    # Reshape to (N, 18, 18) -> This is the RAW COVARIANCE
    cov_matrices = raw_cov.reshape(-1, 18, 18)
    
    # 2. Pre-compute Correlation Matrices (for COLORS)
    corr_matrices = []
    epsilon = 1e-9 
    
    print("Pre-computing correlation matrices...")
    for P in cov_matrices:
        std_dev = np.sqrt(np.diag(P))
        denominator = np.outer(std_dev, std_dev) + epsilon
        rho = P / denominator
        rho = np.clip(rho, -1, 1)
        corr_matrices.append(rho)
        
    corr_matrices = np.array(corr_matrices)

    labels = ['Px', 'Py', 'Pz', 'Vx', 'Vy', 'Vz', 
              'Roll', 'Pitch', 'Yaw', 
              'Ba_x', 'Ba_y', 'Ba_z', 'Bg_x', 'Bg_y', 'Bg_z', 
              'Gx', 'Gy', 'Gz']

    # 3. Call Generic Plotter
    plot_interactive_heatmap(
        data_tensor=corr_matrices,      # Colors = Correlation
        text_data_tensor=cov_matrices,  # Text = Raw Covariance
        time_array=times,
        x_labels=labels,
        y_labels=labels,
        title_prefix="Covariance Analysis",
        cmap='coolwarm',
        vmin=-1,
        vmax=1
    )
    
if __name__ == "__main__":
    plt.close('all')
    folder=r'C:\Users\gunni\Desktop\quadsim\replay\build\logs\2025-12-31_14-53-08'
    # folder=r"C:\Users\gunni\Desktop\quadsim\simulation\build\logs\2025-12-31_11-16-06"
    data = load_data(folder)
    try:
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
    except KeyError:
        pass
    plt.figure(figsize=(10, 6))
    plt.title("Bias")
    plot_three(data['GyroBias']['data'],times=data['GyroBias']['time'],label='gyro',ls='--')
    plot_three(data['AccBias']['data'],times=data['AccBias']['time'],label='acc')
    plt.legend()
    plt.grid()
    # plt.figure(figsize=(10, 6))
    # plt.title("Grav")
    # plot_three(data['Grav']['data'],times=data['Grav']['time'],label='est')
    # plt.legend()
    # plt.grid()
    plt.figure(figsize=(10, 6))
    plt.title("Velocity")
    plot_three(data['Vel']['data'],times=data['Vel']['time'],label='Est',ls='--')
    plot_three(data['GPS']['data'][:,3:],times=data['GPS']['time'],label='Meas')
    plt.legend()
    plt.grid()
    try:
        plt.figure(figsize=(10, 6))
        plt.title("Magnetometer")
        norm = data['MAG']['data']/np.linalg.norm(data['MAG']['data'],axis=1)[:,np.newaxis]
        plot_three(norm,times=data['MAG']['time'],label='Meas')
        plot_three(data['MAG_Ref']['data'],times=data['MAG_Ref']['time'],label='Ref',ls='-.')
        plot_three(data['MAG_Pred']['data'],times=data['MAG_Pred']['time'],label='Pred',ls='--')
        plt.legend()
        plt.grid()
    except KeyError:
        pass
    plt.figure(figsize=(10, 6))
    plt.title("orientation")
    est_ori = np.rad2deg(quaternion_to_euler(data['Quat']['data']))
    plot_three(est_ori,'Estimated',times=data['Quat']['time'],ls='--')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("error state")
    plot_three(data['errorStateMean']['data'][:,:3],times=data['errorStateMean']['time'],label='pos')
    plot_three(data['errorStateMean']['data'][:,3:6],times=data['errorStateMean']['time'],label='vel')
    plot_three(data['errorStateMean']['data'][:,3:6]*180/3.14,times=data['errorStateMean']['time'],label='rpy')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("error state")
    plot_three(data['errorStateMean']['data'][:,6:9],times=data['errorStateMean']['time'],label='acc bias')
    plot_three(data['errorStateMean']['data'][:,9:12],times=data['errorStateMean']['time'],label='gyro bias')
    plt.legend()
    plt.grid()
    labels = ['Px', 'Py', 'Pz', 'Vx', 'Vy', 'Vz', 
              'Roll', 'Pitch', 'Yaw', 
              'Ba_x', 'Ba_y', 'Ba_z', 'Bg_x', 'Bg_y', 'Bg_z', 
              'Gx', 'Gy', 'Gz']
    # plot_interactive_heatmap(data['K_GPS']['data'].reshape(-1,18,6), data['K_GPS']['time'], x_labels=['Pos N','Pos E','Pos D','Vel N',' Vel E', 'Vel D'],y_labels=labels)
    
    # plot_interactive_heatmap(data['K_MAG']['data'].reshape(-1,3,18), data['K_GPS']['time'], y_labels=['MAGX','MAGY','MAGZ'],x_labels=labels)

    # plt.figure(figsize=(10, 6))
    # plt.title("LL")
    # plt.plot(data['GPS']['time'],data['GPS']['data'][:,0])
    # plt.plot(data['GPS']['time'],data['GPS']['data'][:,1])
    # plt.legend()
    # plt.grid()
    
    plot_covariance_heatmap(data['Cov'])