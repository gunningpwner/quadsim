# -*- coding: utf-8 -*-
"""
Created on Fri Jan  9 16:57:09 2026

@author: gunni
"""

import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation  

plt.close('all')
# df = pd.read_csv(r'C:\Users\gunni\Desktop\quadsim\data\20250109\\magnetometer_data1.csv')
df = pd.read_csv(r'C:\Users\gunni\Desktop\quadsim\data\flight_log_part2.csv')
# df[['d0','d1','d2']].to_csv(r'C:/Users/gunni/Downloads/h.txt',sep=' ',index=False,header=False)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

bias = np.array([-0.034516, 0.242338, 0.100965])
Ainv = np.array([ 1.037407, -0.000972,  0.004468,
                 -0.000972,  1.057293, -0.014778,
                  0.004468, -0.014778,  1.006760]).reshape((3,3))



# p=ax.scatter(df['d0'],df['d1'],df['d2'],c=df['timestamp'],cmap='jet')



corrected = np.einsum('ij,jk->ik',Ainv,df[['d0','d1','d2']].values.T-bias[:,np.newaxis])

# p=ax.scatter(corrected[0,:],corrected[1,:],corrected[2,:],c=df['timestamp'],cmap='jet',marker='s')


tilt=np.deg2rad(35)
tilt_mat = Rotation.from_euler('XYZ',[0,tilt,0])
tilt_corr= tilt_mat.apply(corrected.T).T
p=ax.scatter(tilt_corr[0,:],tilt_corr[1,:],tilt_corr[2,:],c=df['timestamp'],cmap='jet',marker='v')

# # ref mag ned is .22363,-.01614,.43474
ref_mag=np.array([.22363,-.01614,.43474])
plt.plot([0,ref_mag[0]],[0,ref_mag[1]],[0,ref_mag[2]])



rot_mats = np.zeros((50,3))

rot_mats[:,2]=np.linspace(0,2*np.pi,50)

rots=Rotation.from_euler('XYZ',rot_mats)

compass_sweep=rots.apply(ref_mag)
ax.scatter(compass_sweep[:,1],compass_sweep[:,0],compass_sweep[:,2])

ax.set_aspect('equal')
plt.xlabel("F")
plt.ylabel("R")
ax.set_zlabel("D")


# test=tilt_mat.apply(df[['d0','d1','d2']].values)

# # df[['d0','d1','d2']].to_csv(r'C:/Users/gunni/Downloads/h.txt',sep=' ',index=False,header=False)