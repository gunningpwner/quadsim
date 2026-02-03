import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sig
from SimCore import Quadcopter
from DataLogger import DataLogger
plt.close('all')
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
class BiquadFilter:
    def __init__(self, cutoff_hz, fs):
        """
        Initialize with coefficients [b0, b1, b2] and [a1, a2].
        Note: standard form assumes a0 = 1.0.
        """
        K = np.tan(np.pi * cutoff_hz / fs)
        K2 = K * K
        norm = 1.0 / (1.0 + np.sqrt(2.0) * K + K2)
        
        self.b = [K2 * norm, 2.0 * K2 * norm, K2 * norm]
        self.a = [2.0 * (K2 - 1.0) * norm, (1.0 - np.sqrt(2.0) * K + K2) * norm]
        
        # Delay buffers (z^-1 and z^-2)
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0

    def __call__(self, x):
        # The Difference Equation: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        y = (self.b[0] * x + 
             self.b[1] * self.x1 + 
             self.b[2] * self.x2 - 
             self.a[0] * self.y1 - 
             self.a[1] * self.y2)

        # Update the state (shifting the delay line)
        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y

        return y

class ExcitationGenerator:
    def __init__(self):
        self._time=0
        self.zero_time=50/1000
        self.step_level=.4
        self.step_time=40/1000
        self.ramp_max=.7
        self.ramp_time=50/1000
        self.dur = self.zero_time+self.step_time+self.ramp_time
        self.time_left=self.dur
        
    def __call__(self,dt):
        if self._time<self.zero_time:
            ret = 0
        elif self._time<(self.zero_time+self.step_time):
            ret=self.step_level
        elif self._time<=self.dur:
            ret=self.ramp_max*(self.ramp_time-self._time+self.zero_time+self.step_time)/self.ramp_time
        else:
            ret=0
        self._time+=dt
        self.time_left=self.dur-self._time
        return ret
    def reset(self):
        self._time=0
        self.time_left=self.dur
        
class EstimatorFramework:
    def __init__(self):
        # 1. Setup Ground Truth
        # Note: Initial state size is now 13.
        # Quat [1, 0, 0, 0] is Identity (No rotation)
        x0 = np.zeros(13)
        x0[6] = 1.0 
        # x0[6:10]= euler_to_quaternion(np.deg2rad(15),np.deg2rad(15),0)
        self.quad = Quadcopter(initial_state=x0,use_motors=True)
        self.logger = DataLogger()
        self.time = 0.0
        self.dt_sim = 0.005
        self.rls = RLSEstimator(self.dt_sim,self.logger)
    def run(self, duration_sec: float):
        steps = int(duration_sec / self.dt_sim)
        gen=ExcitationGenerator()
        motor_idx=0
        for _ in range(steps):
            
            u_control=np.zeros(4)
            if motor_idx<4:
                u_control[motor_idx]=gen(self.dt_sim)
                if gen.time_left<=0:
                    gen.reset()
                    motor_idx+=1
            
            true_state,omega = self.quad.step(self.dt_sim, u_control)

            self.rls.update_motors(self.time, u_control, omega)
            
            self.logger.log(self.time, 
                            truth_pos=true_state[:3],truth_vel=true_state[3:6],
                            truth_quat=true_state[6:10],control=u_control,omega=omega)
            self.time += self.dt_sim
        
class RLSEstimator:
    def __init__(self,dt,logger):
        self.motor_params=np.zeros((4,4))
        self.motor_p = np.tile(np.eye(4)[:,:,np.newaxis],(1,1,4))*1000
        
        self._last_omega=np.zeros(4)
        self._dt=dt
        self.logger=logger
        self.base_lambda=np.exp(dt/.2)
        self.RLS_COV_MAX = 1e10
        self.RLS_COV_MIN = 1e-6
    def update_motors(self,t,control_input,omega):
        if not self._last_omega.any():
            self._last_omega=omega
            return
        omega_d=(omega-self._last_omega)/self._dt
        self._last_omega=omega.copy() # Need to update the state!
        
        X = np.vstack([control_input,np.sqrt(control_input),np.ones(4),-omega_d])
        e=omega-np.einsum('ij,ji->i', X.T, self.motor_params) # Fixed dot product alignment
        K = np.zeros((4,4))

        for i in range(4):
            # --- Dynamic Lambda Logic ---
            current_lambda = self.base_lambda
            for row in range(4):
                if self.motor_p[row, row, i] > self.RLS_COV_MAX:
                    # Attempt return to lower P by un-forgetting
                    current_lambda = 1.0 + 0.1 * (1.0 - self.base_lambda) 
                    break
            # ----------------------------

            P=self.motor_p[:,:,i]
            X_sub=X[:,np.newaxis,i]
            
            # Use current_lambda instead of self.lamba
            K[:,i]=P@X_sub@np.linalg.inv(current_lambda+X_sub.T@P@X_sub)[:,0]
            self.motor_p[:,:,i]=(P-K[:,np.newaxis,i]@X_sub.T@P)/current_lambda
            
        self.motor_params=self.motor_params+K*e
        self.logger.log(t,motor_estimates=self.motor_params)
        
if __name__ == "__main__":
    # Initialize Framework

    
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
            
    np.random.seed(1234)
    
    sim = EstimatorFramework()
    
    
    # Run Simulation
    sim.run(duration_sec=1)
    
    # Plotting Results
    data = sim.logger.get_arrays()
    
    plt.close('all')
    
    plt.figure(figsize=(10, 6))
    plt.title("Position")
    plot_three(data['truth_pos'],'Truth',ls='-')
    plt.legend()
    plt.grid()

    
    plt.figure(figsize=(10, 6))
    plt.title("Velocity Estimation")
    plot_three(data['truth_vel'],'Truth',ls='-')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("orientation")
    truth_ori = np.rad2deg(quaternion_to_euler(data['truth_quat'][:,1:5]))
    plot_three(truth_ori,'Truth',times=data['truth_quat'][:,0],ls='-')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("Control Inputs")
    plt.plot(data['control'][:,0],data['control'][:,1:],ls='-')
    plt.twinx()
    plt.plot(data['omega'][:,0],data['omega'][:,1:],ls='--')
    plt.legend()
    plt.grid()
    
    fig,axes=plt.subplots(2,2)
    mot=data['motor_estimates'][:,1:].reshape((-1,4,4))
    titles=['a','b','omega_idle','tau']
    for i in range(4):
        ax=axes[i//2,i%2]
        ax.set_title(titles[i])
        ax.plot(data['motor_estimates'][:,0],mot[:,i,:])
        ax.grid()