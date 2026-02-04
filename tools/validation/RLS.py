import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sig
from SimCore import Quadcopter
from DataLogger import DataLogger
import utils
from Sensors import IMU,SensorConfig
plt.close('all')

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
        self.zero_time=30/1000
        self.step_level=.4
        self.step_time=50/1000
        self.ramp_max=.7
        self.ramp_time=150/1000
        self.wait_time=100/1000
        self.dur = self.zero_time+self.step_time+self.ramp_time
        self.time_left=self.dur+self.wait_time
        
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
        self.time_left=self.dur+self.wait_time-self._time
        return ret
    def reset(self):
        self._time=0
        self.time_left=self.dur+self.wait_time
        
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
        self.dt_sim = 0.0005
        self.rls = RLSEstimator(self.dt_sim,self.logger)
        self.controller=INDI(self.dt_sim)
        self.imu = IMU(SensorConfig(
            update_rate_hz=2000.0,
            noise_std=np.array([0.0]*3 + [0.00]*3), 
            bias=np.array([0.0]*3+[.0]*3),
            name="IMU"
        ))
        
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
                    if motor_idx==4:
                        self.controller.setup(self.rls)
            else:
                u_control=self.controller.run(self.time,self.last_state,self.last_imu,self.last_omega)
            true_state,omega = self.quad.step(self.dt_sim, u_control)
            
            imu_data = self.imu.get_measurement(self.time, true_state, self.quad)
            if imu_data is not None:
                self.logger.log(self.time,imu_acc=imu_data[:3],imu_gyr=imu_data[3:])
                
            if motor_idx<4:
                self.rls.run(self.time, u_control, omega,imu_data)
            
            self.logger.log(self.time, 
                            truth_pos=true_state[:3],truth_vel=true_state[3:6],
                            truth_quat=true_state[6:10],control=u_control,omega=omega)
            self.time += self.dt_sim
            self.last_state=true_state
            self.last_omega=omega
            self.last_imu=imu_data
        
class RLSEstimator:
    def __init__(self,dt,logger):
        self.motor_params=np.zeros((4,4))
        self.motor_p = np.tile(np.eye(4)[:,:,np.newaxis],(1,1,4))*100
        self.B1=np.zeros((4,3))
        self.B2=np.zeros((8,3))
        self.spf_P=np.eye(4)*100
        
        self.rot_P=np.eye(8)*100
        
        self._last_omega=np.zeros(4)
        self._last_omega_d=np.zeros(4)
        self._last_imu=np.zeros(6)
        
        self._dt=dt
        self.logger=logger
        self.base_lambda=np.exp(dt/.2)
        self.RLS_COV_MAX = 1e10
        self.RLS_COV_MIN = 1e-6
    
    def run(self,t,control_input,omega,imu_data):
        
        
        self.update_motors(t, control_input, omega)
        if imu_data is not None:
            self.update_control_eff(t, omega, imu_data)
            self._last_imu_dot = (imu_data-self._last_imu)/self._dt
            self._last_imu=imu_data
            
        omega_d=(omega-self._last_omega)/self._dt
        self._last_omega_d=omega_d
        self._last_omega=omega.copy()
        
        
    def singularRLS(self,parameters, Y,X,P):
        X=X[:,np.newaxis]
        current_lambda = self.base_lambda
        if any(np.diag(P)>self.RLS_COV_MAX):
            current_lambda = 1.0 + 0.1 * (1.0 - self.base_lambda) 
            
        K=P@X/(current_lambda+X.T@P@X)

        P =(P-K@X.T@P)/current_lambda
        e=Y-X.T@parameters
        parameters = parameters+K@e
        return parameters,P
    
    def parallelRLS(self,parameters, Y,X,P):
        X=X[:,np.newaxis]
        current_lambda = self.base_lambda
        if any(np.diag(P)>self.RLS_COV_MAX):
            current_lambda = 1.0 + 0.1 * (1.0 - self.base_lambda) 
            
        K=P@X/(current_lambda+X.T@P@X)

        P =(P-K@X.T@P)/current_lambda
        e=Y.reshape(1, -1)-X.T@parameters
        parameters = parameters+K@e
        return parameters,P
        
    def update_motors(self,t,control_input,omega):
        if not self._last_omega.any():
            return
        omega_d=(omega-self._last_omega)/self._dt
        
        for i in range(4):
            X=np.array([control_input[i],np.sqrt(control_input[i]),1,-omega_d[i]])
            par_out,P_out=self.singularRLS(self.motor_params[:,i], omega[i], X, self.motor_p[:,:,i])
            self.motor_params[:,i]=par_out
            self.motor_p[:,:,i]=P_out


            
        self.logger.log(t,motor_estimates=self.motor_params)
    
    def update_control_eff(self,t,omega,imu_data):
        if not self._last_omega.any():
            return
        if not self._last_omega_d.any():
            return
        if not self._last_imu.any():
            return
        omega_diff=(omega-self._last_omega)
        omega_d=omega_diff/self._dt
        
        
        imu_diff=imu_data-self._last_imu
        
        # X=2*omega*omega_diff
        X=omega**2-self._last_omega**2
        self.B1, self.spf_P = self.parallelRLS(self.B1, imu_diff[:3], X, self.spf_P)
        imu_dot_diff = imu_diff/self._dt-self._last_imu_dot
        omega_d_diff=omega_d-self._last_omega_d
        X=np.hstack([X,omega_d_diff])

        self.B2,self.rot_P = self.parallelRLS(self.B2, imu_dot_diff[3:], X, self.rot_P)
        
        self.logger.log(t,b1=self.B1,b2=self.B2,X=X,imu_diff=imu_diff,imu_dot_diff=imu_dot_diff[3:],
                        omega_diff=omega_diff,omega_d_dif=omega_d_diff)

class INDI:
    def __init__(self,dt):
        self.dt=dt
        self.u_old = np.zeros(4)
        
        self.att_gains = np.array([6.0, 6.0, 6.0]) 
        self.rate_gains = np.array([20.0, 20.0, 20.0]) 
        
        # Limits
        self.max_tilt_rate = np.radians(360.0)
        self.max_yaw_rate = np.radians(200.0)
        self.last_ang=np.zeros(3)
        self.last_omega=np.zeros(4)
    def run(self,t,state,imu_data,omega_0):
        quat=np.array([1,0,0,0])
        ang_acc = self.get_desired_angular_accel(state,quat)
        ang_acc_meas=(imu_data[3:]-self.last_ang)/self.dt
        v_ref = np.array([0,0,0,*ang_acc])
        v0=np.array([0,0,0,*ang_acc_meas])
        
        omega_dot=(omega_0-self.last_omega)/self.dt
        self.last_ang=imu_data[3:]
        self.last_omega=omega_0
        return self.calculate_control_inputs(t, v_ref, v0, omega_0, omega_dot)
        
    def get_desired_angular_accel(self, state, desired_quat):
        """
        Calculates target angular acceleration (alpha) given current state and desired orientation.
        """
        current_quat = state[6:10] 
        current_rates = state[10:13]
        
        # 1. Calculate Attitude Error Quaternion (q_err = q_est* . q_des)
        q_est_inv = utils.quaternion_conjugate(current_quat)
        
        # Ensure shortest path
        if np.dot(current_quat, desired_quat) < 0:
            desired_quat = -desired_quat
            
        q_err = utils.quaternion_product(q_est_inv, desired_quat)
        
        # 2. Decompose Error: Tilt Component
        # Get the direction the body Z-axis should be pointing
        target_z_body = utils.quaternion_get_z_axis(q_err)
        
        tilt_axis_norm = np.hypot(target_z_body[0], target_z_body[1])
        tilt_axis = np.zeros(3)
        
        if tilt_axis_norm > 1e-8:
            inv_norm = 1.0 / tilt_axis_norm
            tilt_axis[0] = -target_z_body[1] * inv_norm
            tilt_axis[1] = target_z_body[0] * inv_norm
        else:
            # Singularity (pointing straight up/down), rotate around X
            tilt_axis[0] = 1.0 if (-target_z_body[1] > 0) else -1.0
            
        tilt_angle = np.arccos(np.clip(target_z_body[2], -1.0, 1.0))
        
        # Rate Setpoint from Tilt (P controller)
        rate_sp = np.zeros(3)
        rate_sp[0] = self.att_gains[0] * tilt_axis[0] * tilt_angle
        rate_sp[1] = self.att_gains[1] * tilt_axis[1] * tilt_angle
        
        # 3. Decompose Error: Yaw Component
        # q_yaw = q_err * q_tilt_inv
        q_tilt_inv = utils.quaternion_from_axis_angle(tilt_axis, -tilt_angle)
        q_yaw = utils.quaternion_product(q_err, q_tilt_inv)
        
        # Extract yaw angle
        q_yaw_w = np.clip(q_yaw[0], -1.0, 1.0)
        yaw_error = 2.0 * np.arccos(q_yaw_w)
        
        # Wrap to [-pi, pi] and fix direction
        if yaw_error > np.pi:
            yaw_error -= 2.0 * np.pi
        if q_yaw[3] < 0:
            yaw_error = -yaw_error
            
        rate_sp[2] = self.att_gains[2] * yaw_error
        
        # 4. Limit Rates
        rate_sp_xy_mag = np.hypot(rate_sp[0], rate_sp[1])
        if rate_sp_xy_mag > self.max_tilt_rate:
            scale = self.max_tilt_rate / rate_sp_xy_mag
            rate_sp[:2] *= scale
            
        rate_sp[2] = np.clip(rate_sp[2], -self.max_yaw_rate, self.max_yaw_rate)
        
        # 5. Calculate Desired Angular Acceleration
        rate_error = rate_sp - current_rates
        alpha_sp = self.rate_gains * rate_error
        
        return alpha_sp
    
    def setup(self,rls):
        self.B1k=np.hstack([rls.B1,rls.B2[:4,:]]).T
        self.B2=np.hstack([np.zeros((4,3)),rls.B2[4:,:]]).T
        a=rls.motor_params[0,:]
        b=rls.motor_params[0,:]
        self.omega_max = a+b
        self.kappa=a/self.omega_max
        self.tau=rls.motor_params[-1,:]
        
        
    def calculate_control_inputs(self,t,v_ref,v0,omega_0,omega_dot0):
        lhs = v_ref-v0+self.B2@omega_dot0
        rhs = self.B1k*self.omega_max**2 + self.B2*self.omega_max**2/(2*omega_0*self.tau)
        delta_u= np.linalg.pinv(rhs)@lhs
        u_cmd = self.u_old+delta_u
        u_cmd=np.clip(u_cmd, 0, 1)
        self.u_old=u_cmd
        
        return np.array([solve_for_delta(u, k) for u,k in zip(u_cmd,self.kappa)])
        
def solve_for_delta(u, k):
    """
    Solves u = [k*delta + (1-k)sqrt(delta)]^2 for delta.
    
    Args:
        u: The input value (e.g., Normalized Thrust)
        k: The linearity parameter (0 < k < 1)
        
    Returns:
        delta: The solution (e.g., PWM command)
    """
    # Handle the edge case where k is 0 (Purely linear in sqrt space)
    if abs(k) < 1e-10:
        # Equation becomes: u = [sqrt(delta)]^2 -> u = delta
        return u

    # 1. Setup the quadratic coefficients for x = sqrt(delta)
    # k*x^2 + (1-k)*x - sqrt(u) = 0
    sqrt_u = np.sqrt(u)
    
    a = k
    b = 1 - k
    c = -sqrt_u
    
    # 2. Solve Quadratic Formula
    discriminant = b**2 - 4*a*c
    
    # We use the positive root because sqrt(delta) cannot be negative
    x = (-b + np.sqrt(discriminant)) / (2 * a)
    
    # 3. Square to get delta
    delta = x**2
    
    return delta
    
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
    sim.run(duration_sec=4)
    
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
    truth_ori = np.rad2deg(utils.quaternion_to_euler(data['truth_quat'][:,1:5]))
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
        
    fig,axes=plt.subplots(2,2)
    plt.suptitle("F2omegadomega")
    mot=data['b1'][:,1:].reshape((-1,4,3))
    for i in range(4):
        ax=axes[i//2,i%2]
        ax.set_title(f"motor {i}")
        ax.plot(data['b1'][:,0],mot[:,i,:])
        ax.grid()
        
    fig,axes=plt.subplots(2,2)
    plt.suptitle("pqr2omegadomega")
    mot=data['b2'][:,1:].reshape((-1,8,3))
    for i in range(4):
        ax=axes[i//2,i%2]
        ax.set_title(f"motor {i}")
        ax.plot(data['b2'][:,0],mot[:,i,:])
        ax.grid()
        
    fig,axes=plt.subplots(2,2)
    plt.suptitle("pqrdomegad")
    mot=data['b2'][:,1:].reshape((-1,8,3))
    for i in range(4):
        ax=axes[i//2,i%2]
        ax.set_title(f"motor {i}")
        ax.plot(data['b2'][:,0],mot[:,4+i,:])
        ax.grid()
    plt.figure(figsize=(10, 6))
    plot_three(data['imu_acc'][:,1:],'IMU',times=data['imu_acc'][:,0],ls='--')
    
    pls=2*data['omega_diff'][:,1:]*data['omega'][7:,1:]
    pls3=data['b1'][:,1:].reshape((-1,4,3))[:,:,2]
    pls4=data['imu_diff'][:,:4]