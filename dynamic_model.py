import sympy as sp
from sympy.physics.mechanics import dynamicsymbols, init_vprinting
from sympy import Quaternion
sp.init_session()
from sympy import *
init_vprinting()

p = symbols('p_x p_y p_z')
v = symbols('v_x v_y v_z')
a = symbols ('a_x a_y a_z')
theta = symbols('theta_w theta_x theta_y theta_z')
omega = symbols('omega_w omega_x omega_y omega_z')
alpha = symbols('alpha_w alpha_x alpha_y alpha_z')

dt = symbols('dt')

state=[*p,*v,*theta,*omega,]


ori = Quaternion(*theta,norm=1)
body_rates = Quaternion(*omega,norm=1)
angular_acc = Quaternion(*alpha,norm=1)

pmat=Matrix(p)
vmat=Matrix(v)
amat=Matrix(a)

# position, velocity are all in ecef
# acceleration measured in body frame.
p_pred = pmat+vmat*dt
# rotate acceleration to ecef
a_rot=(ori*Quaternion(0,*a)*ori.inverse()).to_Matrix(vector_only=True)
#need to normalize a_rot and p_mat in code, but for jacobian, omit
v_pred = -9.8*pmat.normalized()+a_rot


