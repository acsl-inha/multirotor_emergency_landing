#!/usr/bin/env python
# coding: utf-8

# In[7]:


import numpy as np
import matplotlib.pyplot as plt

Deg2Rad = np.pi/180
Rad2Deg = 1/Deg2Rad

dt = 0.01            # control frequency
tf = 10               # final time
g = 9.8              # m/s^2

l = 1.0              # m
m = 23.56            # kg
Ix = 2.4           # kg m^2
Iy = 2.4           # kg m^2
Iz = 4.5          # kg m^2

# rotor characteristics
kf = 1.1             # N / V^2
kt = 0.52            # N m / V^2 
rotor_limit = 10                  # rotor limit
rotor_sr_limit = rotor_limit/0.1  # rotor slew rate limit
omega_rotor, zeta_rotor = 40, 0.8

t_fail = 3.5
t_detect = 0.2
r_fail = 5

c = l                 # chord length
nRotors = 8
loc = np.zeros((nRotors,3))
# rotor 0-7 position
# 0,2,4,6: positive yaw rotation
# 1,3,5,7: negative yaw rotation
loc[0] = np.array([ 0.25*c,  2.5*c, 0]) 
loc[1] = np.array([ 0.25*c,  1.5*c, 0])
loc[2] = np.array([ 0.25*c, -1.5*c, 0])
loc[3] = np.array([ 0.25*c, -2.5*c, 0])
loc[4] = np.array([-0.75*c, -2.5*c, 0])
loc[5] = np.array([-0.75*c, -1.5*c, 0])
loc[6] = np.array([-0.75*c,  1.5*c, 0])
loc[7] = np.array([-0.75*c,  2.5*c, 0])
thrust = np.array([0, 0, -kf])
moments = np.cross(loc, thrust)

plt.figure(num=1, figsize=(6,6), dpi=100)
plt.broken_barh([(-0.5, 1.0)], (-3.0,  5.0), alpha = 0.2) # fuselage
plt.broken_barh([(-3.0, 6.0)], (-0.75, 1.0), alpha = 0.2) # wing
plt.broken_barh([(-1.2, 2.4)], (-3.0,  1.0), alpha = 0.2) # tail
plt.plot(loc[0::2,1]/c, loc[0::2,0]/c, 'o', markersize=6, label='CW rotation')
plt.plot(loc[1::2,1]/c, loc[1::2,0]/c, 'o', markersize=6, label='CCW rotation')
plt.plot(loc[0::2,1]/c, loc[0::2,0]/c, 'o', alpha=0.1, markersize=30)
plt.plot(loc[1::2,1]/c, loc[1::2,0]/c, 'o', alpha=0.1, markersize=30)

for i in range(nRotors) :
    plt.text(loc[i,1]+0.2, loc[i,0]/c+0.2, 'R%i' %i)
    
plt.xlabel(r'$y_B$ (chord)')
plt.ylabel(r'$x_B$ (chord)')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()  

A = np.zeros((4,nRotors))        # rotor speed (sq) to control (thr/torque)
A[0,:] = np.ones(nRotors)*kf     # thrust
A[[1,2],:] = moments[:,[0,1]].T  # roll moment , pitch moment
A[3,0::2] = -kt                  # yaw moment (CW rotors) 
A[3,1::2] =  kt                  # yaw moment (CCW rotors)

print('location:\n',loc)
print('moments:\n', moments)
print('A:\n',A)


# In[15]:


pinvA = np.linalg.pinv(A)

# quadrotor 6DOF dynamics
def state_derivative(state, t, rotor_cmd, flag_fail):  # computes state derivatives  
    u,v,w,p,q,r,x,y,z,phi,the,psi = state[:12]         # state variables
    rotor = state[12:12+nRotors]         
    rotor_dot = state[12+nRotors:]  
    rotor = np.clip(rotor, 0, rotor_limit)                           # rotor limit
    rotor_dot = np.clip(rotor_dot, -rotor_sr_limit, rotor_sr_limit)  # rotor speed slew rate limit
    if flag_fail and t > t_fail:
        state[12+r_fail] = 0
        rotor[r_fail] = 0
        rotor_dot[r_fail] = 0
    T,L,M,N = A@(rotor**2)                           # control variable
 
    s_phi, c_phi = np.sin(phi), np.cos(phi)
    s_the, c_the = np.sin(the), np.cos(the)
    s_psi, c_psi = np.sin(psi), np.cos(psi)
    
    u_dot   = -g*s_the       - (q*w - r*v)
    v_dot   =  g*c_the*s_phi - (r*u - p*w)
    w_dot   =  g*c_the*c_phi - (p*v - q*u) - T/m
    if z>0:
        u_dot -= -g*s_the
        v_dot -=  g*c_the*s_phi
        w_dot -=  g*c_the*c_phi
    
    p_dot   = -(Iz-Iy)/Ix*q*r + L/Ix 
    q_dot   = -(Ix-Iz)/Iy*r*p + M/Iy
    r_dot   = -(Iy-Ix)/Iz*p*q + N/Iz
    
    Cbn = np.array([         [c_psi*c_the, c_psi*s_the*s_phi-s_psi*c_phi, c_psi*s_the*c_phi+s_psi*s_phi],         [s_psi*c_the, s_psi*s_the*s_phi+c_psi*c_phi, s_psi*s_the*c_phi-c_psi*s_phi],         [     -s_the,                   c_the*s_phi,                   c_the*c_phi]     ])
    
    x_dot,y_dot,z_dot = np.dot(Cbn,np.array([u,v,w]))
    
    phi_dot = p + s_phi*s_the/c_the*q + c_phi*s_the/c_the*r
    the_dot =                 c_phi*q -             s_phi*r
    psi_dot =           s_phi/c_the*q +       c_phi/c_the*r
    
    rotor_ddot = -2*zeta_rotor*omega_rotor*rotor_dot                   + omega_rotor**2*(rotor_cmd-rotor)

    return np.array([u_dot,v_dot,w_dot,p_dot,q_dot,r_dot,x_dot,y_dot,z_dot,                      phi_dot,the_dot,psi_dot,*rotor_dot,*rotor_ddot])

# control computation
def compute_control(state, t):
    u,v,w,p,q,r,x,y,z,phi,the,psi = state[:12]  # state variables
    rotor = state[12:12+nRotors]
    rotor_dot = state[12+nRotors:]
    
    s_phi, c_phi = np.sin(phi), np.cos(phi)
    s_the, c_the = np.sin(the), np.cos(the)
    s_psi, c_psi = np.sin(psi), np.cos(psi)
    
    Cbn = np.array([         [c_psi*c_the, c_psi*s_the*s_phi-s_psi*c_phi, c_psi*s_the*c_phi+s_psi*s_phi],         [s_psi*c_the, s_psi*s_the*s_phi+c_psi*c_phi, s_psi*s_the*c_phi-c_psi*s_phi],         [     -s_the,                   c_the*s_phi,                   c_the*c_phi]     ])

    x_dot,y_dot,z_dot = np.dot(Cbn,[u,v,w])
    h_dot = -z_dot
    h = -z
    
    if t<0.3*tf:
        h_cmd = 1
        x_cmd = 0.6
        y_cmd = 0.8
        psi_cmd = 5*Deg2Rad
    #elif t<0.6*tf:
     #   h_cmd = 1
      #  x_cmd = 1.0
       # y_cmd = 2.0
        #psi_cmd = 5*Deg2Rad
    else:
        h_cmd = 0
        x_cmd = 1.0
        y_cmd = 2.0
        psi_cmd = 0*Deg2Rad
    
    omg_alt, zet_alt = 2, 0.8
    omg_pos, zet_pos = 2, 0.8
    omg_phi, zet_phi = 8, 0.7
    omg_the, zet_the = 8, 0.7
    omg_psi, zet_psi = 4, 0.9
    
    # altitude loop
    up_cmd = -2*zet_alt*omg_alt*h_dot + omg_alt**2*(h_cmd-h) + g
    thrust = m*up_cmd/(c_phi*c_the)

    # horizontal position loop
    ax_cmd = -2*zet_pos*omg_pos*x_dot + omg_pos**2*(x_cmd-x)
    ay_cmd = -2*zet_pos*omg_pos*y_dot + omg_pos**2*(y_cmd-y)
    au_cmd, av_cmd, aw_cmd = np.dot(Cbn.T,[ax_cmd,ay_cmd,0])
    the_cmd = np.arctan2(-au_cmd,g)
    phi_cmd = np.arctan2( av_cmd,g)
        
    # attitude loop
    torque_phi = -2*zet_phi*omg_phi*p + omg_phi**2*(phi_cmd-phi)
    torque_the = -2*zet_the*omg_the*q + omg_the**2*(the_cmd-the)
    torque_psi = -2*zet_psi*omg_psi*r + omg_psi**2*(psi_cmd-psi)
    torque_phi *= Ix
    torque_the *= Iy
    torque_psi *= Iz
    #print('torque_phi',torque_phi)
    #print('torque_the',torque_the)

    return np.array([thrust,torque_phi,torque_the,torque_psi])

def mix_actuators(control):
    rotor = pinvA@control
    return np.sqrt(np.clip(rotor,0,100))

def mix_actuators_cvx(control, r_p, k, flag_fail):
    import cvxpy as cp
    W = np.diag([1, 1, 1, 1])
    
    r = cp.Variable(nRotors,nonneg=True)
    obj_req = cp.sum_squares(W@(A@r-control))
    obj_smooth = 1e-4*cp.sum_squares(r-r_p)
    obj_card = 0.1*cp.norm(r,1)
    obj_max = 0.01*cp.max(r)
    obj_energy = 1e-6*cp.sum_squares(r)
    obj = cp.Minimize( obj_req + obj_smooth + obj_card )
    constr = [ r <= 100 ]
    if flag_fail and k*dt > t_fail + t_detect:
        constr += [r[r_fail]==0]
    cp.Problem(obj, constr).solve(verbose=False)
    rotor = r.value
    return np.sqrt(np.clip(rotor,0,100))


# In[16]:


u0, v0, w0 = 0, 0, 0
p0, q0, r0 = 0, 0, 0
x0, y0, z0 = 0, 0, 0
phi0, the0, psi0 = 0, 0, 0
rotor0 = np.zeros(nRotors)
rotor_dot0 = np.zeros(nRotors)
X0 = np.array([u0,v0,w0,p0,q0,r0,x0,y0,z0,phi0,the0,psi0,*rotor0,*rotor_dot0])
U0 = np.zeros(4)
R0 = np.zeros(nRotors)
t = np.arange(0, tf, dt)
n = len(t)
X = np.zeros((len(X0),n))
U = np.zeros((len(U0),n-1))
R = np.zeros((len(R0),n-1))
X[:,0] = X0
U[:,0] = U0
R[:,0] = R0

dotX_p = X0*0
import time
Tt = np.zeros(n-1)
for k in range(n-1):
    start = time.time()
    U[:,k] = compute_control(X[:,k],t[k])
    if k==0:
        R_p = np.zeros(nRotors)
    else:
        R_p = R[:,k-1]**2
    if k%100==0:
        R[:,k] = mix_actuators_cvx(U[:,k],R_p, k, True)    # fail sim
        dotX = state_derivative(X[:,k],t[k],R[:,k], True)  # fail sim
        X[:,k+1] = X[:,k] + 0.5*(3*dotX-dotX_p)*dt
        dotX_p = dotX
        
X_ca1, U_ca1, R_ca1 = X, U, R
Rr_ca1 = X_ca1[12:12+nRotors,:-1]
Ur_ca1 = A@(Rr_ca1**2)


# In[10]:


print(Tt)
plt.figure(figsize=(10,10), dpi=100)
plt.plot(Tt)


# In[17]:


u0, v0, w0 = 0, 0, 0
p0, q0, r0 = 0, 0, 0
x0, y0, z0 = 0, 0, 0
phi0, the0, psi0 = 0, 0, 0
rotor0 = np.zeros(nRotors)
rotor_dot0 = np.zeros(nRotors)
X0 = np.array([u0,v0,w0,p0,q0,r0,x0,y0,z0,phi0,the0,psi0,*rotor0,*rotor_dot0])
U0 = np.zeros(4)
R0 = np.zeros(nRotors)
t = np.arange(0, tf, dt)
n = len(t)
X = np.zeros((len(X0),n))
U = np.zeros((len(U0),n-1))
R = np.zeros((len(R0),n-1))
X[:,0] = X0
U[:,0] = U0
R[:,0] = R0

dotX_p = X0*0
import time
Tt = np.zeros(n-1)
for k in range(n-1):
    start = time.time()
    U[:,k] = compute_control(X[:,k],t[k])
    if k==0:
        R_p = np.zeros(nRotors)
    else:
        R_p = R[:,k-1]    
    if k%100==0:
        print(k, end=' ')
    R[:,k] = mix_actuators_cvx(U[:,k],R_p, k, False)    # no fail sim
    print(R)
    dotX = state_derivative(X[:,k],t[k],R[:,k], False)  # no fail sim
    X[:,k+1] = X[:,k] + 0.5*(3*dotX-dotX_p)*dt
    dotX_p = dotX
    Tt[k]=time.time()-start
    
X_ca2, U_ca2, R_ca2 = X, U, R
Rr_ca2 = X_ca2[12:12+nRotors,:-1]
Ur_ca2 = A@(Rr_ca2**2)


# In[12]:


print(Tt)
plt.figure(figsize=(10,10), dpi=100)
plt.plot(Tt)


# In[13]:


cmap = plt.get_cmap("tab20")
colors = cmap([0,2,4,6,8,10,12,14])

plt.figure(num=2, figsize=(8,6), dpi=100)
plt.subplot(211)
plt.plot(t[:-1], Ur_ca1[0,:], color=colors[0])
plt.plot(t[:-1], Ur_ca2[0,:], '--', color=colors[0])
plt.axvline(t_fail, linestyle=':', color='r')
plt.ylabel('Thrust (N)')
plt.grid()
plt.subplot(212)
for i in range(3):
    plt.plot(t[:-1], Ur_ca1[i+1,:], color=colors[i])
for i in range(3):
    plt.plot(t[:-1], Ur_ca2[i+1,:], '--', color=colors[i])
plt.axvline(t_fail, linestyle=':',  color='r')
plt.xlabel('Time (s)')
plt.ylabel('Torque (Nm)')
plt.legend([r'$\tau_{\phi}$',r'$\tau_{\theta}$',r'$\tau_{\psi}$'])
plt.grid()
plt.show()


# In[14]:


plt.figure(num=3, figsize=(8,6), dpi=100)
for i in range(nRotors):
    plt.plot(t[:-1], (Rr_ca1[i,:]), color=colors[i], label=rf'$R_{i+1}$')
    plt.plot(t[:-1], (Rr_ca2[i,:]), '--', color=colors[i])
plt.axvline(t_fail, linestyle=':',  color='r')
plt.ylabel('Rotor speed')
plt.xlabel('Time (s)')
plt.legend()
#plt.xlim(0,1)
plt.grid()
plt.show()

plt.figure(num=4, figsize=(8,12), dpi=100)
plt.title('Rotor speed')
for i in range(nRotors):
    plt.subplot(nRotors, 1, i+1)
    plt.plot(t[:-1], (Rr_ca1[i,:]), color=colors[i], label=rf'$R_{i+1}$')
    plt.plot(t[:-1], (Rr_ca2[i,:]), '--', color=colors[i])
    plt.axvline(t_fail, linestyle=':',  color='r')
    plt.ylim(0,10)
    plt.legend()
    plt.grid()
plt.show()

plt.figure(num=5, figsize=(48,6), dpi=100)
plt.title('Rotor voltage (V)')
for i in range(nRotors):
    plt.subplot(1, nRotors, i+1)
    plt.plot(t[:-1], (Rr_ca1[i,:]), color=colors[i], label=rf'$R_{i+1}$')
    plt.plot(t[:-1], (Rr_ca2[i,:]), '--', color=colors[i])
    plt.axvline(t_fail, linestyle=':',  color='r')
    plt.ylim(0,10)
    plt.legend()
    plt.grid()
plt.show()


# In[ ]:


plt.figure(num=6, figsize=(8,6), dpi=100)
plt.subplot(211)
for i in range(3):
    plt.plot(t, X_ca1[i,:], color=colors[i])
for i in range(3):
    plt.plot(t, X_ca2[i,:], '--', color=colors[i])
plt.axvline(t_fail, linestyle=':',  color='r')
plt.ylabel('Body velocity (m/s)')
#plt.ylim(-1.6,1.6)
plt.legend([r'$u$',r'$v$',r'$w$'])
plt.grid()
plt.subplot(212)
for i in range(3):
    plt.plot(t, X_ca1[i+6,:], color=colors[i])
for i in range(3):
    plt.plot(t, X_ca2[i+6,:], '--', color=colors[i])
plt.axvline(t_fail, linestyle=':',  color='r')
plt.xlabel('Time (s)')
plt.ylabel('NED position (m)')
#plt.ylim(-1.1,0.9)
plt.legend([r'$N$',r'$E$',r'$D$'])
plt.grid()
plt.show()

plt.figure(num=7, figsize=(8,6), dpi=100)
plt.subplot(211)
for i in range(3):
    plt.plot(t, X_ca1[i+3,:]*Rad2Deg, color=colors[i])
for i in range(3):
    plt.plot(t, X_ca2[i+3,:]*Rad2Deg, '--', color=colors[i])
plt.axvline(t_fail, linestyle=':',  color='r')
plt.ylabel('Body rate (deg/s)')
#plt.ylim(-150,150)
plt.legend([r'$p$',r'$q$',r'$r$'])
plt.grid()
plt.subplot(212)
for i in range(3):
    plt.plot(t, X_ca1[i+9,:]*Rad2Deg, color=colors[i])
for i in range(3):
    plt.plot(t, X_ca2[i+9,:]*Rad2Deg, '--', color=colors[i])
plt.axvline(t_fail, linestyle=':',  color='r')
plt.xlabel('Time (s)')
plt.ylabel('Euler angle (deg)')
#plt.ylim(-25,25)
plt.legend([r'$\phi$',r'$\theta$',r'$\psi$'])
plt.grid()
plt.show()


# In[ ]:


from mpl_toolkits.mplot3d import Axes3D

plt.figure(figsize=(8,6), dpi=100)
plt.gca(projection='3d')
plt.plot(X_ca1[7,:], X_ca1[6,:], -X_ca1[8,:], color=colors[0], alpha=1.0, linewidth=2, label='Fault')
plt.plot(X_ca2[7,:], X_ca2[6,:], -X_ca2[8,:], '--', color=colors[0], alpha=1.0, linewidth=2, label='Normal')
plt.plot(X_ca2[7,[int(t_fail/dt),int(t_fail/dt)]], X_ca2[6,[int(t_fail/dt),int(t_fail/dt)]], -X_ca2[8,[int(t_fail/dt),int(t_fail/dt)]], 'r*', markersize=10)
plt.xlabel('East (m)')
plt.ylabel('North (m)')
#plt.zlabel('h (m)')
plt.legend()
plt.show()

plt.figure(figsize=(8,6), dpi=100)
plt.plot(X_ca1[7,:], X_ca1[6,:], color=colors[0], alpha=1.0, linewidth=2)
plt.plot(X_ca2[7,:], X_ca2[6,:], '--', color=colors[0], alpha=1.0, linewidth=2)
plt.xlabel('East (m)')
plt.ylabel('North (m)')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()

plt.figure(figsize=(8,6), dpi=100)
plt.plot(X_ca1[7,:], -X_ca1[8,:], color=colors[0], alpha=1.0, linewidth=2)
plt.plot(X_ca2[7,:], -X_ca2[8,:], '--', color=colors[0], alpha=1.0, linewidth=2)
plt.xlabel('East (m)')
plt.ylabel('Altitude (m)')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()


# In[ ]:




