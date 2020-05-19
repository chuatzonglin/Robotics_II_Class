#!/usr/bin/env python3

# ManupulatorControl_odeint.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>
# modified by 2020 Hayashibe

from scipy.integrate import odeint
import numpy as np
import math

# import original modules
import video_ManupulatorControl as v2m

m_1  = 1.0        # mass 1 [kg]
m_2  = 1.0        # mass 2 [kg]
I_1  = 1.0        # inertia 1 [kg m^2]
I_2  = 1.0        # inertia 2 [kg m^2]
l_g1 = 0.5       # length 1g [m]
l_1  = 1.0       # length 1  [m]
l_g2 = 0.5       # length 2g [m]
g = 9.8          # gravitational accelaration[m/s^2]

params = [m_1, m_2, I_1, I_2, l_g1, l_1, l_g2, g] # parameters

K_p1 = 15.0
K_p2 = 15.0
K_v1 = 7.0
K_v2 = 7.0

K = [K_p1, K_p2, K_v1, K_v2]

theta1_d = 0.1*math.pi
theta2_d = 0.5*math.pi
dtheta1_d = 0.0
dtheta2_d = 0.0

# initial conditions(x0, dx0)
max_t = 20.0 # max_time [s]
dt = 0.01    # dt [s]

Xd1 = [ 0.5,  0.7]
Xd2 = [-0.5,  1.9]
Xd3 = [-1.0,  0.5]
Xd4 = [-1.95, 0.0]

XD = [Xd1, Xd2, Xd3, Xd4]

def Trajectory(t):

    xd = XD[0][0]
    yd = XD[0][1]

    if 2.5 < t < 5.0:
        xd = XD[1][0]
        yd = XD[1][1]
    elif 5.0 < t < 7.5:
        xd = XD[2][0]
        yd = XD[2][1]
    elif 7.5 < t < 10.0:
        xd = XD[3][0]
        yd = XD[3][1]

    return [xd, yd]

def InverseKinematics(X, L):

    L1, L2 = L
    x, y = X

    # anlge of the link 1
    th1 = math.atan2(y, x) - math.acos( (L1*L1+(x*x+y*y)-L2*L2)/(2*L1*math.sqrt(x*x+y*y)) )

    # anlge of the link 2
    th2 = math.pi - math.acos( (L1*L1+L2*L2-(x*x+y*y))/(2*L1*L2) )

    return [th1, th2]

def Control(p, t):
    theta_1, dtheta_1, theta_2, dtheta_2 = p

    G_1 = m_1*g*l_g1*math.cos(theta_1) + m_2*g*( l_1*math.cos(theta_1) + l_g2*math.cos(theta_1+theta_2))
    G_2 = m_2*g*l_g2*math.cos(theta_1+theta_2)
    N_1 = -m_2*l_1*l_g2*dtheta_2*( 2*dtheta_1 + dtheta_2 )*math.sin(theta_2)
    N_2 = m_2*l_1*l_g2*dtheta_1*dtheta_1*math.sin(theta_2)

    M_11 = I_1 + I_2 + m_1*l_g1*l_g1 + m_2*( l_1*l_1 + l_g2*l_g2 + 2*l_1*l_g2*math.cos(theta_2) )
    M_12 = I_2 + m_2*(l_g2*l_g2+l_1*l_g2*math.cos(theta_2))
    M_21 = I_2 + m_2*(l_g2*l_g2+l_1*l_g2*math.cos(theta_2))
    M_22 = I_2 + m_2*l_g2*l_g2

    M = np.matrix([[M_11, M_12],[M_21, M_22]])
    N = np.matrix([[N_1],[N_2]])
    G = np.matrix([[G_1],[G_2]])

    #theta1_d = 0.3*math.pi*t
    #theta2_d = math.pi*math.sin(t)

    L  = [l_1, l_g2*2]
    Xd = Trajectory(t)
    [theta1_d, theta2_d] = InverseKinematics(Xd, L)
    
    Feedb = np.matrix([[K_p1*(theta1_d-theta_1) + K_v1*(dtheta1_d-dtheta_1)],[K_p2*(theta2_d-theta_2) + K_v2*(dtheta2_d-dtheta_2)]])
    
    Tau = M.dot(Feedb)+N+G
    #F_1 = K_p1*(theta1_d-theta_1) + K_v1*(dtheta1_d-dtheta_1) + G_1 + N_1
    #F_2 = K_p2*(theta2_d-theta_2) + K_v2*(dtheta2_d-dtheta_2) + G_2 + N_2
    F_1 = Tau[0,0]
    F_2 = Tau[1,0]

    if t > 10.0:
        F_1 = 0
        F_2 = 0

    return [F_1, F_2]

def Manipulator(p, t):
    theta_1, dtheta_1, theta_2, dtheta_2 = p

    M_11 = I_1 + I_2 + m_1*l_g1*l_g1 + m_2*( l_1*l_1 + l_g2*l_g2 + 2*l_1*l_g2*math.cos(theta_2) )
    M_12 = I_2 + m_2*(l_g2*l_g2+l_1*l_g2*math.cos(theta_2))
    M_21 = I_2 + m_2*(l_g2*l_g2+l_1*l_g2*math.cos(theta_2))
    M_22 = I_2 + m_2*l_g2*l_g2

    N_1 = -m_2*l_1*l_g2*dtheta_2*( 2*dtheta_1 + dtheta_2 )*math.sin(theta_2)
    N_2 = m_2*l_1*l_g2*dtheta_1*dtheta_1*math.sin(theta_2)

    G_1 = m_1*g*l_g1*math.cos(theta_1) + m_2*g*( l_1*math.cos(theta_1) + l_g2*math.cos(theta_1+theta_2))
    G_2 = m_2*g*l_g2*math.cos(theta_1+theta_2)

    C = Control(p, t)

    #define matrix
    M = np.matrix([[M_11, M_12],[M_21, M_22]])
    N = np.matrix([[N_1],[N_2]])
    G = np.matrix([[G_1],[G_2]])
    F = np.matrix([[C[0]],[C[1]]])

    IM = np.linalg.inv(M) # calc Inverse matrix
    A = (-1)*IM.dot(N+G-F) # F is right hand side of equations

    ddtheta_1, ddtheta_2 = A

    return [dtheta_1, ddtheta_1, dtheta_2, ddtheta_2]


t = np.arange(0.0, max_t, dt)
x0 = [0.1*math.pi, 0.0, 0.1*math.pi, 0.0]
p = odeint(Manipulator, x0, t)

v2m.video(p, dt, max_t, params, K, XD)