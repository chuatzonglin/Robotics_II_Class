#!/usr/bin/env python3

# 2manupulator_odeint.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

from scipy.integrate import odeint
import numpy as np
import math

# import original modules
import video_2manupulator as v2m

m_1  = 1.0        # mass 1 [kg]
m_2  = 1.0        # mass 2 [kg]
I_1  = 1.0        # inertia 1 [kg m^2]
I_2  = 1.0        # inertia 2 [kg m^2]
l_g1 = 0.5       # length 1g [m]
l_1  = 1.0       # length 1  [m]
l_g2 = 0.5       # length 2g [m]
g = 9.8          # gravitational accelaration[m/s^2]

params = [m_1, m_2, I_1, I_2, l_g1, l_1, l_g2, g] # parameters

# initial conditions(x0, dx0)
max_t = 10.0 # max_time [s]
dt = 0.01    # dt [s]

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

    #define matrix
    M = np.matrix([[M_11, M_12],[M_21, M_22]])
    N = np.matrix([[N_1],[N_2]])
    G = np.matrix([[G_1],[G_2]])

    IM = np.linalg.inv(M) # calc Inverse matrix
    A = (-1)*IM.dot(N+G)

    ddtheta_1, ddtheta_2 = A

    return [dtheta_1, ddtheta_1, dtheta_2, ddtheta_2]


t = np.arange(0.0, max_t, dt)
x0 = [0.1*math.pi, 0.0, 0.1*math.pi, 0.0]
p = odeint(Manipulator, x0, t)

v2m.video(p, dt, max_t, params)
