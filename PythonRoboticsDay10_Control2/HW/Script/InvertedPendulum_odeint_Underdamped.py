#!/usr/bin/env python3

# InvertedPendulum_odeint.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

from scipy.integrate import odeint
import numpy as np
import math
import sys

# import original modules
import video_InvertedPendulum as vip

m_th = 0.10#1.0     # mass theta [kg]
m_x  = 0.50#2.0     # mass x [kg]
I    = 0.01#0.00558389#1.0     # inertia 1 [kg m^2]
l_g  = 0.5     # length of pendulum [m]
g    = 9.80665 # gravitational accelaration[m/s^2]

#K_p = float(sys.argv[1])#-100.0
#K_v = float(sys.argv[2])#-89.6

theta_d = 0.0
dtheta_d = 0.0

params  = [m_th, m_x, I, l_g, g] # parameters

targets = [theta_d, dtheta_d]    # targets

# initial conditions(x0, dx0)
max_t = 5.0 # max_time [s]
dt = 0.01    # dt [s]

alpha = (I*(m_x+m_th))/(m_th*l_g) + m_x*l_g
beta  = (m_x + m_th)*g

K_p = 15
K_v = 1

gains   = [K_p, K_v]             # gains

if K_v*K_v - 4*alpha*(K_p-beta) < 0:
    s1_re = -K_v/(2*alpha)
    s2_re = -K_v/(2*alpha)

    s1_im =  math.sqrt(-(K_v*K_v - 4*alpha*(K_p-beta)))/(2*alpha)
    s2_im = -math.sqrt(-(K_v*K_v - 4*alpha*(K_p-beta)))/(2*alpha)
else:
    s1_re = -K_v/(2*alpha) + math.sqrt(K_v*K_v - 4*alpha*(K_p-beta))/(2*alpha)
    s2_re = -K_v/(2*alpha) - math.sqrt(K_v*K_v - 4*alpha*(K_p-beta))/(2*alpha)

    s1_im = 0.0
    s2_im = 0.0

#S = [math.sqrt(beta/alpha), 0] # Poles of the system (no inputs)
S = [s1_re, s1_im, s2_re, s2_im] # Poles of the system (no inputs)

sqr = 4*alpha*(K_p-beta)
if sqr < 0:
    sqr = -sqr

print('alpha={},beta={}'.format(alpha, beta))
print('K_v^2={}, 4alpha(K_p+beta)={}, sqr={}'.format(K_v*K_v,4*alpha*(K_p-beta), math.sqrt(sqr)))


def Control(p):
    x, dx, theta, dtheta = p

    out = - K_p*(theta_d-theta) - K_v*(dtheta_d-dtheta)

    return out

def InvertedPendulum(p, t):
    x, dx, theta, dtheta = p

    if theta > math.pi:
        theta = theta - 2*math.pi
    elif theta < -math.pi:
        theta = theta + 2*math.pi

    M_11 = m_x + m_th
    M_12 = m_th*l_g*math.cos(theta)
    M_21 = m_th*l_g*math.cos(theta)
    M_22 = I + m_th*l_g*l_g

    #define matrix
    M = np.matrix([[M_11, M_12],[M_21, M_22]])
    N = np.matrix([[-m_th*l_g*math.sin(theta)*dtheta*dtheta],[0]])
    G = np.matrix([[0],[-m_th*g*l_g*math.sin(theta)]])
    F = np.matrix([[Control(p)],[0]])

    IM = np.linalg.inv(M) # calc Inverse matrix
    A = (-1)*IM.dot(N+G-F) # F is right hand side of equations

    ddx, ddtheta = A

    return [dx, ddx, dtheta, ddtheta]


t = np.arange(0.0, max_t, dt)
x0 = [0.0, 0.0, 0.35*math.pi, 0.0]
p = odeint(InvertedPendulum, x0, t)

vip.video(p, dt, max_t, params, gains, targets, S)
