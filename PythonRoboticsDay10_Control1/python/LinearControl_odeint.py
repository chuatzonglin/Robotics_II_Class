#!/usr/bin/env python3

# LinearControl_odeint.py
# Copyright (c) 2017 Dai Owaki <owaki@tohoku.ac.jp>
# Revised 2019 by Dai Owaki
# Current Version Modified by Kuroba for Robotics II HW 

from scipy.integrate import odeint
import numpy as np

# import original modules
import video_LinearControl as vlc

m = 1.0     # mass [kg]
g = 9.8     # gravitational accelaration[m/s^2]

kp = 5.0 #gain for P control
kv = np.sqrt(4*m*kp) #gain for D control
a =  0.0 #disturbance

params = [m, g, kp, kv, a] # parameters

def Controller(p):
    x, dx = p
    e = 0
    return (kp*(e-x)-kv*dx)

def LinearControl(p, t):

    x, dx = p

    ddx = Controller(p) + a

    return [dx,ddx]

# initial conditions(x0, dx0)
max_t = 10.0 # max_time [s]
dt = 0.01    # dt [s]

t = np.arange(0.0, max_t, dt) # time seeies 0.0 to max_t (with dt intervals)
x0 = [2.0, 0.0]                 # initial variables x0=0.5, x1=1.0
p = odeint(LinearControl, x0, t)   # ode calculation

vlc.video(p, dt, max_t, params)
