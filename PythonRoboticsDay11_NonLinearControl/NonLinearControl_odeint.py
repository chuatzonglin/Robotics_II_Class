#!/usr/bin/env python3

# Lyapnov.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

from scipy.integrate import odeint
import numpy as np
import sys

# import original modules
import video_NonLinearControl as vnlc

bc = 0.1   # coulomb friction

kp = 6 #gain for P control
kv = 5 #gain for D control

m = 1.0     # mass [kg]
"""bc = float(sys.argv[3])    # coulomb friction

kp = float(sys.argv[1]) #gain for P control
kv = float(sys.argv[2]) #gain for D control"""

params = [m, bc, kp, kv] # parameters

def ModelBasedController(p):
    x, dx = p
    return (bc*np.sign(dx))

def ServoController(p):
    x, dx = p
    e   = 0
    de  = 0
    dde = 0
    return (dde + kv*(de-dx) + kp*(e-x))

def LinearControl(p, t):

    x, dx = p

    ddx = (1.0/m) * (- bc*np.sign(dx) + m*ServoController(p) + ModelBasedController(p))

    return [dx,ddx]

# initial conditions(x0, dx0)
max_t = 10.0 # max_time [s]
dt = 0.01    # dt [s]

t = np.arange(0.0, max_t, dt) # time seeies 0.0 to max_t (with dt intervals)
x0 = [2.0, 0.0]                 # initial variables x0=0.5, x1=1.0
p = odeint(LinearControl, x0, t)   # ode calculation

vnlc.video(p, dt, max_t, params)
