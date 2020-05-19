#!/usr/bin/env python3

# LinearControl_odeint.py
# Copyright (c) 2017 Dai Owaki <owaki@tohoku.ac.jp>
# Revised 2019 by Dai Owaki

from scipy.integrate import odeint
import numpy as np
import sys

# import original modules
import video_LinearControl as vlc

m = 1.0     # mass [kg]

bc = 0.3    # coulomb friction
kp = 2 #gain for P control
kv = 2 #gain for D control
"""bc = float(sys.argv[3])    # coulomb friction
kp = float(sys.argv[1]) #gain for P control
kv = float(sys.argv[2]) #gain for D control"""

params = [m, bc, kp, kv] # parameters

def Controller(p):
    x, dx = p
    e = 0
    return (kp*(e-x)-kv*dx)

def LinearControl(p, t):

    x, dx = p

    ddx = (1.0/m) * (- bc*np.sign(dx) + Controller(p))

    return [dx,ddx]

# initial conditions(x0, dx0)
max_t = 10.0 # max_time [s]
dt = 0.01    # dt [s]

t = np.arange(0.0, max_t, dt) # time seeies 0.0 to max_t (with dt intervals)
x0 = [2.0, 0.0]                 # initial variables x0=0.5, x1=1.0
p = odeint(LinearControl, x0, t)   # ode calculation

vlc.video(p, dt, max_t, params)
