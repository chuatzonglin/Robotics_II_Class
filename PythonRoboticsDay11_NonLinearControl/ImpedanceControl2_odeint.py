#!/usr/bin/env python3

# ImpedanceControl2_odeint.py
# Copyright (c) 2017 Dai Owaki <owaki@tohoku.ac.jp>
# Revised 2019 by Dai Owaki

from scipy.integrate import odeint
import numpy as np
import sys

# import original modules
import video_ImpedanceControl2 as vic2

m = 2.0 # mass [kg]
c = 0.1 # damper [Ns/m]
k = 1.0 # spring [N/m]

md = float(sys.argv[1]) # desired mass
cd = float(sys.argv[2]) # desired damper
kd = float(sys.argv[3]) # desired spring

kv = float(sys.argv[4])
kp = float(sys.argv[5])

params = [m, c, k, md, cd, kd, kv, kp] # parameters

# initial conditions(x0, dx0)
max_t = 10.0 # max_time [s]
dt = 0.01    # dt [s]

def ImpedanceController2(p):
    x, dx = p
    ddx = (1.0/md) * ( -cd*dx - kd*x )

    xd  = dx*dt  + x
    dxd = ddx*dt + dx

    return ( kv*(dxd-dx) + kp*(xd-x) )

def SpringMassDamper(p, t):

    x, dx = p

    ddx = (1.0/m) * ( -c*dx - k*x + ImpedanceController2(p))

    return [dx,ddx]


t = np.arange(0.0, max_t, dt) # time seeies 0.0 to max_t (with dt intervals)
x0 = [2.0, 0.0]                 # initial variables x0=0.5, x1=1.0
p = odeint(SpringMassDamper, x0, t)   # ode calculation

vic2.video(p, dt, max_t, params)
