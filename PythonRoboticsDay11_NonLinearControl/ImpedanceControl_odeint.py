#!/usr/bin/env python3

# ImpedanceControl_odeint.py
# Copyright (c) 2017 Dai Owaki <owaki@tohoku.ac.jp>
# Revised 2019 by Dai Owaki

from scipy.integrate import odeint
import numpy as np
import sys

# import original modules
import video_ImpedanceControl as vic

m = 2.0 # mass [kg]
c = 0.1 # damper [Ns/m]
k = 1.0 # spring [N/m]

"""md = float(sys.argv[1]) # desired mass
cd = float(sys.argv[2]) # desired damper
kd = float(sys.argv[3]) # desired spring"""
md = float(1) # desired mass
cd = float(4) # desired damper
kd = float(10) # desired spring


params = [m, c, k, md, cd, kd] # parameters

# initial conditions(x0, dx0)
max_t = 10.0 # max_time [s]
dt = 0.01    # dt [s]

def ImpedanceController(p):
    x, dx = p
    ddx = (1.0/m) * ( -c*dx - k*x )

    return ( (m-md)*ddx + (c-cd)*dx + (k-kd)*x )

def SpringMassDamper(p, t):

    x, dx = p

    ddx = (1.0/m) * ( -c*dx - k*x + ImpedanceController(p))

    return [dx,ddx]


t = np.arange(0.0, max_t, dt) # time seeies 0.0 to max_t (with dt intervals)
x0 = [2.0, 0.0]                 # initial variables x0=0.5, x1=1.0
p = odeint(SpringMassDamper, x0, t)   # ode calculation

vic.video(p, dt, max_t, params)
