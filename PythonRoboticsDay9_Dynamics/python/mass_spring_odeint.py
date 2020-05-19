#!/usr/bin/env python3

# mass_spring_odeint.py
# Copyright (c) 2017 Dai Owaki <owaki@tohoku.ac.jp>
# Revised 2019 by Dai Owaki

from scipy.integrate import odeint

# import original modules
import video as v

m = 1.0     # mass [kg]
k = 10.0     # spring constant [N/m]
g = 9.8     # gravitational accelaration[m/s^2]
c = 1.0

params = [m, k, g] # parameters

def MassSpring(p, t):

    x, dx = p

    ddx = ((-k*x-c*dx)/m)

    return [dx,ddx]

# initial conditions(x0, dx0)
max_t = 10.0 # max_time [s]
dt = 0.01    # dt [s]

t = v.np.arange(0.0, max_t, dt) # time seeies 0.0 to max_t (with dt intervals)
x0 = [0.5, 1.0]                 # initial variables x0=0.5, x1=1.0
p = odeint(MassSpring, x0, t)   # ode calculation

v.video(p, dt, max_t, params)
