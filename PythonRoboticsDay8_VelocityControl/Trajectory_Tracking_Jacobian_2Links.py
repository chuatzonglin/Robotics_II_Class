#!/usr/bin/env python3

# Trajectory_Tracking_Jacobian_2Links.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

radius = 20

l_1 = 0.5 # length of link 1 [m]
l_2 = 0.5 # length of link 2 [m]

L   = [l_1, l_2] # link parameters
th  = [0.0*math.pi, 0.2*math.pi] # initial angles
dth = np.array([0.0, 0.0]) # initial angular velocity

X = [0.0, 0.5]# initial position
dx  = np.array([0.0, 0.0]) # initial velocity of end effector

def Jacobian(th, L):

    L1, L2 = L
    Th1, Th2 = th

    J11 = - L1*math.sin(Th1) - L2*math.sin(Th1+Th2)
    J12 = - L2*math.sin(Th1+Th2)
    J21 =   L1*math.cos(Th1) + L2*math.cos(Th1+Th2)
    J22 =   L2*math.cos(Th1+Th2)

    return np.array([[J11, J12],[J21, J22]])

def ForwardKinematics(th, L):

    L1, L2 = L
    Th1, Th2 = th

    x0 = 0.0
    y0 = 0.0

    ##################################################
    # position of tip of link 1
    x1 = L1*math.cos(Th1)
    y1 = L1*math.sin(Th1)

    # position of tip of link 2
    x2 = x1 + L2*math.cos(Th1+Th2)
    y2 = y1 + L2*math.sin(Th1+Th2)
    ##################################################

    X = np.array([[x0, y0],[x1, y1],[x2, y2]])

    return X


dt =  0.1 # delta t
dx =  0.1 # x-velocity
Th0 = [0.35*math.pi, 0.3*math.pi] # initial angles
p0  = ForwardKinematics(Th0, L)  # initial position calculated by FK
p = p0

# parameter for trajectory (linear: y = a_0*x + a_1)
a_1 = -0.8
a_0 = p0[2,1] - a_1*p0[2,0]
a = [a_0, a_1]

Th  = Th0

#animation plot will be ended at the singular potint of manipulator
while -0.01 > l_1*l_2*math.sin(Th[1]) or 0.01 < l_1*l_2*math.sin(Th[1]):

    dX = [dx, dx*a[1]] # definition of the end effector velocity

    # calclation of angular velocity of maniputator joint by using inveese Jacobian
    dTh = np.dot(np.linalg.inv(Jacobian(Th, L)), dX)

    Th = dt*dTh + Th # update the joint angle

    p  = ForwardKinematics(Th, L) # update the manipulator positions by FK

    # plot the graph
    ax = plt.subplot()
    plt.title('Trajectory Tracking Jacobian 2Links')
    plt.axis('equal')
    plt.subplots_adjust(left=0.1, bottom=0.1)
    plt.xlim([-1.3, 1.3])
    plt.ylim([-1.3, 1.3])

    lineT = plt.plot([-2.,2.],[a[1]*(-2.0)+a[0], a[1]*(2.0)+a[0]],'k--',lw=1.0)
    lineX = plt.plot([p[2,0], p[2,0]],[-100, 100], 'r')
    lineY = plt.plot([-100, 100],[p[2,1], p[2,1]], 'r')
    arrow = plt.plot([p[2,0],p[2,0]+dX[0]],[p[2,1],p[2,1]+dX[1]],'r-', lw=7.0)
    graph = plt.plot(p.T[0], p.T[1], '-o', lw=5.0, markersize=radius, mfc='g', mec='g')
    angularvel1 = plt.plot(p[0,0], p[0,1], 'o', markersize=radius*(1.0+dTh[0]), mfc='m', mec='m')
    angularvel2 = plt.plot(p[1,0], p[1,1], 'o', markersize=radius*(1.0+dTh[1]), mfc='c', mec='c')
    text1 = plt.text(0.03, -1.2, 'det|Ja(Th)|={}'.format(l_1*l_2*math.sin(Th[1])))
    c=plt.Circle((0,0),l_1+l_2, edgecolor='k', facecolor='w', LineWidth=2.0)
    ax.add_patch(c)

    plt.grid()
    plt.pause(dt)
    plt.clf()
