#!/usr/bin/env python3

# Trajectory_Tracking_Jacobian_5Links.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

radius = 15

link_num = 5

l_1 = 0.5 # length of link 1 [m]
l_2 = 0.5 # length of link 2 [m]
l_3 = 0.5 # length of link 3 [m]
l_4 = 0.5 # length of link 4 [m]
l_5 = 0.5 # length of link 5 [m]

L = [l_1, l_2, l_3, l_4, l_5] # link parameters
th = [0.0*math.pi, 0.0*math.pi, 0.0*math.pi, 0.0*math.pi, 0.0*math.pi]
dth = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # initial angular velocity

X = [0.0, 0.5]# initial position
dx  = np.array([0.0, 0.0]) # initial velocity of end effector

def Jacobian(th, L):

    L1, L2, L3, L4, L5 = L
    Th1, Th2, Th3, Th4, Th5 = th

    J11 = - L1*math.sin(Th1) - L2*math.sin(Th1+Th2) - L3*math.sin(Th1+Th2+Th3) - L4*math.sin(Th1+Th2+Th3+Th4) - L5*math.sin(Th1+Th2+Th3+Th4+Th5)
    J12 = - L2*math.sin(Th1+Th2) - L3*math.sin(Th1+Th2+Th3) - L4*math.sin(Th1+Th2+Th3+Th4) - L5*math.sin(Th1+Th2+Th3+Th4+Th5)
    J13 = - L3*math.sin(Th1+Th2+Th3) - L4*math.sin(Th1+Th2+Th3+Th4) - L5*math.sin(Th1+Th2+Th3+Th4+Th5)
    J14 = - L4*math.sin(Th1+Th2+Th3+Th4) - L5*math.sin(Th1+Th2+Th3+Th4+Th5)
    J15 = - L5*math.sin(Th1+Th2+Th3+Th4+Th5)

    J21 =  L1*math.cos(Th1) + L2*math.cos(Th1+Th2) + L3*math.cos(Th1+Th2+Th3) + L4*math.cos(Th1+Th2+Th3+Th4) + L5*math.cos(Th1+Th2+Th3+Th4+Th5)
    J22 =  L2*math.cos(Th1+Th2) + L3*math.cos(Th1+Th2+Th3) + L4*math.cos(Th1+Th2+Th3+Th4) + L5*math.cos(Th1+Th2+Th3+Th4+Th5)
    J23 =  L3*math.cos(Th1+Th2+Th3) + L4*math.cos(Th1+Th2+Th3+Th4) + L5*math.cos(Th1+Th2+Th3+Th4+Th5)
    J24 =  L4*math.cos(Th1+Th2+Th3+Th4) + L5*math.cos(Th1+Th2+Th3+Th4+Th5)
    J25 =  L5*math.cos(Th1+Th2+Th3+Th4+Th5)

    return np.array([[J11, J12, J13, J14, J15],[J21, J22, J23, J24, J25]])

def ForwardKinematics(th, L):

    L1, L2, L3, L4, L5 = L
    Th1, Th2, Th3, Th4, Th5 = th

    x0 = 0.0
    y0 = 0.0

    ##################################################
    #position of tip of link 1
    x1 = L1*math.cos(Th1)
    y1 = L1*math.sin(Th1)

    # position of tip of link 2
    x2 = x1 + L2*math.cos(Th1+Th2)
    y2 = y1 + L2*math.sin(Th1+Th2)

    # position of tip of link 3
    x3 = x2 + L3*math.cos(Th1+Th2+Th3)
    y3 = y2 + L3*math.sin(Th1+Th2+Th3)

    # position of tip of link 3
    x4 = x3 + L4*math.cos(Th1+Th2+Th3+Th4)
    y4 = y3 + L4*math.sin(Th1+Th2+Th3+Th4)

    # position of tip of link 3
    x5 = x4 + L5*math.cos(Th1+Th2+Th3+Th4+Th5)
    y5 = y4 + L5*math.sin(Th1+Th2+Th3+Th4+Th5)
    ##################################################


    X = np.array([[x0, y0],[x1, y1],[x2, y2],[x3, y3],[x4, y4],[x5, y5]])

    return X


dt =  0.1 # delta t
dx = -0.1 # x-velocity
Th0 = [0.3*math.pi, 0.0*math.pi, 0.1*math.pi, 0.1*math.pi, 0.1*math.pi] # initial angles
p0  = ForwardKinematics(Th0, L)  # initial position calculated by FK
p = p0

# parameter for trajectory (linear: y = a_0*x + a_1)
a_1 = 2.0
a_0 = p0[link_num,1] - a_1*p0[link_num,0]
a = [a_0, a_1]

Th  = Th0

#animation plot will be ended at the singular potint of manipulator
#for i in range(100):
#while -0.01 > l_1*l_2*math.sin(Th[1]) or 0.01 < l_1*l_2*math.sin(Th[1]):
while math.sqrt(p[link_num,0]*p[link_num,0] + p[link_num,1]*p[link_num,1]) < l_1+l_2+l_3+l_4+l_5 - 0.01:

    dX = [dx, dx*a[1]] # definition of the end effector velocity

    # calclation of angular velocity of maniputator joint by using inveese Jacobian
    dTh = np.dot(np.linalg.pinv(Jacobian(Th, L)), dX)

    Th = dt*dTh + Th # update the joint angle

    p  = ForwardKinematics(Th, L) # update the manipulator positions by FK

    # plot the graph
    ax = plt.subplot()
    plt.title('Trajectory Tracking Jacobian 5Links')
    plt.axis('equal')
    plt.subplots_adjust(left=0.1, bottom=0.1)
    plt.xlim([-3., 3.])
    plt.ylim([-3., 3.])

    lineT = plt.plot([-3.,3.],[a[1]*(-3.0)+a[0], a[1]*(3.0)+a[0]],'k--',lw=1.0)
    lineX = plt.plot([p[link_num,0], p[link_num,0]],[-100, 100], 'r')
    lineY = plt.plot([-100, 100],[p[link_num,1], p[link_num,1]], 'r')
    arrow = plt.plot([p[link_num,0],p[link_num,0]+dX[0]],[p[link_num,1],p[link_num,1]+dX[1]],'r-', lw=7.0)
    graph = plt.plot(p.T[0], p.T[1], '-o', lw=5.0, markersize=radius, mfc='g', mec='g')
    angularvel1 = plt.plot(p[0,0], p[0,1], 'o', markersize=radius*(1.0+dTh[0]), mfc='m', mec='m')
    angularvel2 = plt.plot(p[1,0], p[1,1], 'o', markersize=radius*(1.0+dTh[1]), mfc='c', mec='c')
    angularvel3 = plt.plot(p[2,0], p[2,1], 'o', markersize=radius*(1.0+dTh[2]), mfc='y', mec='y')
    angularvel4 = plt.plot(p[3,0], p[3,1], 'o', markersize=radius*(1.0+dTh[3]), mfc='b', mec='b')
    angularvel5 = plt.plot(p[4,0], p[4,1], 'o', markersize=radius*(1.0+dTh[4]), mfc='k', mec='k')

    #text1 = plt.text(0.03, -1.2, 'det|Ja(Th)|={}'.format(l_1*l_2*math.sin(Th[1])))
    c=plt.Circle((0,0),l_1+l_2+l_3+l_4+l_5, edgecolor='k', facecolor='w', LineWidth=2.0)
    ax.add_patch(c)

    plt.grid()
    plt.pause(dt)
    plt.clf()
