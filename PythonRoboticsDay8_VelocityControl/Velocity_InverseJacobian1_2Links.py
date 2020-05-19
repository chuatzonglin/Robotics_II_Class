#!/usr/bin/env python3

# Veolocity_InverseJacobian1_2Links.py
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
dx  = np.array([0.0, 0.0]) # initial velocity of end effector

def Jacobian(th, L):

    L1, L2 = L
    Th1, Th2 = th

    J11 = - L1*math.sin(Th1) - L2*math.sin(Th1+Th2)
    J12 = - L2*math.sin(Th1+Th2)
    J21 =   L1*math.cos(Th1) + L2*math.cos(Th1+Th2)
    J22 =   L2*math.cos(Th1+Th2)

    return np.array([[J11, J12],[J21, J22]])

def InverseJacobian(th, L):

    L1, L2 = L
    Th1, Th2 = th

    #Singular point
    if -0.001 < L1*L2*math.sin(Th2) < 0:
        detJ = -0.001
        print('Singular point!')
    elif 0 < L1*L2*math.sin(Th2) < 0.001:
        detJ =  0.001
        print('Singular point!')
    else:
        detJ = L1*L2*math.sin(Th2)

    IJ11 = ( 1/detJ )*(  L2*math.cos(Th1+Th2) )
    IJ12 = ( 1/detJ )*(  L2*math.sin(Th1+Th2) )
    IJ21 = ( 1/detJ )*( -L1*math.cos(Th1) - L2*math.cos(Th1+Th2) )
    IJ22 = ( 1/detJ )*( -L1*math.sin(Th1) - L2*math.sin(Th1+Th2) )

    return np.array([[IJ11, IJ12],[IJ21, IJ22]])


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

def update_th1(slider_val):
    # update the angle of link 1
    th[0] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of InverseJacobian
    dth = np.dot(InverseJacobian(th, L), dx)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[2,0],p[2,0]+dx[0]],[p[2,1],p[2,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])

    angularvel1.set_marker('o')
    angularvel2.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel1.set_markerfacecolor('m')
    angularvel2.set_markerfacecolor('c')
    angularvel1.set_markeredgecolor('m')
    angularvel2.set_markeredgecolor('c')

    text1.set_text('det|Ja(Th)|={}'.format(l_1*l_2*math.sin(th[1])))

    # re-draw of the links
    fig.canvas.draw_idle()

def update_th2(slider_val):
    # update the angle of link 2
    th[1] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of InverseJacobian
    dth = np.dot(InverseJacobian(th, L), dx)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[2,0],p[2,0]+dx[0]],[p[2,1],p[2,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])

    angularvel1.set_marker('o')
    angularvel2.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel1.set_markerfacecolor('m')
    angularvel2.set_markerfacecolor('c')
    angularvel1.set_markeredgecolor('m')
    angularvel2.set_markeredgecolor('c')

    text1.set_text('det|Ja(Th)|={}'.format(l_1*l_2*math.sin(th[1])))

    # re-draw of the links
    fig.canvas.draw_idle()

def update_dx(slider_val):
    # update the angular vel of link 1
    dx[0] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of InverseJacobian
    dth = np.dot(InverseJacobian(th, L), dx)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[2,0],p[2,0]+dx[0]],[p[2,1],p[2,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])

    angularvel1.set_marker('o')
    angularvel2.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel1.set_markerfacecolor('m')
    angularvel2.set_markerfacecolor('c')
    angularvel1.set_markeredgecolor('m')
    angularvel2.set_markeredgecolor('c')

    text1.set_text('det|Ja(Th)|={}'.format(l_1*l_2*math.sin(th[1])))

    # re-draw of the links
    fig.canvas.draw_idle()

def update_dy(slider_val):
    # update the anglular vel of link 2
    dx[1] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of InverseJacobian
    dth = np.dot(InverseJacobian(th, L), dx)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[2,0],p[2,0]+dx[0]],[p[2,1],p[2,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])

    angularvel1.set_marker('o')
    angularvel2.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel1.set_markerfacecolor('m')
    angularvel2.set_markerfacecolor('c')
    angularvel1.set_markeredgecolor('m')
    angularvel2.set_markeredgecolor('c')

    text1.set_text('det|Ja(Th)|={}'.format(l_1*l_2*math.sin(th[1])))

    # re-draw of the links
    fig.canvas.draw_idle()


p  = ForwardKinematics(th, L)
#dx = np.dot(Jacobian(th, L), dth)
dth = np.dot(InverseJacobian(th, L), dx)

fig, ax = plt.subplots()
plt.title('Velocity InverseJacobian #1 2Links')
plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-1.3, 1.3])
plt.ylim([-1.3, 1.3])

# plot the graph
plt.grid()
arrow, = plt.plot([p[2,0],p[2,0]+dx[0]],[p[2,1],p[2,1]+dx[1]],'r-', lw=7)
graph, = plt.plot(p.T[0], p.T[1]) # *.T means transpose the position vector
angularvel1, = plt.plot(p[0,0], p[0,1])
angularvel2, = plt.plot(p[1,0], p[1,1])
text1 = plt.text(0.03, -0.7, 'det|Ja(Th)|={}'.format(l_1*l_2*math.sin(th[1])))

# position of sliders
slider1_pos = plt.axes([0.05, 0.05, 0.35, 0.03])
slider2_pos = plt.axes([0.05, 0.01, 0.35, 0.03])
slider3_pos = plt.axes([0.55, 0.05, 0.35, 0.03])
slider4_pos = plt.axes([0.55, 0.01, 0.35, 0.03])

# make the instance for slider objects
threshold_slider1 = Slider(slider1_pos, 'th1',  -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider2 = Slider(slider2_pos, 'th2',  -1.0*math.pi, 1.0*math.pi, 0.2*math.pi)
threshold_slider3 = Slider(slider3_pos, 'dx', -1.0, 1.0, 0.0)
threshold_slider4 = Slider(slider4_pos, 'dy', -1.0, 1.0, 0.0)

# update the graph when the slider positions are modified
threshold_slider1.on_changed(update_th1)
threshold_slider2.on_changed(update_th2)
threshold_slider3.on_changed(update_dx)
threshold_slider4.on_changed(update_dy)

graph.set_linestyle('-')
graph.set_linewidth(5)
graph.set_marker('o')
graph.set_markerfacecolor('g')
graph.set_markeredgecolor('g')
graph.set_markersize(15)

angularvel1.set_marker('o')
angularvel2.set_marker('o')
angularvel1.set_markersize(radius*(1.0+dth[0]))
angularvel2.set_markersize(radius*(1.0+dth[1]))
angularvel1.set_markerfacecolor('m')
angularvel2.set_markerfacecolor('c')
angularvel1.set_markeredgecolor('m')
angularvel2.set_markeredgecolor('c')

plt.grid()
plt.show()
