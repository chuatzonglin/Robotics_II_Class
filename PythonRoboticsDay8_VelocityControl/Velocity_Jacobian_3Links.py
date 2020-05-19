#!/usr/bin/env python3

# Veolocity_Jacobian_3Links.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

radius = 20

l_1 = 0.5 # length of link 1 [m]
l_2 = 0.5 # length of link 2 [m]
l_3 = 0.5 # length of link 3 [m]

L = [l_1, l_2, l_3] # link parameters
th = [0.0*math.pi, 0.0*math.pi, 0.0*math.pi] # initial angles
dth = np.array([0.0, 0.0, 0.0]) # initial angular velocity

def Jacobian(th, L):

    L1, L2, L3 = L
    Th1, Th2, Th3 = th

    J11 = - L1*math.sin(Th1)     - L2*math.sin(Th2+Th3)     - L3*math.sin(Th1+Th2+Th3)
    J12 = - L2*math.sin(Th2+Th3) - L3*math.sin(Th1+Th2+Th3)
    J13 = - L3*math.sin(Th1+Th2+Th3)
    J21 =   L1*math.cos(Th1)     + L2*math.cos(Th2+Th3)     + L3*math.cos(Th1+Th2+Th3)
    J22 =   L2*math.cos(Th2+Th3) + L3*math.cos(Th1+Th2+Th3)
    J23 =   L3*math.cos(Th1+Th2+Th3)

    return np.array([[J11, J12, J13],[J21, J22, J23]])

def ForwardKinematics(th, L):

    L1, L2, L3 = L
    Th1, Th2, Th3 = th

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
    ##################################################

    return np.array([[x0, y0],[x1, y1],[x2, y2],[x3, y3]])

def update_th1(slider_val):
    # update the angle of link 1
    th[0] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])
    angularvel3.set_data(p[2,0], p[2,1])

    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')


    # re-draw of the links
    fig.canvas.draw_idle()

def update_th2(slider_val):
    # update the angle of link 2
    th[1] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])
    angularvel3.set_data(p[2,0], p[2,1])

    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()

def update_th3(slider_val):
    # update the angle of link 3
    th[2] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])
    angularvel3.set_data(p[2,0], p[2,1])

    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()


def update_dth1(slider_val):
    # update the angular vel of link 1
    dth[0] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])
    angularvel3.set_data(p[2,0], p[2,1])

    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()

def update_dth2(slider_val):
    # update the anglular vel of link 2
    dth[1] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)
    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])
    angularvel3.set_data(p[2,0], p[2,1])

    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()

def update_dth3(slider_val):
    # update the anglular vel of link 2
    dth[2] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)
    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])
    angularvel3.set_data(p[2,0], p[2,1])

    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()


p  = ForwardKinematics(th, L)
dx = np.dot(Jacobian(th, L), dth)

fig, ax = plt.subplots()
plt.title('Velocity Jacobian 3Links')
plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-2.3, 2.3])
plt.ylim([-2.3, 2.3])

# plot the graph
plt.grid()
arrow, = plt.plot([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]],'r-', lw=7)
graph, = plt.plot(p.T[0], p.T[1]) # *.T means transpose the position vector
angularvel1, = plt.plot(p[0,0], p[0,1])
angularvel2, = plt.plot(p[1,0], p[1,1])
angularvel3, = plt.plot(p[2,0], p[2,1])

# position of sliders
slider1_pos = plt.axes([0.05, 0.07, 0.35, 0.02])
slider2_pos = plt.axes([0.05, 0.04, 0.35, 0.02])
slider3_pos = plt.axes([0.05, 0.01, 0.35, 0.02])
slider4_pos = plt.axes([0.55, 0.07, 0.35, 0.02])
slider5_pos = plt.axes([0.55, 0.04, 0.35, 0.02])
slider6_pos = plt.axes([0.55, 0.01, 0.35, 0.02])

# make the instance for slider objects
threshold_slider1 = Slider(slider1_pos, 'th1',  -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider2 = Slider(slider2_pos, 'th2',  -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider3 = Slider(slider3_pos, 'th3',  -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider4 = Slider(slider4_pos, 'dth1', -1.0, 1.0, 0.0)
threshold_slider5 = Slider(slider5_pos, 'dth2', -1.0, 1.0, 0.0)
threshold_slider6 = Slider(slider6_pos, 'dth3', -1.0, 1.0, 0.0)

# update the graph when the slider positions are modified
threshold_slider1.on_changed(update_th1)
threshold_slider2.on_changed(update_th2)
threshold_slider3.on_changed(update_th3)
threshold_slider4.on_changed(update_dth1)
threshold_slider5.on_changed(update_dth2)
threshold_slider6.on_changed(update_dth3)

graph.set_linestyle('-')
graph.set_linewidth(5)
graph.set_marker('o')
graph.set_markerfacecolor('g')
graph.set_markeredgecolor('g')
graph.set_markersize(radius)

angularvel1.set_marker('o')
angularvel1.set_markersize(radius*(1.0+dth[0]))
angularvel1.set_markerfacecolor('m')
angularvel1.set_markeredgecolor('m')

angularvel2.set_marker('o')
angularvel2.set_markersize(radius*(1.0+dth[1]))
angularvel2.set_markerfacecolor('c')
angularvel2.set_markeredgecolor('c')

angularvel3.set_marker('o')
angularvel3.set_markersize(radius*(1.0+dth[2]))
angularvel3.set_markerfacecolor('y')
angularvel3.set_markeredgecolor('y')


plt.grid()
plt.show()
