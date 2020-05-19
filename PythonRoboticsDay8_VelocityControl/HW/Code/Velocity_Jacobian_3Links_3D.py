#!/usr/bin/env python3

# Velocity_Jacobian_2Links_3D.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D

radius = 15

l_0 = 0.2

l_1 = 0.5 # length of link 1 [m]
l_2 = 0.5 # length of link 2 [m]

L = [l_0, l_1, l_2] # link parameters
th = [0.0*math.pi, 0.0*math.pi, 0.0*math.pi] # initial angles
dth = np.array([0.0, 0.0, 0.0]) # initial angular velocity


def Jacobian(th, L):

    L0, L1, L2 = L
    Th1, Th2, Th3 = th

    J11 = (- L1*math.cos(Th2)     - L2*math.cos(Th2+Th3)) * math.sin(Th1)
    J12 = (- L1*math.sin(Th2)     - L2*math.sin(Th2+Th3)) * math.cos(Th1)
    J13 =  - L2*math.sin(Th2+Th3)*math.cos(Th1)
    J21 = (  L1*math.cos(Th2)     + L2*math.cos(Th2+Th3)) * math.cos(Th1)
    J22 = (- L1*math.sin(Th2)     - L2*math.sin(Th2+Th3)) * math.sin(Th1)
    J23 =  - L2*math.sin(Th2+Th3)*math.sin(Th1)
    J31 =   0
    J32 =   L1*math.cos(Th2) + L2*math.cos(Th2+Th3)
    J33 =   L2*math.cos(Th2+Th3)

    return np.array([[J11, J12, J13],[J21, J22, J23],[J31, J32, J33]])


def ForwardKinematics(th, L):

    L0, L1, L2 = L
    Th1, Th2, Th3 = th

    x0 = 0.0
    y0 = 0.0
    z0 = 0.0

    x1 = 0.0
    y1 = 0.0
    z1 = L0

    ##################################################
    # position of tip of link 1
    x2 = L1*math.cos(Th2)*math.cos(Th1)
    y2 = L1*math.cos(Th2)*math.sin(Th1)
    z2 = L1*math.sin(Th2) + z1

    # position of tip of link 2
    x3 = x2 + L2*math.cos(Th2+Th3)*math.cos(Th1)
    y3 = y2 + L2*math.cos(Th2+Th3)*math.sin(Th1)
    z3 = z2 + L2*math.sin(Th2+Th3)
    ##################################################

    X = np.array([[x0, y0, z0],[x1, y1, z1],[x2, y2, z2],[x3, y3, z3]])

    return X

def update_th1(slider_val):
    # update the angle of link 1 yaw
    th[0] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    arrow.set_3d_properties([p[3,2],p[3,2]+dx[2]])

    graph.set_data(p[:,0], p[:,1])
    graph.set_3d_properties(p[:,2])

    angularvel1.set_data(p[0,0], p[0,1])
    angularvel1.set_3d_properties(p[0,2])
    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_data(p[1,0], p[1,1])
    angularvel2.set_3d_properties(p[1,2])
    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_data(p[2,0], p[2,1])
    angularvel3.set_3d_properties(p[2,2])
    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()

def update_th2(slider_val):
    # update the angle of link 1 pitch
    th[1] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    arrow.set_3d_properties([p[3,2],p[3,2]+dx[2]])

    graph.set_data(p[:,0], p[:,1])
    graph.set_3d_properties(p[:,2])

    angularvel1.set_data(p[0,0], p[0,1])
    angularvel1.set_3d_properties(p[0,2])
    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_data(p[1,0], p[1,1])
    angularvel2.set_3d_properties(p[1,2])
    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_data(p[2,0], p[2,1])
    angularvel3.set_3d_properties(p[2,2])
    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()

def update_th3(slider_val):
    # update the angle of link 2
    th[2] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    arrow.set_3d_properties([p[3,2],p[3,2]+dx[2]])

    graph.set_data(p[:,0], p[:,1])
    graph.set_3d_properties(p[:,2])

    angularvel1.set_data(p[0,0], p[0,1])
    angularvel1.set_3d_properties(p[0,2])
    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_data(p[1,0], p[1,1])
    angularvel2.set_3d_properties(p[1,2])
    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_data(p[2,0], p[2,1])
    angularvel3.set_3d_properties(p[2,2])
    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()


def update_dth1(slider_val):
    # update the angle of link 1 yaw
    dth[0] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    arrow.set_3d_properties([p[3,2],p[3,2]+dx[2]])

    graph.set_data(p[:,0], p[:,1])
    graph.set_3d_properties(p[:,2])

    angularvel1.set_data(p[0,0], p[0,1])
    angularvel1.set_3d_properties(p[0,2])
    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_data(p[1,0], p[1,1])
    angularvel2.set_3d_properties(p[1,2])
    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_data(p[2,0], p[2,1])
    angularvel3.set_3d_properties(p[2,2])
    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()

def update_dth2(slider_val):
    # update the angle of link 1 pitch
    dth[1] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    arrow.set_3d_properties([p[3,2],p[3,2]+dx[2]])

    graph.set_data(p[:,0], p[:,1])
    graph.set_3d_properties(p[:,2])

    angularvel1.set_data(p[0,0], p[0,1])
    angularvel1.set_3d_properties(p[0,2])
    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_data(p[1,0], p[1,1])
    angularvel2.set_3d_properties(p[1,2])
    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_data(p[2,0], p[2,1])
    angularvel3.set_3d_properties(p[2,2])
    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()

def update_dth3(slider_val):
    # update the angle of link 2
    dth[2] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of Jacobian
    dx = np.dot(Jacobian(th, L), dth)

    # update the potision of each point
    arrow.set_data([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]])
    arrow.set_3d_properties([p[3,2],p[3,2]+dx[2]])

    graph.set_data(p[:,0], p[:,1])
    graph.set_3d_properties(p[:,2])

    angularvel1.set_data(p[0,0], p[0,1])
    angularvel1.set_3d_properties(p[0,2])
    angularvel1.set_marker('o')
    angularvel1.set_markersize(radius*(1.0+dth[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')

    angularvel2.set_data(p[1,0], p[1,1])
    angularvel2.set_3d_properties(p[1,2])
    angularvel2.set_marker('o')
    angularvel2.set_markersize(radius*(1.0+dth[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    angularvel3.set_data(p[2,0], p[2,1])
    angularvel3.set_3d_properties(p[2,2])
    angularvel3.set_marker('o')
    angularvel3.set_markersize(radius*(1.0+dth[2]))
    angularvel3.set_markerfacecolor('y')
    angularvel3.set_markeredgecolor('y')

    # re-draw of the links
    fig.canvas.draw_idle()


p = ForwardKinematics(th, L)
dx = np.dot(Jacobian(th, L), dth)

#fig, ax = plt.subplots()
fig = plt.figure()
plt.title('Velocity Jacobian 2Links in 3D')
ax = fig.add_subplot(111, projection='3d')

plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-1.3, 1.3])
plt.ylim([-1.3, 1.3])

# plot the graph
plt.grid()
arrow, = ax.plot([p[3,0],p[3,0]+dx[0]],[p[3,1],p[3,1]+dx[1]],[p[3,2],p[3,2]+dx[2]], 'r-', lw=7)
graph, = ax.plot(p[:,0], p[:,1], p[:,2])
angularvel1, = ax.plot([p[0,0]], [p[0,1]], [p[0,2]], 'o')
angularvel2, = ax.plot([p[1,0]], [p[1,1]], [p[1,2]], 'o')
angularvel3, = ax.plot([p[2,0]], [p[2,1]], [p[2,2]], 'o')
ax.set_zlim([0, 1.0])


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

angularvel1.set_markersize(radius*(1.0+dth[0]))
angularvel1.set_markerfacecolor('m')
angularvel1.set_markeredgecolor('m')
angularvel2.set_markersize(radius*(1.0+dth[1]))
angularvel2.set_markerfacecolor('c')
angularvel2.set_markeredgecolor('c')
angularvel3.set_markersize(radius*(1.0+dth[2]))
angularvel3.set_markerfacecolor('y')
angularvel3.set_markeredgecolor('y')

plt.grid()
plt.show()
