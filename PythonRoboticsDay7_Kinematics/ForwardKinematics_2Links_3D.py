#!/usr/bin/env python3

# ForwardKinematics_2Links_3D.py
# The input angles for link 1 and 2 are modified by sliders for the variable

# cittation: <https://algorithm.joho.info/programming/python/forward-kinematics-simulation/>
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D

l_1 = 0.5 # length of link 1 [m]
l_2 = 0.5 # length of link 2 [m]

L = [l_1, l_2] # link parameters
th = [0.0*math.pi, 0.0*math.pi, 0.0*math.pi] # initial angles

def ForwardKinematics(th, L):

    L1, L2 = L
    Th1, Th2, Th3 = th

    x0 = 0.0
    y0 = 0.0
    z0 = 0.0

    ##################################################
    # position of tip of link 1
    x1 = L1*math.cos(Th2)*math.cos(Th1)
    y1 = L1*math.cos(Th2)*math.sin(Th1)
    z1 = L1*math.sin(Th2)

    # position of tip of link 2
    x2 = x1 + L2*math.cos(Th2+Th3)*math.cos(Th1)
    y2 = y1 + L2*math.cos(Th2+Th3)*math.sin(Th1)
    z2 = z1 + L2*math.sin(Th2+Th3)
    ##################################################

    X = np.array([[x0, y0, z0],[x1, y1, z1],[x2, y2, z2]])

    return X

def update_th1(slider_val):
    # update the angle of link 1 yaw
    th[0] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # update the potision of each point
    #graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    graph.set_data(p[:,0], p[:,1])
    graph.set_3d_properties(p[:,2])
    graph.set_linestyle('-')
    graph.set_linewidth(5)
    graph.set_marker('o')
    graph.set_markerfacecolor('g')
    graph.set_markeredgecolor('g')
    graph.set_markersize(15)

    # re-draw of the links
    fig.canvas.draw_idle()

def update_th2(slider_val):
    # update the angle of link 1 pitch
    th[1] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # update the potision of each point
    #graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    graph.set_data(p[:,0], p[:,1])
    graph.set_3d_properties(p[:,2])
    graph.set_linestyle('-')
    graph.set_linewidth(5)
    graph.set_marker('o')
    graph.set_markerfacecolor('g')
    graph.set_markeredgecolor('g')
    graph.set_markersize(15)

    # re-draw of the links
    fig.canvas.draw_idle()

def update_th3(slider_val):
    # update the angle of link 2
    th[2] = slider_val

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # update the potision of each point
    #graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    graph.set_data(p[:,0], p[:,1])
    graph.set_3d_properties(p[:,2])
    graph.set_linestyle('-')
    graph.set_linewidth(5)
    graph.set_marker('o')
    graph.set_markerfacecolor('g')
    graph.set_markeredgecolor('g')
    graph.set_markersize(15)

    # re-draw of the links
    fig.canvas.draw_idle()


p = ForwardKinematics(th, L)

#fig, ax = plt.subplots()
fig = plt.figure()
plt.title('Forward Kinematics 2Links in 3D')
ax = fig.add_subplot(111, projection='3d')

plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-1.3, 1.3])
plt.ylim([-1.3, 1.3])

# plot the graph
plt.grid()
#graph, = ax.plot(p.T[0], p.T[1]) # *.T means transpose the position vector
graph, = ax.plot(p[:,0], p[:,1], p[:,2])
ax.set_zlim([0, 1.0])


# position of sliders
slider1_pos = plt.axes([0.1, 0.09, 0.8, 0.03])
slider2_pos = plt.axes([0.1, 0.05, 0.8, 0.03])
slider3_pos = plt.axes([0.1, 0.01, 0.8, 0.03])

# make the instance for slider objects
threshold_slider1 = Slider(slider1_pos, 'th1', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider2 = Slider(slider2_pos, 'th2', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider3 = Slider(slider3_pos, 'th3', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)

# update the graph when the slider positions are modified
threshold_slider1.on_changed(update_th1)
threshold_slider2.on_changed(update_th2)
threshold_slider3.on_changed(update_th3)
graph.set_linestyle('-')
graph.set_linewidth(5)
graph.set_marker('o')
graph.set_markerfacecolor('g')
graph.set_markeredgecolor('g')
graph.set_markersize(15)
plt.grid()
plt.show()
