#!/usr/bin/env python3

# InverseKinematics_2Links.py
# The input position for end effector are modified by sliders

# cittation: <https://algorithm.joho.info/programming/python/forward-kinematics-simulation/>
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

l_1 = 0.5 # length of link 1 [m]
l_2 = 0.5 # length of link 2 [m]

L = [l_1, l_2] # link parameters
th = [0.0*math.pi, 0.0*math.pi] # initial angles
P = [0.0, 0.5]

def InverseKinematics(X, L):

    L1, L2 = L
    x, y = X
    L = np.sqrt(np.power(x,2) + np.power(y,2))

    # anlge of the link 1
    th1 = np.arctan2(y,x) - np.arccos((np.power(L1,2) + np.power(L,2) - np.power(L2,2))/(2*L1*L))

    # anlge of the link 2
    th2 = np.pi - np.arccos((np.power(L1,2) + np.power(L2,2) - np.power(L,2))/(2*L1*L2))


    return [th1, th2]

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

    return np.array([[x0, y0],[x1, y1],[x2, y2]])

def update_x(slider_val):
    # update the x-position of  end effector
    P[0] = slider_val

    th = InverseKinematics(P, L)

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    graph.set_linestyle('-')
    graph.set_linewidth(5)
    graph.set_marker('o')
    graph.set_markerfacecolor('g')
    graph.set_markeredgecolor('g')
    graph.set_markersize(15)

    # update the target position
    lineX.set_data([P[0], P[0]],[-100, 100])
    lineY.set_data([-100, 100],[P[1], P[1]])

    # re-draw of the links
    fig.canvas.draw_idle()

def update_y(slider_val):
    # update the y-position of  end effector
    P[1] = slider_val

    th = InverseKinematics(P, L)

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    graph.set_linestyle('-')
    graph.set_linewidth(5)
    graph.set_marker('o')
    graph.set_markerfacecolor('g')
    graph.set_markeredgecolor('g')
    graph.set_markersize(15)

    # update the target position
    lineX.set_data([P[0], P[0]],[-100, 100])
    lineY.set_data([-100, 100],[P[1], P[1]])

    # re-draw of the links
    fig.canvas.draw_idle()

th = InverseKinematics(P, L)
p  = ForwardKinematics(th, L)


fig, ax = plt.subplots()
plt.title('Inverse Kinematics 2Links(first)')
plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-1.1, 1.1])
plt.ylim([-1.1, 1.1])

# plot the graph
plt.grid()
lineX, = plt.plot([p[2,0], p[2,0]],[-100, 100], 'r')
lineY, = plt.plot([-100, 100],[p[2,1], p[2,1]], 'r')
graph, = plt.plot(p.T[0], p.T[1]) # *.T means transpose the position vector
c=plt.Circle((0,0),l_1+l_2, edgecolor='k', facecolor='w', LineWidth=2.0)
ax.add_patch(c)

# position of sliders
slider1_pos = plt.axes([0.1, 0.05, 0.8, 0.03])
slider2_pos = plt.axes([0.1, 0.01, 0.8, 0.03])

# make the instance for slider objects
threshold_slider1 = Slider(slider1_pos, 'x', -1.0, 1.0, 0.0)
threshold_slider2 = Slider(slider2_pos, 'y', -1.0, 1.0, 0.5)

# update the graph when the slider positions are modified
threshold_slider1.on_changed(update_x)
threshold_slider2.on_changed(update_y)
graph.set_linestyle('-')
graph.set_linewidth(5)
graph.set_marker('o')
graph.set_markerfacecolor('g')
graph.set_markeredgecolor('g')
graph.set_markersize(15)
ax.add_patch(c)

plt.grid()
plt.show()
