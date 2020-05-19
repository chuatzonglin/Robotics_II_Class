#!/usr/bin/env python3

# InverseKinematics_2Links_Exc1.py

# cittation: <https://algorithm.joho.info/programming/python/forward-kinematics-simulation/>
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

l_1 = 0.5 # length of link 1 [m]
l_2 = 0.5 # length of link 2 [m]

y_0 = 0.5

L = [l_1, l_2] # link parameters
P = [0.0, 0.0*math.pi] # initial length and angles (l, theta)
X = [0.5, 0.5] # initial potision of tip of manipulator

def InverseKinematics(X, L):

    L1, L2 = L
    x, y = X

    # anlge of the link 2
    th =

    # length of the link 1
    l  = 

    return [l, th]


def ForwardKinematics(P, L):

    L1, L2 = L
    l, Th = P

    x0 = 0.0
    y0 = y_0

    ##################################################
    # position of tip of link 1
    x1 =
    y1 =

    # position of tip of link 2
    x2 =
    y2 =
    ##################################################

    return np.array([[x0, y0],[x1, y1],[x2, y2]])

def update_x(slider_val):
    # update the x-position of link tip
    X[0] = slider_val

    # calculation of inverse kinematics
    P = InverseKinematics(X, L)

    # calculation of forward kinematics
    p = ForwardKinematics(P, L)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    graph.set_linestyle('-')
    graph.set_linewidth(5)
    graph.set_marker('o')
    graph.set_markerfacecolor('g')
    graph.set_markeredgecolor('g')
    graph.set_markersize(15)

    # update the target position
    lineX.set_data([p[2,0], p[2,0]],[-100, 100])
    lineY.set_data([-100, 100],[p[2,1], p[2,1]])

    # re-draw of the links
    fig.canvas.draw_idle()

def update_y(slider_val):
    # update the y-position of link tip
    X[1] = slider_val

    # calculation of inverse kinematics
    P = InverseKinematics(X, L)

    # calculation of forward kinematics
    p = ForwardKinematics(P, L)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    graph.set_linestyle('-')
    graph.set_linewidth(5)
    graph.set_marker('o')
    graph.set_markerfacecolor('g')
    graph.set_markeredgecolor('g')
    graph.set_markersize(15)

    # update the target position
    lineX.set_data([p[2,0], p[2,0]],[-100, 100])
    lineY.set_data([-100, 100],[p[2,1], p[2,1]])

    # re-draw of the links
    fig.canvas.draw_idle()


P = InverseKinematics(X, L)
p = ForwardKinematics(P, L)


fig, ax = plt.subplots()
plt.title('Inverse Kinematics 2Links (Ex1)')
plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-1.0, 1.0])
plt.ylim([-0.3, 1.3])

# plot the graph
plt.grid()
lineX, = plt.plot([p[2,0], p[2,0]],[-100, 100], 'r')
lineY, = plt.plot([-100, 100],[p[2,1], p[2,1]], 'r')
graph, = plt.plot(p.T[0], p.T[1]) # *.T means transpose the position vector

# position of sliders
slider1_pos = plt.axes([0.1, 0.05, 0.8, 0.03])
slider2_pos = plt.axes([0.1, 0.01, 0.8, 0.03])

# make the instance for slider objects
threshold_slider1 = Slider(slider1_pos, 'x',-1.0, 1.0, 0.0)
threshold_slider2 = Slider(slider2_pos, 'y', 0.0, 1.0, 0.5)

# update the graph when the slider positions are modified
threshold_slider1.on_changed(update_x)
threshold_slider2.on_changed(update_y)
graph.set_linestyle('-')
graph.set_linewidth(5)
graph.set_marker('o')
graph.set_markerfacecolor('g')
graph.set_markeredgecolor('g')
graph.set_markersize(15)


plt.grid()
plt.show()
