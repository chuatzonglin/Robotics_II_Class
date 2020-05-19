#!/usr/bin/env python3

# InverseKinematics_2Links_Exc2.py

# cittation: <https://algorithm.joho.info/programming/python/forward-kinematics-simulation/>
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

L = 0.5 # length of link [m]
P = [0.25, 0.0*math.pi] # initial length and angles (l, theta)
X = [0.5, 0.5] # initial position of tip of manipulator

#Edited
#P = [0.25, 0.0*math.pi, 0.0*math.pi] # initial length and angles (l, theta, Phi)

def InverseKinematics(X, L):

    x, y = X

    # anlge of the link
    if x == 0:
        th = np.inf
    elif x < 0:
        th = np.pi + np.arctan(y/x)
    else:
        th = np.arctan(y/x)

    # length of the link
    l = (np.sqrt(x*x + y*y) - L)

    return [l, th]

#Edited
def ForwardKinematics(P, L):

    l, Th = P
    #Edited
    #l, Th, Phi = P

    x0 = 0.0
    y0 = 0.0

    ##################################################
    # position of tip of link 1
    x1 = x0 + l * np.cos(Th)
    y1 = y0 + l * np.sin(Th)

    x2 = x1 + L * np.cos(Th)
    y2 = y1 + L * np.sin(Th)
    ##################################################

    return np.array([[x0, y0],[x1, y1],[x2, y2]])


"""
def ForwardKinematics(P, L):

    l, Th = P

    x0 = 0.0
    y0 = 0.0

    ##################################################
    # position of tip of link 1
    x1 =
    y1 =

    x2 =
    y2 =
    ##################################################

    return np.array([[x0, y0],[x1, y1],[x2, y2]])

#Edited
def ForwardKinematics(P, L):

    #l, Th = P
    #Edited
    l, Th, Phi = P

    x0 = 0.0
    y0 = 0.0

    ##################################################
    # position of tip of link 1
    x1 = x0 + l * np.cos(Th)
    y1 = y0 + l * np.sin(Th)

    x2 = x1 + L * np.cos(Th + Phi)
    y2 = y1 + L * np.sin(Th + Phi)
    ##################################################

    return np.array([[x0, y0],[x1, y1],[x2, y2]])
"""

def update_x(slider_val):
    # update the positon of link tip
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
plt.title('Inverse Kinematics 2Links (Ex2)')
plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-1.5, 1.5])
plt.ylim([-1.5, 1.5])

# plot the graph
plt.grid()
lineX, = plt.plot([p[2,0], p[2,0]],[-100, 100], 'r')
lineY, = plt.plot([-100, 100],[p[2,1], p[2,1]], 'r')
graph, = plt.plot(p.T[0], p.T[1]) # *.T means transpose the position vector


# position of sliders
slider1_pos = plt.axes([0.1, 0.05, 0.8, 0.03])
slider2_pos = plt.axes([0.1, 0.01, 0.8, 0.03])

# make the instance for slider objects
threshold_slider1 = Slider(slider1_pos, 'x', -1.0, 1.0, 0.5)
threshold_slider2 = Slider(slider2_pos, 'y', -.5, 0.5, 0.5)

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
