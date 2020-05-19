#!/usr/bin/env python3

# ForwardKinematics_2Links_Exc2.py

# cittation: <https://algorithm.joho.info/programming/python/forward-kinematics-simulation/>
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

L = 0.5 # length of link [m]

P = [0.25, 0.0*math.pi] # initial length and angles (l, theta)

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

def update_l(slider_val):
    # update the angle of link 1
    P[0] = slider_val

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

    # re-draw of the links
    fig.canvas.draw_idle()

def update_th(slider_val):
    # update the angle of link 2
    P[1] = slider_val

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

    # re-draw of the links
    fig.canvas.draw_idle()


p = ForwardKinematics(P, L)


fig, ax = plt.subplots()
plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-1.5, 1.5])
plt.ylim([-1.5, 1.5])

# plot the graph
plt.grid()
graph, = plt.plot(p.T[0], p.T[1]) # *.T means transpose the position vector


# position of sliders
slider1_pos = plt.axes([0.1, 0.05, 0.8, 0.03])
slider2_pos = plt.axes([0.1, 0.01, 0.8, 0.03])

# make the instance for slider objects
threshold_slider1 = Slider(slider1_pos, 'l', 0.0, 0.5, 0.25)
threshold_slider2 = Slider(slider2_pos, 'th', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)

# update the graph when the slider positions are modified
threshold_slider1.on_changed(update_l)
threshold_slider2.on_changed(update_th)
graph.set_linestyle('-')
graph.set_linewidth(5)
graph.set_marker('o')
graph.set_markerfacecolor('g')
graph.set_markeredgecolor('g')
graph.set_markersize(15)
plt.grid()
plt.show()
