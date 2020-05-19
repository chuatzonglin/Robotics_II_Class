#!/usr/bin/env python3

# ForwardKinematics_2Links.py
# The input angles for link 1 and 2 are modified by sliders for the variable

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

def ForwardKinematics(th, L):

    L1, L2 = L
    Th1, Th2 = th

    x0 = 0.0
    y0 = 0.0

    ##################################################
    # position of tip of link 1
    x1 = L1*np.cos(Th1)
    y1 = L1*np.sin(Th1)

    # position of tip of link 2
    x2 = L1*np.cos(Th1) + L2*np.cos(Th1+Th2)
    y2 = L1*np.sin(Th1) + L2*np.sin(Th1+Th2)
    ##################################################

    X = np.array([[x0, y0],[x1, y1],[x2, y2]])

    return X

def update_th1(slider_val):
    # update the angle of link 1
    th[0] = slider_val

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

    # re-draw of the links
    fig.canvas.draw_idle()

def update_th2(slider_val):
    # update the angle of link 2
    th[1] = slider_val

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

    # re-draw of the links
    fig.canvas.draw_idle()


p = ForwardKinematics(th, L)

fig, ax = plt.subplots()
plt.title('Forward Kinematics_2Links')
plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-1.3, 1.3])
plt.ylim([-1.3, 1.3])

# plot the graph
plt.grid()
graph, = plt.plot(p.T[0], p.T[1]) # *.T means transpose the position vector


# position of sliders
slider1_pos = plt.axes([0.1, 0.05, 0.8, 0.03])
slider2_pos = plt.axes([0.1, 0.01, 0.8, 0.03])

# make the instance for slider objects
threshold_slider1 = Slider(slider1_pos, 'th1', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider2 = Slider(slider2_pos, 'th2', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)

# update the graph when the slider positions are modified
threshold_slider1.on_changed(update_th1)
threshold_slider2.on_changed(update_th2)
graph.set_linestyle('-')
graph.set_linewidth(5)
graph.set_marker('o')
graph.set_markerfacecolor('g')
graph.set_markeredgecolor('g')
graph.set_markersize(15)
plt.grid()
plt.show()
