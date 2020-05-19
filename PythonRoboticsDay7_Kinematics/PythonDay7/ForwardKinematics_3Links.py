#!/usr/bin/env python3

# ForwardKinematics_3Links.py
# The input angles for link 1, 2 and  are modified by sliders for the variable

# cittation: <https://algorithm.joho.info/programming/python/forward-kinematics-simulation/>
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

l_1 = 0.5 # length of link 1 [m]
l_2 = 0.5 # length of link 2 [m]
l_3 = 0.5 # length of link 3 [m]

L = [l_1, l_2, l_3] # link parameters
th = [0.0*math.pi, 0.0*math.pi, 0.0*math.pi] # initial angles

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


    X = np.array([[x0, y0],[x1, y1],[x2, y2],[x3, y3]])

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

def update_th3(slider_val):
    # update the angle of link 3
    th[2] = slider_val

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
plt.title('Forward Kinematics 3Links')
plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.2)
plt.xlim([-1.8, 1.8])
plt.ylim([-1.8, 1.8])

# plot the graph
plt.grid()
graph, = plt.plot(p.T[0], p.T[1]) # *.T means transpose the position vector


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
