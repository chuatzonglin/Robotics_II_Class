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
l_3 = 0.5 # length of link 3 [m]
l_4 = 0.5 # length of link 4 [m]

L = [l_1, l_2, l_3, l_4] # link parameters
#th = [0.0*math.pi, 0.0*math.pi, 0.0*math.pi] # initial angles
#Edited
th = [0.0*math.pi, 0.0*math.pi, 0.0*math.pi, 0.0*math.pi, 0.0*math.pi, 0.0*math.pi, 0.0*math.pi, 0.0*math.pi] # initial angles

def ForwardKinematics(th, L):

    L1, L2, L3, L4 = L
    Th1, Th2, Th3, Th4, Th5, Th6, Th7, Th8= th

    x0 = 0.0
    y0 = 0.0
    z0 = 0.0

    ##################################################
    # position of tip of link 1
    x1 = L1*math.cos(Th2)*math.cos(Th1)
    y1 = L1*math.cos(Th2)*math.sin(Th1)
    z1 = L1*math.sin(Th2)

    # position of tip of link 2
    x2 = x1 + L2*math.cos(Th2+Th4)*math.cos(Th1 + Th3)
    y2 = y1 + L2*math.cos(Th2+Th4)*math.sin(Th1 + Th3)
    z2 = z1 + L2*math.sin(Th2+Th4)

    # position of tip of link 3
    x3 = x2 + L3*math.cos(Th2+Th4+Th6)*math.cos(Th1 + Th3 + Th5)
    y3 = y2 + L3*math.cos(Th2+Th4+Th6)*math.sin(Th1 + Th3 + Th5)
    z3 = z2 + L3*math.sin(Th2+Th4+Th6)

    # position of tip of link 3
    x4 = x3 + L4*math.cos(Th2+Th4+Th6+Th8)*math.cos(Th1 + Th3 + Th5 + Th7)
    y4 = y3 + L4*math.cos(Th2+Th4+Th6+Th8)*math.sin(Th1 + Th3 + Th5 + Th7)
    z4 = z3 + L4*math.sin(Th2+Th4+Th6+Th8)
    ##################################################

    X = np.array([[x0, y0, z0],[x1, y1, z1],[x2, y2, z2],[x3, y3, z3], [x4, y4, z4]])

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

def update_th4(slider_val):
    # update the angle of link 2
    th[3] = slider_val

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

def update_th5(slider_val):
    # update the angle of link 3
    th[4] = slider_val

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

def update_th6(slider_val):
    # update the angle of link 3
    th[5] = slider_val

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

def update_th7(slider_val):
    # update the angle of link 3
    th[6] = slider_val

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

def update_th8(slider_val):
    # update the angle of link 3
    th[7] = slider_val

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

def update_th(slider_val):
    # update the angle of link 3
    th[thNumber] = slider_val

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
plt.subplots_adjust(left=0.1, bottom=0.20)
plt.xlim([-2.0, 2.0])
plt.ylim([-2.0, 2.0])

# plot the graph
plt.grid()
#graph, = ax.plot(p.T[0], p.T[1]) # *.T means transpose the position vector
graph, = ax.plot(p[:,0], p[:,1], p[:,2])
ax.set_zlim([0, 2.0])


# position of sliders
slider1_pos = plt.axes([0.1, 0.10, 0.35, 0.02])
slider2_pos = plt.axes([0.1, 0.07, 0.35, 0.02])
slider3_pos = plt.axes([0.1, 0.04, 0.35, 0.02])
slider4_pos = plt.axes([0.1, 0.01, 0.35, 0.02])
slider5_pos = plt.axes([0.55, 0.10, 0.35, 0.02])
slider6_pos = plt.axes([0.55, 0.07, 0.35, 0.02])
slider7_pos = plt.axes([0.55, 0.04, 0.35, 0.02])
slider8_pos = plt.axes([0.55, 0.01, 0.35, 0.02])

"""
slider_pos = {slider1_pos, slider2_pos, slider3_pos, slider4_pos, slider5_pos, slider6_pos, slider7_pos, slider8_pos}
threshold_slider = {}
# make the instance for slider objects
for i in range(len(L)):
    threshold_slider[i] = Slider(slider_pos[i], 'th' + str(i + 1), -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
    thNumber = i
    threshold_slider[i].on_changed(update_th)
"""

threshold_slider1 = Slider(slider1_pos, 'th1', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider2 = Slider(slider2_pos, 'th2', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider3 = Slider(slider3_pos, 'th3', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider4 = Slider(slider4_pos, 'th4', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider5 = Slider(slider5_pos, 'th5', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider6 = Slider(slider6_pos, 'th6', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider7 = Slider(slider7_pos, 'th7', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
threshold_slider8 = Slider(slider8_pos, 'th8', -1.0*math.pi, 1.0*math.pi, 0.0*math.pi)
"""
# update the graph when the slider positions are modified
"""
threshold_slider1.on_changed(update_th1)
threshold_slider2.on_changed(update_th2)
threshold_slider3.on_changed(update_th3)
threshold_slider4.on_changed(update_th4)
threshold_slider5.on_changed(update_th5)
threshold_slider6.on_changed(update_th6)
threshold_slider7.on_changed(update_th7)
threshold_slider8.on_changed(update_th8)

graph.set_linestyle('-')
graph.set_linewidth(5)
graph.set_marker('o')
graph.set_markerfacecolor('g')
graph.set_markeredgecolor('g')
graph.set_markersize(15)
plt.grid()
plt.show()
