#!/usr/bin/env python3

# Force_InverseJacobian_2Links.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

radius = 20

a_0 =  1.0
a_1 =  0.8
a = [a_0, a_1] # parameter for environment

l_1 = 0.5 # length of link 1 [m]
l_2 = 0.5 # length of link 2 [m]

L   = [l_1, l_2] # link parameters
th  = [0.0*math.pi, 0.2*math.pi] # initial angles
dth = np.array([0.0, 0.0]) # initial angular velocity
F     = np.array([0.0, 0.0]) # initial force
tau   = np.array([0.0, 0.0]) # initial torque

X = [0.0, 0.5]# initial position
dx  = np.array([0.0, 0.0]) # initial velocity of end effector

def Jacobian(th, L):

    L1, L2 = L
    Th1, Th2 = th

    J11 = - L1*math.sin(Th1) - L2*math.sin(Th1+Th2)
    J12 = - L2*math.sin(Th1+Th2)
    J21 =   L1*math.cos(Th1) + L2*math.cos(Th1+Th2)
    J22 =   L2*math.cos(Th1+Th2)

    return np.array([[J11, J12],[J21, J22]])

def InverseJacobian(th, L):

    L1, L2 = L
    Th1, Th2 = th

    #Singular point
    if -0.001 < L1*L2*math.sin(Th2) < 0:
        detJ = -0.001
        print('Singular point!')
    elif 0 <= L1*L2*math.sin(Th2) < 0.001:
        detJ =  0.001
        print('Singular point!')
    else:
        detJ = L1*L2*math.sin(Th2)

    IJ11 = ( 1/detJ )*(  L2*math.cos(Th1+Th2) )
    IJ12 = ( 1/detJ )*(  L2*math.sin(Th1+Th2) )
    IJ21 = ( 1/detJ )*( -L1*math.cos(Th1) - L2*math.cos(Th1+Th2) )
    IJ22 = ( 1/detJ )*( -L1*math.sin(Th1) - L2*math.sin(Th1+Th2) )

    return np.array([[IJ11, IJ12],[IJ21, IJ22]])

def InverseKinematics(X, L):

    L1, L2 = L
    x, y = X

    # anlge of the link 1
    if (L1*L1+(x*x+y*y)-L2*L2)/(2*L1*math.sqrt(x*x+y*y)) > 1:
        th1 = math.atan2(y, x)
        print('th1 error {}, {}'.format(x,y))
    elif (L1*L1+(x*x+y*y)-L2*L2)/(2*L1*math.sqrt(x*x+y*y)) < -1:
        th1 = math.atan2(y, x) - math.pi
        print('th1 error {}, {}'.format(x,y))
    else:
        th1 = math.atan2(y, x) - math.acos( (L1*L1+(x*x+y*y)-L2*L2)/(2*L1*math.sqrt(x*x+y*y)) )

    # anlge of the link 2
    if (L1*L1+L2*L2-(x*x+y*y))/(2*L1*L2) > 1:
        th2 = math.pi
        print('th2 error {}, {}'.format(x,y))
    elif (L1*L1+L2*L2-(x*x+y*y))/(2*L1*L2) < -1:
        th2 = 0.0
        print('th2 error {}, {}'.format(x,y))
    else:
        th2 = math.pi - math.acos( (L1*L1+L2*L2-(x*x+y*y))/(2*L1*L2) )


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

    X = np.array([[x0, y0],[x1, y1],[x2, y2]])

    return X

def update_x(slider_val):
    # update the x-position of  end effector
    X[0] = slider_val
    X[1] = a[1]*X[0] + a[0]

    # calculation of inverse kinematics
    th = InverseKinematics(X, L)

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of InverseJacobian
    #dth = np.dot(InverseJacobian(th, L), dx)

    # calculation of Transpose Jaconian
    tau = np.dot(Jacobian(th, L).T, F)

    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[2,0],p[2,0]+F[0]],[p[2,1],p[2,1]+F[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])

    angularvel1.set_markersize(radius*(1.0+tau[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')
    angularvel2.set_markersize(radius*(1.0+tau[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    text1.set_text('det|Ja(Th)|={}'.format(l_1*l_2*math.sin(th[1])))

    # update the target position
    lineX.set_data([X[0], X[0]],[-100, 100])
    lineY.set_data([-100, 100],[X[1], X[1]])


    # re-draw of the links
    fig.canvas.draw_idle()


def update_fx(slider_val):
    # update the angular vel of link 1
    F[0] = slider_val
    # y-force on environment
    F[1] = (-1/a[1])*F[0]

    # calculation of inverse kinematics
    th = InverseKinematics(X, L)

    # calculation of forward kinematics
    p = ForwardKinematics(th, L)

    # calculation of InverseJacobian
    #dth = np.dot(InverseJacobian(th, L), dx)

    # calculation of Transpose Jaconian
    tau = np.dot(Jacobian(th, L).T, F)


    # update the potision of each point
    graph.set_data(p.T[0], p.T[1]) # *.T means transpose the position vector
    arrow.set_data([p[2,0],p[2,0]+F[0]],[p[2,1],p[2,1]+F[1]])
    angularvel1.set_data(p[0,0], p[0,1])
    angularvel2.set_data(p[1,0], p[1,1])

    angularvel1.set_markersize(radius*(1.0+tau[0]))
    angularvel1.set_markerfacecolor('m')
    angularvel1.set_markeredgecolor('m')
    angularvel2.set_markersize(radius*(1.0+tau[1]))
    angularvel2.set_markerfacecolor('c')
    angularvel2.set_markeredgecolor('c')

    text1.set_text('det|Ja(Th)|={}'.format(l_1*l_2*math.sin(th[1])))

    # update the target position
    lineX.set_data([X[0], X[0]],[-100, 100])
    lineY.set_data([-100, 100],[X[1], X[1]])


    # re-draw of the links
    fig.canvas.draw_idle()


# y-position on environment
X[1] = a[1]*X[0] + a[0]

# y-force on environment
F[1] = (-1/a[1])*F[0]

th = InverseKinematics(X, L)
p  = ForwardKinematics(th, L)
#dx = np.dot(Jacobian(th, L), dth)
#dth = np.dot(InverseJacobian(th, L), dx)
tau = np.dot(Jacobian(th, L).T, F)


fig, ax = plt.subplots()
plt.title('Force Transpose Jacobian 2Links')
plt.axis('equal')
plt.subplots_adjust(left=0.1, bottom=0.15)
plt.xlim([-1.3, 1.3])
plt.ylim([-1.3, 1.3])

# plot the graph
plt.grid()
lineE  = plt.plot([-2.0,2.0],[a[1]*(-2.0)+a[0],a[1]*(2.0)+a[0]],'k-',lw=2.0)
lineX, = plt.plot([p[2,0], p[2,0]],[-100, 100], 'r')
lineY, = plt.plot([-100, 100],[p[2,1], p[2,1]], 'r')
arrow, = plt.plot([p[2,0],p[2,0]+F[0]],[p[2,1],p[2,1]+F[1]],'r-', lw=7)
graph, = plt.plot(p.T[0], p.T[1]) # *.T means transpose the position vector
angularvel1, = plt.plot(p[0,0], p[0,1])
angularvel2, = plt.plot(p[1,0], p[1,1])
text1 = plt.text(0.03, -1.1, 'det|Ja(Th)|={}'.format(l_1*l_2*math.sin(th[1])))
c=plt.Circle((0,0),l_1+l_2, edgecolor='k', facecolor='w', LineWidth=2.0)
ax.add_patch(c)

# position of sliders
slider1_pos = plt.axes([0.05, 0.05, 0.35, 0.03])
slider3_pos = plt.axes([0.55, 0.05, 0.35, 0.03])

# make the instance for slider objects
threshold_slider1 = Slider(slider1_pos, 'x',  -1.0, 1.0, 0.0)
threshold_slider3 = Slider(slider3_pos, 'fx', -1.0, 1.0, 0.0)

# update the graph when the slider positions are modified
threshold_slider1.on_changed(update_x)
threshold_slider3.on_changed(update_fx)


graph.set_linestyle('-')
graph.set_linewidth(5)
graph.set_marker('o')
graph.set_markerfacecolor('g')
graph.set_markeredgecolor('g')
graph.set_markersize(radius)

angularvel1.set_marker('o')
angularvel1.set_markersize(radius*(1.0+tau[0]))
angularvel1.set_markerfacecolor('m')
angularvel1.set_markeredgecolor('m')
angularvel2.set_marker('o')
angularvel2.set_markersize(radius*(1.0+tau[1]))
angularvel2.set_markerfacecolor('c')
angularvel2.set_markeredgecolor('c')

plt.grid()
plt.show()
