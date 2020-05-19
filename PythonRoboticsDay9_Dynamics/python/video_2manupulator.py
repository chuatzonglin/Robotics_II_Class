#!/usr/bin/env python3

# video_2manupulator.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import math

def video(x, dt, max_t, params):

    m_1, m_2, I_1, I_2, l_g1, l_1, l_g2, g = params

    #initial setting of figures
    fig = plt.figure(figsize=(10, 6.))
    gs  = gridspec.GridSpec(2,3)

    # setting for ax1 area
    ax1 = fig.add_subplot(gs[0,0], xlim=(-2, 2), ylim=(-2, 2))
    ax1.set_xlabel('x[m]')
    ax1.set_ylabel('y[m]')
    ax1.grid()

    l2,     = plt.plot([], [], 'b-', lw=2, animated = True)                    # draw l2 anime
    l1,     = plt.plot([], [], 'b-', lw=2, animated = True)                    # draw l1 anime
    sholder,= plt.plot(0, 0, 'ro', markersize = 10, animated = False)         # draw sholder anime
    hand,   = plt.plot([], [], 'co', markersize = 10, animated = True)          # draw hand
    elbow,  = plt.plot([], [], 'mo', markersize = 10, animated = True)         # draw elbow anime

    hand_traject,  = plt.plot([], [], 'c-', lw=1, alpha=0.5, animated = True)   # hand trajectory w anime
    elbow_traject, = plt.plot([], [], 'm-', lw=1, alpha=0.5, animated = True)   # elbow trajectory w anime

    # setting for ax2 area
    ax2 = fig.add_subplot(gs[0,1:3], xlim=(0, max_t), ylim=(-30, 30))
    ax2.set_xlabel('t[s]')
    ax2.set_ylabel('E[J]')
    ax2.grid()

    #calculation of energies
    kinetic_energy   = 0.5*(m_1*l_g1*l_g1+m_2*l_1*l_1)*x[:,1]*x[:,1] + 0.5*m_2*l_g2*l_g2*(x[:,1]+x[:,3])*(x[:,1]+x[:,3]) \
                     + m_2*l_g2*x[:,1]*(x[:,1]+x[:,3])*np.cos(x[:,2]) \
                     + 0.5*I_1*x[:,1]*x[:,1] + 0.5*I_2*(x[:,1]+x[:,3])*(x[:,1]+x[:,3])

    potential_energy = (m_1*l_g1+m_2*l_1)*g*np.sin(x[:,0]) + m_2*l_g2*g*np.sin(x[:,0]+x[:,2])

    total_energy     = kinetic_energy + potential_energy

    time  =  np.arange(0.0, max_t, dt) # make a list fot time
    line1, = plt.plot(time, kinetic_energy,   'g-', lw=1, alpha=0.3) #draw kinetic energy
    line2, = plt.plot(time, potential_energy, 'b-', lw=1, alpha=0.3) #draw potantial energy
    line3, = plt.plot(time, total_energy,     'r-', lw=1, alpha=0.3) #draw total energy

    bar, = plt.plot([], [], 'r-', lw=1, animated=True)           #draw time bar w anime
    ke, = plt.plot([], [], 'go', markersize = 5,  animated=True) #draw ke point w anime
    pe, = plt.plot([], [], 'bo', markersize = 5,  animated=True) #draw pe point w anime
    te, = plt.plot([], [], 'ro', markersize = 10, animated=True) #draw te point w anime

    time_template = 'time = %.2f s'
    time_text = ax1.text(0.54, 0.925, '', transform=ax1.transAxes)

    # setting for ax3 area
    ax3 = fig.add_subplot(gs[1,0], xlim=(-8*math.pi, 8*math.pi), ylim=(-20, 20))
    ax3.set_xlabel('theta[rad]')
    ax3.set_ylabel('dtheta[rad/s]')
    ax3.grid()

    cycle_1, = plt.plot(x[:,0], x[:,1], 'm-', lw=1.5, alpha=0.5) #draw 1 cycle
    cycle_2, = plt.plot(x[:,2], x[:,3], 'c-', lw=1.5, alpha=0.5) #draw 2 cycle

    pt_1, = plt.plot([], [], 'mo', markersize = 10, alpha=1.0) #draw 1 anime
    pt_2, = plt.plot([], [], 'co', markersize = 10, alpha=1.0) #draw 2 anime


    # setting for ax4 area
    ax4 = fig.add_subplot(gs[1,1])
    ax4.tick_params(labelbottom="off",bottom="off")
    ax4.tick_params(labelleft="off",left="off")
    ax4.set_xticklabels([])

    params_text1 = ax4.text(0.1, 0.9, '', transform=ax4.transAxes)
    params_text2 = ax4.text(0.1, 0.8, '', transform=ax4.transAxes)
    params_text3 = ax4.text(0.1, 0.7, '', transform=ax4.transAxes)
    params_text4 = ax4.text(0.1, 0.6, '', transform=ax4.transAxes)
    params_text5 = ax4.text(0.1, 0.5, '', transform=ax4.transAxes)
    params_text6 = ax4.text(0.1, 0.4, '', transform=ax4.transAxes)
    params_text7 = ax4.text(0.1, 0.3, '', transform=ax4.transAxes)
    params_text8 = ax4.text(0.1, 0.2, '', transform=ax4.transAxes)

    plt.tight_layout()

    #initial function for animation
    def init():
        return hand_traject, elbow_traject, l2, l1, sholder, elbow, hand, line1, line2, line3, bar, ke, pe, te, time_text, cycle_1, cycle_2, pt_1, pt_2,\
                params_text1, params_text2, params_text3, params_text4, params_text5, params_text6, params_text7, params_text8, \

    #function for animation
    def anime(i):
        next_elx   = 0 + l_1*math.cos(x[i,0])  # x position of elbow mass
        next_ely   = 0 + l_1*math.sin(x[i,0])  # y position of elbow mass
        next_hax   = next_elx + 2*l_g2*math.cos(x[i,0]+x[i,2])  # x position of hand mass
        next_hay   = next_ely + 2*l_g2*math.sin(x[i,0]+x[i,2])  # y position of hand mass
        next_l1x   = [0, next_elx]       # x positions for l1
        next_l1y   = [0, next_ely]       # y positions for l1
        next_l2x   = [next_elx, next_hax] # x positions for leg
        next_l2y   = [next_ely, next_hay] # y positions for legs
        next_bx   = [time[i], time[i]]  # x positions for time bar
        next_by   = [-30, 30]           # y positions for time bar

        next_etrx   = l_1*np.cos(x[0:i,0]) # x positions for elbow traject
        next_etry   = l_1*np.sin(x[0:i,0]) # y positions for elbow traject
        elbow_traject.set_data(next_etrx, next_etry)

        next_htrx   = l_1*np.cos(x[0:i,0]) + 2*l_g2*np.cos(x[0:i,0]+x[0:i,2]) # x positions for hand traject
        next_htry   = l_1*np.sin(x[0:i,0]) + 2*l_g2*np.sin(x[0:i,0]+x[0:i,2]) # y positions for hand traject
        hand_traject.set_data(next_htrx, next_htry)


        l1.set_data(next_l1x, next_l1y)          # set l1 positions
        l2.set_data(next_l2x, next_l2y)          # set l2 positions
        elbow.set_data(next_elx, next_ely)       # set elbow position
        hand.set_data(next_hax, next_hay)        # set hand position
        bar.set_data(next_bx, next_by)           # set time bar position
        ke.set_data(time[i], kinetic_energy[i])  # set ke position
        pe.set_data(time[i], potential_energy[i])# set pe position
        te.set_data(time[i], total_energy[i])    # set te position

        time_text.set_text(time_template % time[i]) # display timer

        pt_1.set_data(x[i,0], x[i,1]) # set point 1
        pt_2.set_data(x[i,2], x[i,3]) # set point 2

        params_text1.set_text('m1   = %.3f kg' % (m_1)) # display params
        params_text2.set_text('m2   = %.3f kg' % (m_2)) # display params
        params_text3.set_text('I1   = %.3f kg m^2' % (I_1)) # display params
        params_text4.set_text('I2   = %.3f kg m^2' % (I_2)) # display params
        params_text5.set_text('l_g1 = %.3f m' % (l_g1)) # display params
        params_text6.set_text('l_1  = %.3f m' % (l_1)) # display params
        params_text7.set_text('l_g2 = %.3f m' % (l_g2)) # display params
        params_text8.set_text('g    = %.3f m/s^2' % (g)) # display params


        return hand_traject, elbow_traject, l2, l1, sholder, elbow, hand, line1, line2, line3, bar, ke, pe, te, time_text, cycle_1, cycle_2, pt_1, pt_2,\
                params_text1, params_text2, params_text3, params_text4, params_text5, params_text6, params_text7, params_text8, \


    ani = animation.FuncAnimation(fig, anime, np.arange(1, len(x)), interval=dt*1.0e+3, blit=True, init_func=init)
    #ani.save('py_2manipulator.mp4', writer='ffmpeg')

    plt.show()
