#!/usr/bin/env python3

# video.py
# Copyright (c) 2017 Dai Owaki <owaki@tohoku.ac.jp>
# Revised 2019 by Dai Owaki
#
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation


def video(x, dt, max_t, params):

    basewidth  = 0.25   # base width
    baseheight = 1.0   # base height

    #initial setting of figures
    fig = plt.figure(figsize=(13, 3))
    gs  = gridspec.GridSpec(1,4)

    # setting for ax1 area
    ax1 = fig.add_subplot(gs[0,0], xlim=(-1, 1), ylim=(-0.5, 0.5))
    ax1.set_xlabel('x[m]')
    ax1.set_ylabel('y[m]')
    ax1.grid()

    spring, = plt.plot([], [], 'b-', lw=2, animated = True)                     # draw spring w anime
    mass, = plt.plot([], [], 'ro', markersize = 20, animated = True)            # draw mass   w anime
    base  = plt.Rectangle((-1, -0.5), basewidth, baseheight, fc='g', fill=True) # draw base
    ax1.add_patch(base)

    # setting for ax2 area
    ax2 = fig.add_subplot(gs[0,1:4], xlim=(0, max_t), ylim=(0, 2))
    ax2.set_xlabel('t[s]')
    ax2.set_ylabel('E[J]')
    ax2.grid()

    #calculation of energies
    kinetic_energy   = 0.5*params[0]*x[:,1]*x[:,1]
    potential_energy = 0.5*params[1]*x[:,0]*x[:,0]
    total_energy     = kinetic_energy + potential_energy

    time  =  np.arange(0.0, max_t, dt) # make a list fot time
    line1, = plt.plot(time, kinetic_energy,   'g-', lw=1, alpha=0.3) #draw kinetic energy
    line2, = plt.plot(time, potential_energy, 'b-', lw=1, alpha=0.3) #draw potantial energy
    line3, = plt.plot(time, total_energy,     'r-', lw=1, alpha=0.3) #draw total energy

    bar, = plt.plot([], [], 'r-', lw=1, animated=True)           #draw time bar w anime
    ke, = plt.plot([], [], 'go', markersize = 5,  animated=True) #draw ke point w anime
    pe, = plt.plot([], [], 'bo', markersize = 5,  animated=True) #draw pe point w anime
    te, = plt.plot([], [], 'ro', markersize = 10, animated=True) #draw te point w anime

    time_template = 'time = %.2fs'
    time_text = ax1.text(0.54, 0.925, '', transform=ax1.transAxes)

    plt.tight_layout()

    #initial function for animation
    def init():
        return spring, base, mass, line1, line2, line3, bar, ke, pe, te, time_text

    #function for animation
    def anime(i):

        mass.set_data(x[i,0], 0)                 # set mass position
        spring.set_data([-0.95, x[i,0]], [0, 0]) # set spring line (positions)
        bar.set_data([time[i], time[i]], [0, 2])    # set time bar position
        ke.set_data(time[i], kinetic_energy[i])   # set ke position
        pe.set_data(time[i], potential_energy[i])   # set pe position
        te.set_data(time[i], total_energy[i])   # set te position

        time_text.set_text(time_template % time[i]) # display timer

        return spring, base, mass, line1, line2, line3, bar, ke, pe, te, time_text

    ani = animation.FuncAnimation(fig, anime, np.arange(1, len(x)), interval=dt*1.0e+3, blit=True, init_func=init)
    #ani.save('py_mass_spring.mp4', writer='ffmpeg')

    plt.show()
