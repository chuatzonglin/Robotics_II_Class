#!/usr/bin/env python3

# video_ImpedanceControl.py
# Copyright (c) 2017 Dai Owaki <owaki@tohoku.ac.jp>
# Revised 2019 by Dai Owaki
#
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation


def video(x, dt, max_t, params):

    m, c, k, md, cd, kd = params

    #initial setting of figures
    fig = plt.figure(figsize=(13, 3))
    gs  = gridspec.GridSpec(1,4)

    # setting for ax1 area
    ax1 = fig.add_subplot(gs[0,0], xlim=(-2, 2), ylim=(-0.5, 0.5))
    ax1.set_xlabel('x[m]')
    ax1.set_ylabel('y[m]')
    ax1.grid()

    spring, = plt.plot([], [], 'b-', lw=2, animated = True)                     # draw spring w anime
    mass, = plt.plot([], [], 'ro', markersize = 15, animated = True)            # draw mass   w anime
    plt.plot([0,0],[-2,2],'k', lw=2)

    # setting for ax2 area
    ax2 = fig.add_subplot(gs[0,1:4], xlim=(0, max_t), ylim=(-2, 2))
    ax2.set_xlabel('t[s]')
    ax2.set_ylabel('x[m]')
    ax2.grid()

    time  =  np.arange(0.0, max_t, dt) # make a list fot time
    line, = plt.plot(time, x[:,0],     'r-', lw=1, alpha=0.3) #draw mass trajectory

    bar, = plt.plot([], [], 'r-', lw=1, animated=True)          #draw time bar w anime
    p, = plt.plot([], [], 'ro', markersize = 10, animated=True) #draw mass point w anime

    time_template = 'time = %.2fs'
    time_text1 = ax1.text(0.52, 0.925, '', transform=ax1.transAxes)
    time_text2 = ax1.text(0.02, 0.925, '', transform=ax1.transAxes)
    time_text3 = ax1.text(0.02, 0.825, '', transform=ax1.transAxes)
    time_text4 = ax1.text(0.02, 0.725, '', transform=ax1.transAxes)

    plt.tight_layout()

    #initial function for animation
    def init():
        return spring, mass, line, bar, p, time_text1, time_text2, time_text3, time_text4,

    #function for animation
    def anime(i):

        mass.set_data(x[i,0], 0)                 # set mass position
        spring.set_data([0, x[i,0]], [0, 0]) # set spring line (positions)
        bar.set_data([time[i], time[i]], [-2, 2])    # set time bar position
        p.set_data(time[i], x[i,0])   # set ke position

        time_text1.set_text(time_template % time[i]) # display timer
        time_text2.set_text('M,Md=({},{})'.format(m, md)) # display md
        time_text3.set_text('C,Cd=({},{})'.format(c, cd)) # display cd
        time_text4.set_text('K,Kd=({},{})'.format(k, kd)) # display cd

        return spring, mass, line, bar, p, time_text1, time_text2, time_text3, time_text4,

    ani = animation.FuncAnimation(fig, anime, np.arange(1, len(x)), interval=dt*1.0e+3, blit=True, init_func=init)
    #ani.save('py_Impedance(Md{},Cd{},Kd{}).mp4'.format(md,cd,kd), writer='ffmpeg')

    plt.show()
