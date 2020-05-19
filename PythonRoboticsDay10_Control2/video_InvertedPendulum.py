#!/usr/bin/env python3

# video_InvertedPendulum.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import math


def video(x, dt, max_t, params, gains, targets, S):

    m_th, m_x, I, l_g, g = params
    K_p, K_v = gains
    theta_d, dtheta_d = targets
    s1_re, s1_im, s2_re, s2_im = S

    alpha = (I*(m_x+m_th))/(m_th*l_g) + m_x*l_g
    beta  = (m_x + m_th)*g

    #initial setting of figures
    fig = plt.figure(figsize=(10, 6.))
    gs  = gridspec.GridSpec(2,3)

    # setting for ax1 area
    ax1 = fig.add_subplot(gs[0,0], xlim=(-1.1, 1.1), ylim=(-0.6, 1.6))
    ax1.set_xlabel('x[m]')
    ax1.set_ylabel('y[m]')
    ax1.grid()

    lp,     = plt.plot([], [], 'b-', lw=2, animated = True)                    # draw link anime
    cart,   = plt.plot([], [], 'co', markersize = 10, animated = True)         # draw cart
    p_com,  = plt.plot([], [], 'mo', markersize = 10, animated = True)         # draw p_com anime
    p_tip,  = plt.plot([], [], 'mo', markersize = 10, animated = True)         # draw p_com anime



    # setting for ax2 area
    ax2 = fig.add_subplot(gs[0,1:3], xlim=(0, max_t), ylim=(-0.5*math.pi, 0.5*math.pi))
    ax2.set_xlabel('time[s]')
    ax2.set_ylabel('theta[rad]')
    ax2.grid()

    #calculation of energies
    kinetic_energy   = 0.5*I*x[:,3]*x[:,3] + 0.5*m_th*(x[:,1]*x[:,1]+2*l_g*x[:,1]*x[:,3]*np.cos(x[:,2])+l_g*l_g*x[:,3]*x[:,3]) + 0.5*m_x*x[:,1]*x[:,1]

    potential_energy = m_th*g*l_g*np.cos(x[:,2])

    total_energy     = kinetic_energy + potential_energy

    Theta_d, = plt.plot([0, max_t], [theta_d, theta_d], 'r--', lw=1, alpha=0.1) #draw theta_d

    time  =  np.arange(0.0, max_t, dt) # make a list fot time
    Theta, = plt.plot(time, x[:,2], 'g-', lw=1.5, alpha=0.8) #draw theta

    bar, = plt.plot([], [], 'r-', lw=1, animated=True)           #draw time bar w anime
    TH,  = plt.plot([], [], 'go', markersize = 5,  animated=True) #draw theta point w anime

    time_template = 'time = %.2f s'
    time_text = ax1.text(0.54, 0.125, '', transform=ax1.transAxes)


    # setting for ax3 area
    ax3 = fig.add_subplot(gs[1,0], xlim=(-10, 10), ylim=(-10, 10))
    ax3.set_xlabel('Re{s}')
    ax3.set_ylabel('Im{s}')
    ax3.grid()
    ax3.plot([-15, 15],[0, 0], 'k-') # x axis
    ax3.plot([0, 0],[-15, 15], 'k-') # y axis

    cycle_1, = plt.plot(s1_re, s1_im, 'rx', markersize = 10, linewidth=10, alpha=0.5) #draw pole 1
    cycle_2, = plt.plot(s2_re, s2_im, 'bx', markersize = 10, linewidth=10, alpha=0.5) #draw pole 2

    r = plt.Rectangle((-15, -15), 15.0, 30.0, fc='b', alpha=0.1, fill=True)

    # setting for ax5 area
    ax5 = fig.add_subplot(gs[1,1], xlim=(0, 30), ylim=(0, 10))
    ax5.set_xlabel('Kp')
    ax5.set_ylabel('Kv')
    ax5.grid()
    ax5.plot([-500, 500],[0, 0], 'k-') # x axis
    ax5.plot([0, 0],[-500, 500], 'k-') # y axis

    xs = np.arange(beta,50,0.1)
    yp =  np.sqrt(4*alpha*(xs-beta))
    #ym = -np.sqrt(4*alpha*(xs-beta))

    ax5.plot(xs,yp)
    #ax5.plot(xs,ym)

    gain, = plt.plot(K_p, K_v, 'r*', markersize = 15, alpha=0.5) #draw pole 1

    rg = plt.Rectangle((beta, -200), 200.0, 400.0, fc='b', alpha=0.1, fill=True)


    # setting for ax4 area
    ax4 = fig.add_subplot(gs[1,2])
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

    plt.tight_layout()

    #initial function for animation
    def init():

        r = plt.Rectangle((-15, -15), 15.0, 30.0, fc='b', alpha=0.1, fill=True)
        rg = plt.Rectangle((beta, -200), 200.0, 400.0, fc='b', alpha=0.1, fill=True)
        ax3.add_patch(r)
        ax5.add_patch(rg)

        return rg, gain, r, lp, cart, p_com, p_tip, Theta, Theta_d, bar, TH, time_text, cycle_1, cycle_2, \
                params_text1, params_text2, params_text3, params_text4, params_text5, params_text6, params_text7\

    #function for animation
    def anime(i):

        ax3.add_patch(r)
        ax5.add_patch(rg)

        next_cartx   = 0 # x[i,0] # x position of elbow mass
        next_carty   = 0 # y position of elbow mass
        next_pcomx   = next_cartx +   l_g*math.sin(x[i,2])  # x position of pendulum COM
        next_pcomy   = next_carty +   l_g*math.cos(x[i,2])  # y position of pendulum COM
        next_ptipx   = next_cartx + 2*l_g*math.sin(x[i,2])  # x position of pendulum tip
        next_ptipy   = next_carty + 2*l_g*math.cos(x[i,2])  # y position of pendulum tip
        next_lx   = [next_cartx, next_ptipx] # x positions for link
        next_ly   = [next_carty, next_ptipy] # y positions for link
        next_bx   = [time[i], time[i]]  # x positions for time bar
        next_by   = [-30, 30]           # y positions for time bar


        lp.set_data(next_lx, next_ly)          # set lp positions
        cart.set_data(next_cartx, next_carty)       # set elbow position
        p_com.set_data(next_pcomx, next_pcomy)        # set hand position
        p_tip.set_data(next_ptipx, next_ptipy)        # set hand position

        bar.set_data(next_bx, next_by)           # set time bar position
        TH.set_data(time[i], x[i,2])  # set ke position

        time_text.set_text(time_template % time[i]) # display timer

        params_text1.set_text('m_th = %.3f kg' % (m_th))  # display params
        params_text2.set_text('m_x  = %.3f kg' % (m_x))   # display params
        params_text3.set_text('I    = %.3f kg m^2' % (I)) # display params
        params_text4.set_text('l_g  = %.3f m' % (l_g))    # display params
        params_text5.set_text('g    = %.3f m/s^2' % (g))  # display params
        params_text6.set_text('K_p  = %.3f N/rad' % (K_p))    # display params
        params_text7.set_text('K_v  = %.3f Ns/rad' % (K_v))  # display params


        return rg, gain, r, lp, cart, p_com, p_tip, Theta, Theta_d, bar, TH, time_text, cycle_1, cycle_2, \
                params_text1, params_text2, params_text3, params_text4, params_text5, params_text6, params_text7\


    ani = animation.FuncAnimation(fig, anime, np.arange(1, len(x)), interval=dt*1.0e+3, blit=True, init_func=init)

    #ani.save('py_InvertedPendulum_{}_{}.mp4'.format(K_p,K_v), writer='ffmpeg')
    #ani.save('py_InvertedPendulum.mp4', writer='ffmpeg')

    #plt.tight_layout()
    plt.show()
