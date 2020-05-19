#!/usr/bin/env python3

# ManupulatorControl_odeint.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>
# modified by 2020 Hayashibe
# submitted and modified as assignment by 2020 ALvin Quek

from scipy.integrate import odeint
import numpy as np
import math

# import original modules
import video_ManupulatorControl as v2m

m_1  = 1.0        # mass 1 [kg]
m_2  = 1.0        # mass 2 [kg]
I_1  = 1.0        # inertia 1 [kg m^2]
I_2  = 1.0        # inertia 2 [kg m^2]
l_g1 = 0.5       # length 1g [m]
l_1  = 1.0       # length 1  [m]
l_g2 = 0.5       # length 2g [m]
l_2 = 1.0       #l2 length 1[m]
g = 9.8          # gravitational accelaration[m/s^2]

params = [m_1, m_2, I_1, I_2, l_g1, l_1, l_g2, g] # parameters for printing in video

K_p1 = 15.0
K_p2 = 15.0
K_v1 = 7.0
K_v2 = 7.0

K = [K_p1, K_p2, K_v1, K_v2]
Kp= [K_p1,K_p2]
Kv= [K_v1,K_v2]
L=[l_1,l_2]

##theta1_d = 0.1*math.pi
##theta2_d = 0.5*math.pi
##dtheta1_d = 0.0
##dtheta2_d = 0.0

# initial conditions(x0, dx0)
max_t = 20.0 # max_time [s]
dt = 0.01    # dt [s]

Xd1 = [ 0.5,  0.7]
Xd2 = [-0.5,  1.9]
Xd3 = [-1.0,  0.5]
Xd4 = [-1.95, 0.0]

XD = [Xd1, Xd2, Xd3, Xd4]

def Trajectory(t):

    xd = XD[0][0]
    yd = XD[0][1]

    if 2.5 < t < 5.0:
        xd = XD[1][0]
        yd = XD[1][1]
    elif 5.0 < t < 7.5:
        xd = XD[2][0]
        yd = XD[2][1]
    elif 7.5 < t < 10.0:
        xd = XD[3][0]
        yd = XD[3][1]

    return [xd, yd]

def InverseKinematics(X, L):

    L1, L2 = L
    x, y = X
    # anlge of the link 1
    th1 = math.atan2(y, x) - math.acos( (L1*L1+(x*x+y*y)-L2*L2)/(2*L1*math.sqrt(x*x+y*y)) )
    # anlge of the link 2
    th2 = math.pi - math.acos( (L1*L1+L2*L2-(x*x+y*y))/(2*L1*L2) )

    return [th1, th2]

def Forward_Kinematics(theta_1, theta_2,l_1,l_2):

    L1, L2 = l_1, l_2
    Th1, Th2 = theta_1, theta_2

    #x0 = 0.0
    #y0 = 0.0
    ##################################################
    # position of tip of link 1
    x1 = L1*math.cos(Th1)
    y1 = L1*math.sin(Th1)
    # position of tip of link 2
    x2 = x1 + L2*math.cos(Th1+Th2)
    y2 = y1 + L2*math.sin(Th1+Th2)
    ##################################################
 #  ''' X = np.array([[x0, y0],[x1, y1],[x2, y2]])'''

	#	x2 = self.L1*np.cos(self.theta_1) + self.L2*np.cos(self.theta_1 + self.theta_2)
	#	y2 = self.L1*np.sin(self.theta_1) + self.L2*np.sin(self.theta_1 + self.theta_2)
	#	return np.array([x2,y2])
    return np.array([x2,y2]) #X

def Jacobian(theta_1, theta_2,L1,L2):
    
    #theta_1, theta_2 = th
    
    J11 = - L1*math.sin(theta_1) - L2*math.sin(theta_1+theta_2)
    J12 = - L2*math.sin(theta_1+theta_2)
    J21 =   L1*math.cos(theta_1) + L2*math.cos(theta_1+theta_2)
    J22 =   L2*math.cos(theta_1+theta_2)
    
    return np.array([[J11, J12],[J21, J22]])

def diff_Jacobian( theta_1, dtheta_1, theta_2, dtheta_2, L1, L2  ):
    
	dJ11 = - L1*dtheta_1*np.cos(theta_1) - L2*(dtheta_1+dtheta_2)*np.cos(theta_1+theta_2)
	dJ12 = - L2*(dtheta_1+dtheta_2)*np.cos(theta_1+theta_2)
	dJ21 = - L1*dtheta_1*np.sin(theta_1) - L2*(dtheta_1+dtheta_2)*np.sin(theta_1+theta_2)
	dJ22 = - L2*(dtheta_1+dtheta_2)*np.sin(theta_1+theta_2)

	return np.array([[dJ11, dJ12],[dJ21, dJ22]])



#def Task_Space(self, target_position, target_velocity, target_acceleration = [0,0]):


def Inverse_Jacobian(theta_1,theta_2,L1,L2):
		#Singular point
		if -0.001 < L1*L2*np.sin(theta_2) < 0:
		    detJ = -0.001
		    print('Singularity')
		elif 0 <= L1*L2*np.sin(theta_2) < 0.001:
		    detJ =  0.001
		    print('Singularity')
		else:
		    detJ = L1*L2*np.sin(theta_2)

		IJ11 = ( 1/detJ )*(  L2*np.cos(theta_1+theta_2) )
		IJ12 = ( 1/detJ )*(  L2*np.sin(theta_1+theta_2) )
		IJ21 = ( 1/detJ )*( -L1*np.cos(theta_1) - L2*np.cos(theta_1+theta_2) )
		IJ22 = ( 1/detJ )*( -L1*np.sin(theta_1) - L2*np.sin(theta_1+theta_2) )

		return np.array([[IJ11, IJ12],[IJ21, IJ22]])


def Control(p, t):
    theta_1, dtheta_1, theta_2, dtheta_2 = p

    
    
    #GRAVITY
    G_1 = m_1*g*l_g1*math.cos(theta_1) + m_2*g*( l_1*math.cos(theta_1) + l_g2*math.cos(theta_1+theta_2))
    G_2 = m_2*g*l_g2*math.cos(theta_1+theta_2)
    #NON-LINEAR
    N_1 = -m_2*l_1*l_g2*dtheta_2*( 2*dtheta_1 + dtheta_2 )*math.sin(theta_2)
    N_2 = m_2*l_1*l_g2*dtheta_1*dtheta_1*math.sin(theta_2)

    #Change this part for HW
    #Also change the way task is initiated to give end point position
    
    #MOMENT
    M_11 = I_1 + I_2 + m_1*l_g1*l_g1 + m_2*( l_1*l_1 + l_g2*l_g2 + 2*l_1*l_g2*math.cos(theta_2) )
    M_12 = I_2 + m_2*(l_g2*l_g2+l_1*l_g2*math.cos(theta_2))
    M_21 = I_2 + m_2*(l_g2*l_g2+l_1*l_g2*math.cos(theta_2))
    M_22 = I_2 + m_2*l_g2*l_g2
    
    #PUT INTO ARRAY
    M = np.array([[M_11, M_12],[M_21, M_22]])
    N = np.array([N_1,N_2])
    G = np.array([G_1,G_2])
    
    #theta1_d = 0.3*math.pi*t
    #theta2_d = math.pi*math.sin(t)

    #target trajectory
    Xd = Trajectory(t)
    
    #[theta1_d, theta2_d] = InverseKinematics(Xd, L)
    
    #Feedback control
    ##Feedb = np.matrix([[K_p1*(theta1_d-theta_1) + K_v1*(dtheta1_d-dtheta_1)],[K_p2*(theta2_d-theta_2) + K_v2*(dtheta2_d-dtheta_2)]])
    
#    F = Feedback(p,[l_1, l_2],[K_p1,K_p2],[K_v1,K_v2])
    
    #Tau = M.dot(Feedb)+N+G
    #F_1 = K_p1*(theta1_d-theta_1) + K_v1*(dtheta1_d-dtheta_1) + G_1 + N_1
    #F_2 = K_p2*(theta2_d-theta_2) + K_v2*(dtheta2_d-dtheta_2) + G_2 + N_2
    #F_1 = Tau[0,0]
   # F_2 = Tau[1,0]


     #Task_Space(self, target_position, target_velocity, target_acceleration = [0,0]):
 
   # F = Feedback(p,[l_1, l_2],[K_p1,K_p2],[K_v1,K_v2])
  #  Torque = M.dot(F.Joint_Space(Xd,[0,0])) + N + G
    
    #PartialDifferential
    PD = Kp*(Xd - Forward_Kinematics(theta_1, theta_2,l_1,l_2)) + Kv*([0,0] - np.dot(Jacobian(theta_1, theta_2,l_1,l_2), [dtheta_1,dtheta_2]))
	
    dJdTh = np.dot(diff_Jacobian(theta_1, dtheta_1, theta_2, dtheta_2, l_1, l_2),np.array([dtheta_1,dtheta_2]))
    invJ = Inverse_Jacobian(theta_1,theta_2,l_1,l_2)
    rv = np.dot(invJ,np.array(PD-dJdTh))
        
   
    #TORQUE CALCULATION
    T = M.dot(rv) + N + G

#    if t > 10.0:
 #       F_1 = 0
  #      F_2 = 0

    return T






def Manipulator(p, t):
    theta_1, dtheta_1, theta_2, dtheta_2 = p

    M_11 = I_1 + I_2 + m_1*l_g1*l_g1 + m_2*( l_1*l_1 + l_g2*l_g2 + 2*l_1*l_g2*math.cos(theta_2) )
    M_12 = I_2 + m_2*(l_g2*l_g2+l_1*l_g2*math.cos(theta_2))
    M_21 = I_2 + m_2*(l_g2*l_g2+l_1*l_g2*math.cos(theta_2))
    M_22 = I_2 + m_2*l_g2*l_g2

    N_1 = -m_2*l_1*l_g2*dtheta_2*( 2*dtheta_1 + dtheta_2 )*math.sin(theta_2)
    N_2 = m_2*l_1*l_g2*dtheta_1*dtheta_1*math.sin(theta_2)

    # Gravity
    G_1 = m_1*g*l_g1*math.cos(theta_1) + m_2*g*( l_1*math.cos(theta_1) + l_g2*math.cos(theta_1+theta_2))
    G_2 = m_2*g*l_g2*math.cos(theta_1+theta_2)

    C = Control(p, t)

    #define matrix
    M = np.matrix([[M_11, M_12],[M_21, M_22]])
    N = np.matrix([[N_1],[N_2]])
    G = np.matrix([[G_1],[G_2]])
    F = np.matrix([[C[0]],[C[1]]])

    IM = np.linalg.inv(M) # calc Inverse matrix
    A = (-1)*IM.dot(N+G-F) # F is right hand side of equations

    ddtheta_1, ddtheta_2 = A

    return [dtheta_1, ddtheta_1, dtheta_2, ddtheta_2]


t = np.arange(0.0, max_t, dt)
x0 = [0.1*math.pi, 0.0, 0.1*math.pi, 0.0]
p = odeint(Manipulator, x0, t)

v2m.video(p, dt, max_t, params, K, XD)
