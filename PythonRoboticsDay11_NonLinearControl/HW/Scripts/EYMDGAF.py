import numpy as np

class Kinematics:
	"""Class for Jacobian and other functions of Jacobian
	theta_1 and theta_2 represents the joint angles
	L1 and L2 are the lengths of the joint"""
	def __init__(self, p, L):
		self.theta_1 = p[0]
		self.dtheta_1 = p[1]
		self.theta_2 = p[2]
		self.dtheta_2 = p[3]
		self.L1 = L[0]
		self.L2 = L[1]

	@property
	def Forward_Kinematics(self):
		x2 = self.L1*np.cos(self.theta_1) + self.L2*np.cos(self.theta_1 + self.theta_2)
		y2 = self.L1*np.sin(self.theta_1) + self.L2*np.sin(self.theta_1 + self.theta_2)
		
		return np.array([x2,y2])

	def Inverse_Kinematics(self, X):
		L = np.sqrt(X[0]**2 + X[1]**2)
		th_1 = np.arctan2(X[1],X[0]) - np.arccos((self.L1**2 + np.power(L,2) - self.L2**2)/(2*self.L1*L))
		th_2 = np.pi - np.arccos((self.L1**2 + self.L2**2 - L**2)/(2*self.L1*self.L2))

		return np.array([th_1,th_2])

	@property
	def Jacobian(self):
		J11 = - self.L1*np.sin(self.theta_1) - self.L2*np.sin(self.theta_1+self.theta_2)
		J12 = - self.L2*np.sin(self.theta_1+self.theta_2)
		J21 =   self.L1*np.cos(self.theta_1) + self.L2*np.cos(self.theta_1+self.theta_2)
		J22 =   self.L2*np.cos(self.theta_1+self.theta_2)

		return np.array([[J11, J12],[J21, J22]])

	@property
	def diff_Jacobian(self):
		dJ11 = - self.L1*self.dtheta_1*np.cos(self.theta_1) - self.L2*(self.dtheta_1+self.dtheta_2)*np.cos(self.theta_1+self.theta_2)
		dJ12 = - self.L2*(self.dtheta_1+self.dtheta_2)*np.cos(self.theta_1+self.theta_2)
		dJ21 = - self.L1*self.dtheta_1*np.sin(self.theta_1) - self.L2*(self.dtheta_1+self.dtheta_2)*np.sin(self.theta_1+self.theta_2)
		dJ22 = - self.L2*(self.dtheta_1+self.dtheta_2)*np.sin(self.theta_1+self.theta_2)

		return np.array([[dJ11, dJ12],[dJ21, dJ22]])
	
	@property
	def Inverse_Jacobian(self):
		#Singular point
		if -0.001 < self.L1*self.L2*np.sin(self.theta_2) < 0:
		    detJ = -0.001
		    print('Singular point!')
		elif 0 <= self.L1*self.L2*np.sin(self.theta_2) < 0.001:
		    detJ =  0.001
		    print('Singular point!')
		else:
		    detJ = self.L1*self.L2*np.sin(self.theta_2)

		IJ11 = ( 1/detJ )*(  self.L2*np.cos(self.theta_1+self.theta_2) )
		IJ12 = ( 1/detJ )*(  self.L2*np.sin(self.theta_1+self.theta_2) )
		IJ21 = ( 1/detJ )*( -self.L1*np.cos(self.theta_1) - self.L2*np.cos(self.theta_1+self.theta_2) )
		IJ22 = ( 1/detJ )*( -self.L1*np.sin(self.theta_1) - self.L2*np.sin(self.theta_1+self.theta_2) )

		return np.array([[IJ11, IJ12],[IJ21, IJ22]])


class Feedback(object):
	"""Feedback?"""
	def __init__(self, p, L, Kp, Kv):
		self.p = p
		self.L = L
		self.Kp = np.array(Kp)
		self.Kv = np.array(Kv)
		self.Kinematics = Kinematics(self.p,self.L)

	def Task_Space(self, target_position, target_velocity, target_acceleration = [0,0]):
		PD = self.Kp*(np.array(target_position) - self.Kinematics.Forward_Kinematics) + self.Kv*(np.array(target_velocity) - self.Kinematics.Jacobian.dot(np.array([self.p[1],self.p[3]])))
		dJdTh = self.Kinematics.diff_Jacobian.dot(np.array([self.p[1],self.p[3]]))
		rv = self.Kinematics.Inverse_Jacobian.dot(np.array(target_acceleration) + PD-dJdTh)

		return rv
	
	def Joint_Space(self, target_position, target_velocity, target_acceleration = [0,0]):
		PD = self.Kp*(self.Kinematics.Inverse_Kinematics(target_position) - np.array([self.p[0],self.p[2]])) + self.Kv*(np.array(target_velocity) - np.array([self.p[1],self.p[3]]))
		rv = np.array(np.array(target_acceleration) + PD)
		
		return rv
	