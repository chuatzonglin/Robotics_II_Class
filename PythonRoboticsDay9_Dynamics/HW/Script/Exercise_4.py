# Manipulator

from sympy import *
from sympy.physics.mechanics import LagrangesMethod, Lagrangian
from sympy.physics.mechanics import ReferenceFrame, Particle, Point
from sympy.physics.mechanics import dynamicsymbols, kinetic_energy
from sympy.physics.mechanics import mprint, mlatex

# Anglular position of Arm joint, Total Length of Arm joint
theta_1 = dynamicsymbols("theta_1")
theta_2 = dynamicsymbols("theta_2")

var = [theta_1,theta_2]

# Constants and Time
m_1, m_2, I_1, lg_1, l_1 , lg_2, torque_1, torque_2, F_x, g, t = symbols("m_1 m_2 I_1 lg_1 l_1 lg_2 torque_1 torque_2 F_x g t")

# Projection of COM on X-Y plane
x1 = lg_1*cos(theta_1)
y1 = lg_1*sin(theta_1)

x2 = l_1*cos(theta_1) + lg_2*cos(theta_1+theta_2)
y2 = l_1*sin(theta_1) + lg_2*sin(theta_1+theta_2)

# Velocity on X-Y plane
vx1 = diff(x1,t)
vy1 = diff(y1,t)

vx2 = diff(x2,t)
vy2 = diff(y2,t)

# Setting Reference Frames
N = ReferenceFrame("N")

# Lumped mass abstraction
P_1 = Point("P_1")
P_2 = Point("P_2")

# Velocity of Point mass in X-Y plane
P_1.set_vel(N,vx1 * N.x + vy1 * N.y)
P_2.set_vel(N,vx2 * N.x + vy2 * N.y)

# Making a particle from point mass
Pa_1 = Particle("P_1",P_1,m_1)
Pa_2 = Particle("P_2",P_2,m_2)

# Potential energy of system
Pa_1.potential_energy = m_1 * g * y1
Pa_2.potential_energy = m_2 * g * y2

# Non-restorative forces
f_1 = (P_1, torque_1*( -sin(theta_1) * N.x + cos(theta_1) * N.y)/lg_1)#torque_1*( -sin(theta_1) * N.x + cos(theta_1) * N.y)/lg_1)
f_2 = (P_2, 0*N.x)#torque_2*( -sin(theta_1 + theta_2) * N.x + cos(theta_1 + theta_2) * N.y)/lg_2)
fl = [f_1, f_2]

# Setting-up Lagrangian
L = Lagrangian(N,Pa_1,Pa_2)

# Generation Equation of Motion
LM = LagrangesMethod(L,var,forcelist = fl,frame=N)
EOM = LM.form_lagranges_equations()


""" Printing results """
mprint( simplify(EOM) )
#print( mlatex(simplify(me)) )

#print( mlatex(LM.rhs()) )

#simplyfied
#eq1 = simplify( LM.rhs() )
#mprint( eq1 )
