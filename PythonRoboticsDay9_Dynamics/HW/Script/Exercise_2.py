# Manipulator

from sympy import *
from sympy.physics.mechanics import LagrangesMethod, Lagrangian
from sympy.physics.mechanics import ReferenceFrame, Particle, Point
from sympy.physics.mechanics import dynamicsymbols, kinetic_energy
from sympy.physics.mechanics import mprint, mlatex

# Anglular position of Arm joint, Total Length of Arm joint
theta = dynamicsymbols("theta")
l = dynamicsymbols("l")

var = [theta,l]

# Constants and Time
m_theta, m_l, I, lg_1, torque, sliding_force, g, t = symbols("m_theta m_l I lg_1 torque sliding_force g t")

# Projection of COM on X-Y plane
x1 = lg_1 * cos(theta)
y1 = lg_1 * sin(theta)

x2 = l * cos(theta)
y2 = l * sin(theta)

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
Pa_1 = Particle("P_1",P_1,m_theta)
Pa_2 = Particle("P_2",P_2,m_l)

# Potential energy of system
Pa_1.potential_energy = m_theta * g * y1
Pa_2.potential_energy = m_l * g * y2

# Non-restorative forces
f_theta = (P_1, torque*( -sin(theta) * N.x + cos(theta) * N.y)/lg_1)
f_sliding = (P_2, sliding_force * (cos(theta) * N.x + sin(theta) * N.y))
fl = [f_theta, f_sliding]

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
