#coding:utf-8
from sympy import *
from sympy.physics.mechanics import LagrangesMethod, Lagrangian
from sympy.physics.mechanics import ReferenceFrame, Particle, Point
from sympy.physics.mechanics import dynamicsymbols, kinetic_energy
from sympy.physics.mechanics import mprint, mlatex

#angles of joint 1, 2
theta1 = dynamicsymbols("theta_1")
theta2 = dynamicsymbols("theta_2")

#derivative of joint angles 1, 2
theta1_d = dynamicsymbols("theta_1",1)
theta2_d = dynamicsymbols("theta_2",1)

#parameters and time
m1,lg1,l1,m2,lg2,g,t = symbols("m_1 l_g1 l_1 m_2 l_g2 g t")

#potisions of each mass point on x-y coordinate
x1 = lg1 * cos(theta1)
y1 = lg1 * sin(theta1)
x2 = l1 * cos(theta1) + lg2 * cos(theta1+theta2)
y2 = l1 * sin(theta1) + lg2 * sin(theta1+theta2)

#velocities of each mass point on x-y coordinate
vx1 = diff(x1,t)
vy1 = diff(y1,t)
vx2 = diff(x2,t)
vy2 = diff(y2,t)

#definition of reference frame
N = ReferenceFrame("N")

#generation of mass point
P1 = Point("P_1")
P2 = Point("P_2")

#setting velocity for mass point
P1.set_vel(N,vx1 * N.x + vy1 * N.y)
P2.set_vel(N,vx2 * N.x + vy2 * N.y)

# make mass point
Pa1 = Particle("Pa_1",P1,m1)
Pa2 = Particle("Pa_2",P2,m2)

# Potential energy
Pa1.potential_energy = m1 * g * y1
Pa2.potential_energy = m2 * g * y2

# list up force applied to each mass
fl = [(P1,0*N.x),(P2,0*N.x)]

# make lagrangian
L = Lagrangian(N,Pa1,Pa2)

#generate equations
LM = LagrangesMethod(L,[theta1,theta2],forcelist = fl,frame=N)
me = LM.form_lagranges_equations()

#mprint( simplify(me) )
print( mlatex(simplify(me)) )

#print( mlatex(LM.rhs()) )

#simplyfied
#eq1 = simplify( LM.rhs() )
#mprint( eq1 )
