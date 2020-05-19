# Mass sliding from ramp

from sympy import *
from sympy.physics.mechanics import LagrangesMethod, Lagrangian
from sympy.physics.mechanics import ReferenceFrame, Particle, Point
from sympy.physics.mechanics import dynamicsymbols, kinetic_energy
from sympy.physics.mechanics import mprint, mlatex

# Position on ramp
s = dynamicsymbols("s")
var = [s]

# Derivative of position
#s_d = dynamicsymbols("s",1)

# Constants and Time
m, alpha, friction, g, t = symbols("m alpha friction g t")

# Projection on X-Y plane
x1 = s * cos(alpha)
y1 = s * sin(alpha)


# Velocity on X-Y plane
vx1 = diff(x1,t)
vy1 = diff(y1,t)

# Setting Reference Frames
N = ReferenceFrame("N")

# Point mass assumption
P = Point("P")

# Velocity of Point mass in X-Y plane
P.set_vel(N,vx1 * N.x + vy1 * N.y)

# Making a particle from point mass
Pa = Particle("P",P,m)

# Potential energy of system
Pa.potential_energy = m * g * y1

# Non-restorative forces
fl = [(P,friction*cos(alpha)*N.x + friction*sin(alpha)*N.y)]

# Setting-up Lagrangian
L = Lagrangian(N,Pa)

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
