from sympy.physics.continuum_mechanics.beam import Beam
from sympy import symbols, Piecewise
import numpy as np
import matplotlib as plt

# Beam Properties:
E=690e9         #Young's modulus
rho=2720.       #density, in kg/m^3
L=1.203325      #total length, in meters (X-AXIS)
b=0.3048        #beam depth, in meters (Y-AXIS)
h=0.003175      #beam width, in meters (Z-AXIS)
A=b*h           #beam cross sectional area 
I=b*h**3/12     #beam moment of inertia
mass=rho*L*A    #total mass, in kg

# Simulating from the TOP-DOWN orientation of the beam:
b = Beam(L,E,I)
i=1

while i <10:
    b.apply_load(1,L/i,-1) # creates a point load at RXN_1 
    #NB: save this force as a variable, apply it in the next while loop!
    b.load
    
    b.bc_deflection = [(0,0),(0,0)]# fixed at one end, so no vertical deflection or rotation
    b.bc_slope.append((0,0))
    R,M = symbols ('R,M')
    b.apply_load (R,0,-1)
    b.apply_load (M, 0, -2)

    b.solve_for_reaction_loads(R,M)
    b.reaction_loads 
    b.load
    b.shear_force()
    b.bending_moment()
    #see line 1623 in documentation about plotting for changing data; probably add pandas for data management
    b.plot_deflection(subs = {E: E, I: I})
    i = i +1
# sim for loop for applying forces ramp up, then ramp down!!!
# reprint graph for each time!