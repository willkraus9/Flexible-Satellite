# REx Lab: Will Kraus 
# November 2023

import matplotlib as plt
import numpy as np

# INPUTS
l = 1 # length of beam, in meters
w = 1 # width of beam, in meters
h = 1 # thickness of beam, in meters
m = 1 # mass of entire beam, in kilograms
E = 1 # Young's Modulus, in GPa
A = w*h # Cross-sectional area, in m^2
I = (1/12)*w*h**3 #assuming rectangular, moment of inertia about cross section, in kg*m**2
d = m/(l*h*w) #density, in kg/m**3

# FREQUENCIES
# Source for calculations: 
    # https://www.vibrationdata.com/tutorials2/beam.pdf
    # https://vlab.amrita.edu/?sub=3&brch=175&sim=1080&cnt=1
    # https://www.engineeringtoolbox.com/area-moment-inertia-d_1328.html
W = (1/(2*np.pi))*np.sqrt((3*E*I)/(m*l^3)) #natural frequency
first_frequency = (1/(2*np.pi))*(3.516 / l^2)*np.sqrt((E*I / (m*l)))
second_frequency = first_frequency*6.268
third_frequency = first_frequency*17.456








