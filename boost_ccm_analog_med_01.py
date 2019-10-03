# -*- coding: utf-8 -*-
#usar python3
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from math import log, sqrt, exp
from scipy.signal import lti, bode, lsim, TransferFunction, step, step2
from scipy.signal import cont2discrete, dbode
from tc_udemm import sympy_to_lti, lti_to_sympy

"""
        Boost Voltage-Mode, este es un modelo alternativo.
        Large Signal - derivado de una respuesta escalon del Boost.
        El modelo de baja senial, no representa bien el arranque
        del conversor. Esto es porque la ganancia alrededor del punto de trabajo
        es demasiado grande y no es representativa del arranque.
        Lo que ensayo es una respuesta escalon del boost en modo ccm
        y con el resultado deduzco una ecuacion de segundo orden.
"""


##########################################################
# Step response of the pulse by pulse converter model.   #
# Always on CCM. Step from 0.17 to 0.67 without feedback #
##########################################################
# From the simulation results:
# 12V input
# fn = 166.8
# Max_peak_value = 54
# Final_value = 35.7
# Input_step_value = 0.67

# 16V input
fn = 136.4
Max_peak_value = 56
Final_value = 35.6
Input_step_value = 0.53

# Auxiliary calcs
wn = fn * 2 * np.pi
Mp = Max_peak_value / Final_value - 1
log_mp_2 = (log(Mp))**2
psi = sqrt(log_mp_2/(np.pi**2+log_mp_2))
Mp2 = exp((-psi*np.pi)/sqrt(1-psi**2))

print(f'Mp: {Mp}, psi vale: {psi}, Mp revisado: {Mp2}')

#TF without constant
s = Symbol('s')

Plant_num = Final_value * wn**2 / Input_step_value
Plant_den = s**2 + 2 * psi * wn * s + wn**2
Plant_out = Plant_num/Plant_den

Plant_out_sim = Plant_out.simplify()
print ('Plant_out: ')
print (Plant_out_sim)

#####################################################
# Desde aca utilizo ceros y polos que entrego sympy #
#####################################################
planta = sympy_to_lti(Plant_out_sim)
print ("planta con sympy:")
print (planta)


##################################
# Respuesta escalon de la planta #
##################################
tiempo_de_simulacion = 0.1
t = np.linspace(0, tiempo_de_simulacion, num=2000)
t, y = step2(planta, T=t)
y = y * Input_step_value

fig, ax = plt.subplots()
ax.set_title('Respuesta Escalon Planta')
ax.set_ylabel('Vout')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y)

plt.tight_layout()
plt.show()

########################################
# Respuesta en Frecuencia de la Planta #
########################################
freq = np.arange(1, 100000, 1)
w, mag, phase = bode(planta, freq)

# fig.clear()
fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('Plant Tf - Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()
