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
fn = 136.4
Max_peak_value = 56
Final_value = 35.6
Input_step_value = 0.53
sense_probe_alpha = 1.8 / (1.8 + 22)

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
sensado = sympy_to_lti(Plant_out_sim * sense_probe_alpha)
print ("planta con sympy:")
print (planta)


###############################################
# Respuesta escalon de la planta y el sensado #
###############################################
tiempo_de_simulacion = 0.1
t = np.linspace(0, tiempo_de_simulacion, num=2000)
t, y = step2(planta, T=t)
yp = y * Input_step_value
t, y = step2(sensado, T=t)
ys = y * Input_step_value


fig, ax = plt.subplots()
ax.set_title('Respuesta Escalon Planta [yellow] Sensado [Blue]')
ax.set_ylabel('Vout')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, yp, 'y')
ax.plot(t, ys, 'b')

plt.tight_layout()
plt.show()

##################################################
# Respuesta en Frecuencia de la Planta y Sensado #
##################################################
freq = np.arange(1, 100000, 1)
w, mag_p, phase = bode(planta, freq)
w, mag_s, phase = bode(sensado, freq)

# fig.clear()
fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag_p, 'y-', linewidth="1")
ax1.semilogx (w/(2*np.pi), mag_s, 'b-', linewidth="1")
ax1.set_title('Plant Tf - Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

"""    
        PID Analogico
        PID completo Tf = Kp + Ki/s + s Kd    Tf = 1/s * (s**2 Kd + s Kp + Ki)
        muy dificil analizar, basicamente polo en origen y dos ceros
        los dos ceros, segun los parametros elegidos, pueden llegar a ser complejos conjugados

        si fuese solo PI tengo Tf = 1/s * Kp * (s + Ki/Kp)
        esto es polo en origen w = 1; cero en w = Ki/Kp; ganancia Kp

        si fuese solo PD tengo Tf = Kd * (s + Kp/Kd)
        esto es cero en w = Kp/Kd y ganancia Kd

        Conclusion:
        elijo Kp para la ganancia media, ej 0dB Kp = 1
        elijo primer cero, ej 15.9Hz, Ki = 100
        elijo segundo cero, ej 1590Hz, Kd = 0.0001
"""
#################
# PID analogico #
#################
kp = 1
ki = 50
kd = 0.0003

Pid_out = kp + ki/s + s*kd
Pid_out_sim = Pid_out.simplify()

print ('Pid_out: ')
print (Pid_out_sim)

##############################################
# Grafico de Bode con Polos y Ceros de sympy #
##############################################
pid = sympy_to_lti(Pid_out_sim)
print ("PID con sympy:")
print (pid)

freq = np.arange(1, 1000000, 1)

w, mag, phase = bode(pid, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('PID Tf - Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()

#######################################################
# Multiplico Transferencias para OpenLoop y CloseLoop #
#######################################################
c = lti_to_sympy(pid)
p = lti_to_sympy(sensado)

ol = c * p

open_loop = sympy_to_lti(ol)
open_loop = TransferFunction(open_loop.num, open_loop.den)   #normalizo

w, mag, phase = bode(open_loop, freq)

fig, (ax1, ax2) = plt.subplots(2,1)
ax1.semilogx (w/(2*np.pi), mag, 'b-', linewidth="1")
ax1.set_title('Sensado - Open Loop Tf - Magnitude')

ax2.semilogx (w/(2*np.pi), phase, 'r-', linewidth="1")
ax2.set_title('Phase')

plt.tight_layout()
plt.show()


######################################
# Realimento y veo Respuesta escalon #
######################################
# setpoint para 36V
setpoint = 36 * sense_probe_alpha

cl = ol / (1 + ol)
close_loop = sympy_to_lti(cl)
close_loop = TransferFunction(close_loop.num, close_loop.den)   #normalizo

t = np.linspace(0, tiempo_de_simulacion, num=2000)
t, y = step2(close_loop, T=t)
y = y * setpoint / sense_probe_alpha


fig.clear()
fig, ax = plt.subplots()
ax.set_title('Respuesta escalon Close Loop')
ax.set_ylabel('Vout')
ax.set_xlabel('Tiempo [s]')
ax.grid()
ax.plot(t, y)

plt.tight_layout()
plt.show()

