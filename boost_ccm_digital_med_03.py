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

# print(f'Mp: {Mp}, psi vale: {psi}, Mp revisado: {Mp2}')

#TF without constant
s = Symbol('s')

Plant_num = Final_value * wn**2 / Input_step_value
Plant_den = s**2 + 2 * psi * wn * s + wn**2
Plant_out = Plant_num/Plant_den

Plant_out_sim = Plant_out.simplify()
# print ('Plant_out: ')
# print (Plant_out_sim)

#####################################################
# Desde aca utilizo ceros y polos que entrego sympy #
#####################################################
# planta = sympy_to_lti(Plant_out_sim)
sensado = sympy_to_lti(Plant_out_sim * sense_probe_alpha / 3.3)
# print ("planta con sympy:")
# print (planta)

##########################################################
# Convierto Planta Digital - Sensado Digital en realidad #
# por Tustin                                             #
##########################################################
Fsampling = 24000
Tsampling = 1 / Fsampling
planta_dig_tustin_n, planta_dig_tustin_d, td = cont2discrete((sensado.num, sensado.den), Tsampling, method='tustin')

#normalizo con TransferFunction
print ("Planta Digital:")
planta_dig_tustin = TransferFunction(planta_dig_tustin_n, planta_dig_tustin_d, dt=td)
print (planta_dig_tustin)


################################################
# Respuesta escalon de la planta punto a punto #
# entrando con Duty propuesto como escalon     #
################################################
tiempo_de_simulacion = 0.2
print('td:')
print (td)
t = np.arange(0, tiempo_de_simulacion, td)

# Planta Digital por Tustin
b_planta = np.transpose(planta_dig_tustin_n)
a_planta = np.transpose(planta_dig_tustin_d)

Duty = Input_step_value
vin_plant = np.ones(t.size) * Duty
vout_plant = np.zeros(t.size)

# for i in range(2, len(vin_plant)):        
#     ########################################
#     # aplico la transferencia de la planta #
#     ########################################
#     vout_plant[i] = b_planta[0]*vin_plant[i] \
#                     + b_planta[1]*vin_plant[i-1] \
#                     + b_planta[2]*vin_plant[i-2] \
#                     - a_planta[1]*vout_plant[i-1] \
#                     - a_planta[2]*vout_plant[i-2]

# fig, ax = plt.subplots()
# ax.set_title('Respuesta de la Planta Open Loop')
# ax.set_ylabel('Vout')
# ax.set_xlabel('Tiempo en muestras')
# ax.grid()
# ax.plot(t, vin_plant, 'r')
# ax.plot(t, vout_plant, 'c')
# plt.tight_layout()
# plt.show()


############################
# PID Digital              #
# ki_dig = ki / Fsampling  #
# kp_dig = kp - ki_dig / 2 #
# kd_dig = kd * Fsampling  #
############################
# con undersampling = 100 y 50
ki_dig = 6 / 128
kp_dig = 1 / 128
kd_dig = 0
# con undersampling = 100 y 50
# ki_dig = 6 / 128
# kp_dig = 1
# kd_dig = 0
# con undersampling = 10
# ki_dig = 1 / 128
# kp_dig = 42 / 128
# kd_dig = 3

k1 = kp_dig + ki_dig + kd_dig
k2 = -kp_dig - 2*kd_dig
k3 = kd_dig

## este es el pid digital
b_pid = [k1, k2, k3]
a_pid = [1, -1]
print ("")
print (f"kp_dig: {kp_dig} ki_dig: {ki_dig} kd_dig: {kd_dig}")
print ("")

pid_dig = TransferFunction(b_pid, a_pid, dt=td)
print ("PID Digital:")
print (pid_dig)

#########################################
# Realimento punto a punto con setpoint #
#########################################
# Respuesta escalon de la planta punto a punto
tiempo_de_simulacion = 0.5
print('td:')
print (td)
t = np.arange(0, tiempo_de_simulacion, td)

# Planta Digital por Tustin
b_planta = np.transpose(planta_dig_tustin_n)
a_planta = np.transpose(planta_dig_tustin_d)

vout_plant = np.zeros(t.size)
vin_plant = np.zeros(t.size)

############################################
# Armo la senial que quiero en el SETPOINT #
############################################
vin_setpoint = np.ones(t.size) * 36 * sense_probe_alpha * 1000 / 3.3
vin_setpoint.astype(int)
vin_setpoint = vin_setpoint / 1000

d = np.zeros(t.size)
error = np.zeros(t.size)
max_d_pwm = 0.85
under_roof = 49
undersampling = 0
integral_term = np.zeros(t.size)

for i in range(2, len(vout_plant)):
    ###################################################
    # primero calculo el error, siempre punto a punto #
    ###################################################
    dummy_adc_out = int(vout_plant[i-1] * 1000)
    dummy_adc_out = dummy_adc_out / 1000
    # vout_plant[i - 1] = dummy_adc_out / 1000    
    
    # error[i] = vin_setpoint[i] - vout_plant[i-1]
    error[i] = vin_setpoint[i] - dummy_adc_out

    #############################################################
    # aplico lazo PID y ajusto los maximo y minimos que permito #
    #############################################################
    if undersampling < under_roof:
        #nada
        undersampling = undersampling + 1
        d[i] = d[i-1]
        integral_term[i] = integral_term[i-1]
    else:
        undersampling = 0

        # desarmo k1 y el termino integral para tener mejor definicion
        integral_term[i] = integral_term[i-1] + ki_dig * error[i]
        k1 = kp_dig * error[i] + integral_term[i] + kd_dig * error[i]
        if integral_term[i] > 0.001:
            integral_term[i] = 0
            
        d[i] = k1 + b_pid[1] * error[i-1] + b_pid[2] * error[i-2] - a_pid[1] * d[i-1]
        # d[i] = b_pid[0] * error[i] + b_pid[1] * error[i-1] + b_pid[2] * error[i-2] - a_pid[1] * d[i-1]
        dummy_d = int(d[i] * 1000)
        d[i] = dummy_d / 1000
        
    if d[i] > max_d_pwm:
        d[i] = max_d_pwm

    if d[i] < 0:
        d[i] = 0
        
    ########################################
    # aplico la transferencia de la planta #
    ########################################
    vin_plant[i] = d[i]
    vout_plant[i] = b_planta[0]*vin_plant[i] \
                    + b_planta[1]*vin_plant[i-1] \
                    + b_planta[2]*vin_plant[i-2] \
                    - a_planta[1]*vout_plant[i-1] \
                    - a_planta[2]*vout_plant[i-2]
               

fig, ax = plt.subplots()
ax.set_title('Respuesta Realimentada punto a punto')
ax.set_ylabel('Vout')
ax.set_xlabel('Tiempo en muestras')
ax.grid()
ax.plot(t, d, 'r')
ax.plot(t, error, 'g')
ax.plot(t, vin_setpoint, 'y')
ax.plot(t, integral_term, 'b')
# ax.stem(t, vout_plant / sense_probe_alpha)
# ax.plot(t, vout_plant / sense_probe_alpha, 'c')
ax.plot(t, vout_plant, 'c')
plt.tight_layout()
plt.show()
