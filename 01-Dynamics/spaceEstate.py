import control
import scipy.signal as sig
import numpy as np
import matplotlib.pyplot as plt


# parametros de simulacion

x0 = [0,0]

Tstar = 0
Tstop = 30
step = 1 

t = np.arange(Tstar,Tstop, step)
K =3
T = 4

# State space model
A = [[-1/T,0],[0, 0]]

B = [[K/T],[0]]

C = [[1,0]]

D = 0


sys = sig.StateSpace(A,B,C,D)

# STEP RESPONSE

t, x1 = sig.step(sys, x0, t)

plt.plot(t,x1)

plt.title("1. Orde Dynamic system space State")
plt.xlabel('t[s]')
plt.ylabel('y(t)')
plt.grid()
plt.show()




