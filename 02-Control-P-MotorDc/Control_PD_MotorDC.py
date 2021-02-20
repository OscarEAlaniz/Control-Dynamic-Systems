import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt


# parametros de motor


R = 1
L = 0.5
J = 0.01
B= 0.1
Km=0.01
Ka=0.01


# funcion que retorna dx/dt

def motorControlP(x,t,R,L,J,B,Km,Ka,Kp):
    w=x[0]
    i=x[1]
    # Step
    if t < 1:
        ref = 0
    else:
        ref =0.3

    # Control P
    e = ref - w
    u=Kp*e

    dwdt = (-B/J)*w + (Km/J)*i
    didt =(-R/L)*i - (Ka/L)*w + (1/L)*u
    dxdt= [dwdt,didt]

    return dxdt









# condiciones iniciales
x0=[0,0]

# times point
Tstar = 0
Tstop = 3
step = 0.001 

t = np.arange(Tstar,Tstop, step)

# ganacia de control P
Kp=1500

# solucion de ODE
x  = odeint(motorControlP,x0,t,args=(R,L,J,B,Km,Ka,Kp))



#plot resultados
plt.plot([0,1,1,3],[0,0,0.3,0.3],'b-',label=r'$Set point$')
plt.plot(t,x[:,0],'r', label=r'$\omega(t) $')

#plt.plot(t,x[:,1],'r', label=r'$i(t) $')

plt.ylabel('$\omega[rpm]$')
plt.xlabel('time')
plt.legend(loc='best')
plt.show()