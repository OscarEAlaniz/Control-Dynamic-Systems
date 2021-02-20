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
u=1


# funcion que retorna dx/dt

def motorDC(x,t,R,L,J,B,Km,Ka,u):
    w=x[0]
    i=x[1]

    dwdt = (-B/J)*w + (Km/J)*i
    didt =(-R/L)*i - (Ka/L)*w + (1/L)*u
    dxdt= [dwdt,didt]

    return dxdt


# condiciones iniciales

x0=[0,0]

# times point

t = np.linspace(0,5)



# solucion de ODE

x  = odeint(motorDC,x0,t,args=(R,L,J,B,Km,Ka,u))

#plot resultados

plt.plot(t,x[:,0],'b', label=r'$\omega(t) $')
plt.plot(t,x[:,1],'r', label=r'$i(t) $')
plt.ylabel('respuesta')
plt.xlabel('time')
plt.legend(loc='best')
plt.show()










