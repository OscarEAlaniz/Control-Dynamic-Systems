import numpy as np
import matplotlib.pyplot as plt


# parametros de motor


R = 1
L = 0.5
J = 0.01
B= 0.1
Km=0.01
Ka=0.01

# parametros de simulacion
Tstar = 0
Tstop= 3
Ts=0.1
uk= 1
wk=0
ik=0               # condicion inicial
N = int(Tstop/Ts) # longitud de simulacion
W=[]
I=[]

W.append(wk)
I.append(ik)

# simulacion dinamica

for k in range(N):
    wk1 = wk + Ts*((-B/J)*wk + (Km/J)*ik)
    wk=wk1
    ik1 = ik + Ts*((-R/L)*ik - (Ka/L)*wk + (1/L)*uk)
    ik=ik1
    W.append(wk1)
    I.append(ik1)


t = np.arange(Tstar,Tstop+Ts, Ts)



#plot resultados
plt.title("Step Response Motor DC Discrete")
plt.plot(t,W,'b', label=r'$\omega(t) $')
plt.plot(t,I,'r', label=r'$i(t) $')
plt.ylabel('respuesta')
plt.xlabel('time')
plt.legend(loc='best')
plt.show()
