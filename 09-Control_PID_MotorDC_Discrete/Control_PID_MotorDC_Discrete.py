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
Tstop= 5
Ts=0.005 # tiempo de muestreo 

N = int(Tstop/Ts) # longitud de simulacion
e = np.zeros(N+2)
u = np.zeros(N+2)
w = np.zeros(N+2)
i = np.zeros(N+2)
e[0] = 0
u[0] = 0
w[0] = 0
i[0] = 0               # condicion inicial


Kp=20.5
Ki=0.5
Kd=0.1



ref =0.3

# simulacion controlador 

for k in range(N+1):

     

    e[k] = ref - w[k]
    u[k] = u[k-1]+ Kp*(e[k] - e[k-1])+Kd*((e[k] - e[k-1])/Ts)+Ki*e[k]

    w[k+1] = w[k] + Ts*((-B/J)*w[k] + (Km/J)*i[k])
    w[k]=w[k+1]
    i[k+1] = i[k] + Ts*((-R/L)*i[k] - (Ka/L)*w[k] + (1/L)*u[k])
    i[k]=i[k+1]
  

t = np.arange(Tstar,Tstop+2*Ts, Ts)




#plot resultados
plt.title("Control PID Motor DC Discrete, $T_s=0.005$")
plt.plot([0,0,0,5],[0,0,0.3,0.3],'b-',label=r'$Set point$')
plt.plot(t,w,marker='*',linestyle=':',color='r', label=r'$\omega(t) $')

#plt.plot(t,x[:,1],'r', label=r'$i(t) $')

plt.ylabel('$\omega[rpm]$')
plt.xlabel('time')
plt.legend(loc='best')
plt.show()
