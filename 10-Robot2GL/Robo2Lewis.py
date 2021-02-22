from math import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import matplotlib.gridspec as gridspec

#parametros

m1=1
m2=1
a1=1
a2=1
g0=9.81
alf01=8
alf02=2
alf11=1.1
alf12=1
alf21=0.9
alf22=0.95
B11=50
B12=55
B21=65
B22=60




#Valores de Matrices 



# parametros de simulacion
Tstar = 0
Tstop= 10
Ts=0.001




              # condicion inicial
N = int(Tstop/Ts) # longitud de simulacion

q1= np.zeros(N+2)
q1p = np.zeros(N+2)
q2 = np.zeros(N+2)
q2p = np.zeros(N+2)
tau1= np.zeros(N+2)
tau2 = np.zeros(N+2)
q1[0] = 0
q1p[0] = 0
q2[0] = 0
q2p[0] = 0    
tau1[0] = 1
tau2[0] = 1   



# simulacion dinamica

for k in range(N+1):
    #matriz inercia
    M11=(m1+m2)*a1*a1 + m2*a2*a2+2*m2*a1*a2*cos(q2[k])
    M12=m2*a2*a2+(m2*a1*a2*cos(q2[k]))
    M22=m2*a2*a2
    N1= -m2*a1*a2*(2*q1p[k]*q2p[k]+q2p[k]*q2p[k])*sin(q2[k])
    N1=N1+(m1+m2)*g0*a1*cos(q1[k])+m2*g0*a2*cos(q1[k]+q2[k])
    N2=m2*a1*a2*q2p[k]*q2p[k]*sin(q2[k])+m2*g0*a2*cos(q1[k]+q2[k])
    #matriz inversa
    detM=M11*M22-(M12*M12)
    MI11=M22/detM
    MI12=-M12/detM
    MI22=M11/detM
     
     #ecuaicones de estado

    q1[k+1] =q1[k]+Ts*q1p[k]
    q2[k+1] = q2[k]+Ts*q2p[k]
    q1p[k+1] =q1p[k]+Ts*(MI11*(-N1+tau1[k])+MI12*(-N2+tau2[k]))
    q2p[k+1] =q2p[k]+Ts*(MI12*(-N1+tau1[k])+MI22*(-N2+tau2[k]))
    

    q1[k]=q1[k+1]
    q2[k]=q2[k+1]
    q1p[k]=q1p[k+1]
    q2p[k]=q2p[k+1]
    

t = np.arange(Tstar,Tstop+2*Ts, Ts)



#plot resultados
plt.figure(1)
plt.subplot(211)
plt.title("Robot 2GL Posición y Velocidad")
plt.plot(t,q1,'b', label=r'$q_1(t) $')
plt.plot(t,q2,'r', label=r'$q_2(t) $')
plt.ylabel('[m]')
plt.legend(loc='best')
plt.subplot(212)
plt.plot(t,q1p,'b', label=r'$\dot{q}_1(t) $')
plt.plot(t,q2p,'r', label=r'$\dot{q}_2(t) $')
plt.ylabel('[m\s]')
plt.xlabel('time')
plt.legend(loc='best')
plt.show()
