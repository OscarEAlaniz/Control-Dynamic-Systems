from math import *
import numpy as np
import matplotlib.pyplot as plt


#parametros

m1=3
m2=2.3
l1=1.1
l2=1
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







# parametros de braz robot 2GL
a=l2*l2*m2+l1*(m1+m2)
b=2*l1*l2*m2
c=l2*l2*m2
d=(m1+m2)*(l1*g0)
e=m2*l2*g0

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
t1= np.zeros(N+2)
t2 = np.zeros(N+2)
q1[0] = 0
q1p[0] = 0
q2[0] = 0
q2p[0] = 0    
t1[0] = 1
t2[0] = 1   





def sgn(x):
    if x>0: 
        return 1
    else:
        if x<0: 
            return -1 
        else: 
            return  0




# simulacion dinamica

for k in range(N+1):

    
    """
    # Control PD
    """








    """
    #Dinamica del Robot 2GL lazo cerrado
    """
    #matriz inercia
    M11=a+b*cos(q2[k])
    M12=c+((b*cos(q2[k]))/2)
    M22=c
    detM=M11*M22-(M12*M12)

    #matriz coriolis
    V1=(-b*q1p[k]*q2p[k]*sin(q2[k]))-((b*q2p[k]*q2p[k]/2)*sin(q2[k]))
    V2=((b*q1p[k]*q1p[k]/2)*sin(q2[k]))

    #pares gravitatorios
    G1=d*cos(q1[k])+e*cos(q1[k]+q2[k])
    G2=e*cos(q1[k]+q2[k])

    #friccion
    F1=(alf01+alf11*exp(-B11*fabs(q1p[k]))+alf21*(1-exp(-B21*fabs(q1p[k]))))*sgn(q1p[k])
    F2=(alf02+alf12*exp(-B12*fabs(q2p[k]))+alf22*(1-exp(-B22*fabs(q2p[k]))))*sgn(q2p[k])

    S1=t1[k]-V1-G1-F1
    S2=t2[k]-V2-G2-F2

    q1[k+1] =q1[k]+Ts*q1p[k]
    q1p[k+1] =q1p[k]+Ts*((M22*S1-M12*S2)/detM)
    q2[k+1] = q2[k]+Ts*q2p[k]
    q2p[k+1] =q2p[k]+Ts*((-M12*S1+M11*S2)/detM)
    

    q1[k]=q1[k+1]
    q1p[k]=q1p[k+1]
    q2[k]=q2[k+1]
    q2p[k]=q2p[k+1]
    

t = np.arange(Tstar,Tstop+2*Ts, Ts)



#plot resultados
plt.figure(1)
plt.title("Robot 2GL Posición")
plt.plot(t,q1,'b', label=r'$q_1(t) $')
plt.plot(t,q2,'r', label=r'$q_2(t) $')
plt.ylabel('[m]')
plt.xlabel('time')
plt.legend(loc='best')


plt.figure(2)
plt.title("Robot 2GL  Velocidad")
plt.plot(t,q1p,'b', label=r'$\dot{q}_1(t) $')
plt.plot(t,q2p,'r', label=r'$\dot{q}_2(t) $')
plt.ylabel('[m\s]')
plt.xlabel('time')
plt.legend(loc='best')
plt.show()