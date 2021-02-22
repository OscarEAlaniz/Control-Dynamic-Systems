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
Tstop= 60
Ts=0.001




              # condicion inicial
N = int(Tstop/Ts) # longitud de simulacion

q1= np.zeros(N+2)
q1p = np.zeros(N+2)
q2 = np.zeros(N+2)
q2p = np.zeros(N+2)
#tau1= np.zeros(N+2)
#tau2 = np.zeros(N+2)
e1 =np.zeros(N+2)
e1p=np.zeros(N+2)
e2 = np.zeros(N+2)
e2p = np.zeros(N+2)
seno = np.zeros(N+2)
sen = np.zeros(N+2)
coseno = np.zeros(N+2)
cose = np.zeros(N+2)

q1[0] = 0
q1p[0] = 0
q2[0] = 0
q2p[0] = 0    



e=[]
ep=[]
Kv=20
Kp=100

# simulacion dinamica
t = np.arange(Tstar,Tstop+2*Ts, Ts)

for k in range(N+1):
    period=2
    amp1=0.1
    amp2=0.1
    fact=0.0005
    seno[k]=sin(fact*k)
    sen[k]=amp1*seno[k]
    coseno[k]=cos(fact*k)
    cose[k]=amp2*coseno[k]
 
    qd =[amp1*seno[k],amp2*coseno[k]]
    qdp =[fact*amp1*cose[k],-fact*amp2*seno[k]]
    qdpp=[-fact*fact*amp1*seno[k],-fact*fact*amp2*coseno[k]]
    # señales de error
    e1[k]=qd[0]-q1[k]
    e2[k]=qd[1]-q2[k]
    e1p[k]=qdp[0]-q1p[k]
    e2p[k]=qdp[1]-q2p[k]

    e=[e1[k],e2[k]]
    ep=[e1p[k],e2p[k]]


    # calculo de los del controlador 
    #matriz inercia
    M11=(m1+m2)*a1*a1 + m2*a2*a2+2*m2*a1*a2*cos(q2[k])
    M12=m2*a2*a2+(m2*a1*a2*cos(q2[k]))
    M22=m2*a2*a2
    N1= -m2*a1*a2*(2*q1p[k]*q2p[k]+q2p[k]*q2p[k])*sin(q2[k])
    N1=N1+(m1+m2)*g0*a1*cos(q1[k])+m2*g0*a2*cos(q1[k]+q2[k])
    N2=m2*a1*a2*q2p[k]*q2p[k]*sin(q2[k])+m2*g0*a2*cos(q1[k]+q2[k])
    # calculo de torques de control 
    s1=qdpp[0]+Kv*ep[0]+Kp*e[0]
    s2=qdpp[1]+Kv*ep[1]+Kp*e[1]
    tau1=M11*s1+M12*s2+N1
    tau2=M12*s1+M22*s2+N2


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
    q1p[k+1] =q1p[k]+Ts*(MI11*(-N1+tau1)+MI12*(-N2+tau2))
    q2p[k+1] =q2p[k]+Ts*(MI12*(-N1+tau1)+MI22*(-N2+tau2))
    

    q1[k]=q1[k+1]
    q2[k]=q2[k+1]
    q1p[k]=q1p[k+1]
    q2p[k]=q2p[k+1]
    





#plot resultados
plt.figure(1)
plt.subplot(211)
plt.title("Robot 2GL Posición ")
plt.plot(t,sen,'b', label=r'$qd_1(t) $')
plt.plot(t,q1,'r', label=r'$q_1(t) $')
plt.ylabel('[m]')
plt.legend(loc='best')
plt.subplot(212)
plt.plot(t,cose,'b', label=r'$qd_2(t) $')
plt.plot(t,q2,'r', label=r'$q_2(t) $')
plt.ylabel('[m\s]')
plt.xlabel('time')
plt.legend(loc='best')



plt.figure(2)
plt.title("señales de error")
plt.plot(t,e1,'b', label=r'$e(t)$')
plt.plot(t,e2,'r', label=r'$e(t)$')
plt.show()
