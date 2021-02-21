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
e1 =np.zeros(N+2)
e1p=np.zeros(N+2)
e2 = np.zeros(N+2)
e2p = np.zeros(N+2)
#x=np.zeros(N+2)

r1=np.zeros(N+2)
r2=np.zeros(N+2)

q1[0] = 0
q1p[0] = 0
q2[0] = 0
q2p[0] = 0    
t1[0] = 0
t2[0] = 0   



qd =[0.5,0.5]
qdp =[0,0]
qdpp=[0,0]

er=[e1,e2]
erp=[e1p,e2p]
r =[r1,r2]

# parametros del controlador 
M =2
N1 =10
L = 20 
Ka=0.1
DELTA =[3,3]
Kv=[1,1]
PD=[0,0]
fnn1=np.zeros(N+2)
fnn2=np.zeros(N+2)
functionActVect=np.zeros(N+2)
F = 50*[0.0326,0.5612,0.8819,0.6692,0.1904,0.3689,0.4607,0.9816,0.1564,0.8555,0.6448,0.3763,0.1909,0.4283,0.4820,0.1206,0.5895,0.2262,0.3846,0.5830]

w00=np.zeros(N+2)
w01=np.zeros(N+2)
w02=np.zeros(N+2)
w03=np.zeros(N+2)
w04=np.zeros(N+2)
w05=np.zeros(N+2)
w06=np.zeros(N+2)
w07=np.zeros(N+2)
w08=np.zeros(N+2)
w09=np.zeros(N+2)
w10=np.zeros(N+2)
w11=np.zeros(N+2)
w12=np.zeros(N+2)
w13=np.zeros(N+2)
w14=np.zeros(N+2)
w15=np.zeros(N+2)
w16=np.zeros(N+2)
w17=np.zeros(N+2)
w18=np.zeros(N+2)
w19=np.zeros(N+2)
w20=np.zeros(N+2)
w21=np.zeros(N+2)
w22=np.zeros(N+2)
w23=np.zeros(N+2)
w24=np.zeros(N+2)
w25=np.zeros(N+2)
w26=np.zeros(N+2)
w27=np.zeros(N+2)
w28=np.zeros(N+2)
w29=np.zeros(N+2)
w30=np.zeros(N+2)
w31=np.zeros(N+2)
w32=np.zeros(N+2)
w33=np.zeros(N+2)
w34=np.zeros(N+2)
w35=np.zeros(N+2)
w36=np.zeros(N+2)
w37=np.zeros(N+2)
w38=np.zeros(N+2)
w39=np.zeros(N+2)






w00[0]=-0.1225
w01[0]=-0.2369
w02[0]=0.5310
w03[0]=0.5904
w04[0]=-0.6263
w05[0]=-0.0205
w06[0]=-0.1088
w07[0]=0.2926
w08[0]=0.4187
w09[0]=0.5094
w10[0]=-0.4479
w11[0]=0.3594
w12[0]=0.3102
w13[0]=-0.6748
w14[0]=-0.7620
w15[0]=-0.0033
w16[0]=0.9195
w17[0]=-0.3192
w18[0]=0.1705
w19[0]=-0.5524
w20[0]=0.5025
w21[0]=-0.4898
w22[0]=0.0119
w23[0]=0.3982
w24[0]=0.7818
w25[0]=0.9186
w26[0]=0.0944
w27[0]=-0.7228
w28[0]=-0.7014
w29[0]=-0.4850
w30[0]=0.6814
w31[0]=-0.4914
w32[0]=0.6286
w33[0]=-0.5130
w34[0]=0.8585
w35[0]=-0.3000
w36[0]=-0.6068
w37[0]=-0.4978
w38[0]=0.2321
w39[0]=-0.0534


Pesos_estim=[]





V0=[0.629,0.812,-0.746,0.827,0.265,-0.321]
VV1=[-0.805,-0.443,0.094,0.915,0.930,0.245]
VV2=[-0.685,0.941,0.914,-0.029,0.601,-0.876]
V3=[-0.716,-0.156,0.831,0.584,0.919,0.123]
V4=[0.311,-0.929,0.698,0.868,0.357,-0.115]
V5=[0.515,0.486,-0.216,0.311,-0.658,0.411]
V6=[0.412,-0.936,-0.446,-0.908,-0.806,0.102]
V7=[0.647,0.390,-0.366,0.900,-0.931,0.219]
V8=[-0.123,-0.237,0.531,0.590,-0.626,0.311]
V9=[-0.740,0.138,-0.061,-0.976,-0.326,0.001]
V10=[-0.020,-0.109,0.293,0.419,0.509,0.901]
V11=[-0.448,0.359,0.310,-0.675,-0.762,0.032]
V12=[-0.003,0.919,-0.319,0.171,-0.552,0.655]
V13=[0.503,-0.490,0.012,0.398,0.782,-0.087]
V14=[0.919,0.094,-0.723,-0.701,-0.485,-0.234]
V15=[0.681,-0.491,0.629,-0.513,0.859,0.733]
V16=[0.329,0.512,-0.446,0.127,0.665,-0.221]
V17=[-0.205,-0.543,0.394,0.615,0.630,0.945]
V18=[-0.185,0.641,0.214,-0.729,0.301,-0.676]
V19=[-0.016,-0.756,0.131,0.684,0.019,0.423]


V_input=0.1

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

    # señales de error
    er[0]=qd[0]-q1[k]
    er[1]=qd[1]-q2[k]
    erp[0]=qdp[0]-q1p[k]
    erp[1]=qdp[1]-q2p[k]

    #filtrado de error

    r[0]=DELTA[0]*er[0]+erp[0]
    r[1]=DELTA[1]*er[1]+erp[1]
    norm_r=sqrt(r[0]*r[0]+r[1]*r[1])
    # Terminos PD
    PD[0]=Kv[0]*r[0]
    PD[1]=Kv[1]*r[1]

    #Vector de entradas de la RED neuronal 
    x1 = [1,qd[0],qdp[0],qdpp[0],er[0],erp[0]]
    x2 = [1,qd[1],qdp[1],qdpp[1],er[1],erp[1]] 
    



    # Vector de funciones base  para q1
    functionActVect[0]=tanh(V0[0]*x1[0]+V0[1]*x1[1]+V0[2]*x1[2]+V0[3]*x1[3]+V0[4]*x1[4]+V0[5]*x1[5])
    functionActVect[1]=tanh(VV1[0]*x1[0]+VV1[1]*x1[1]+VV1[2]*x1[2]+VV1[3]*x1[3]+VV1[4]*x1[4]+VV1[5]*x1[5])
    functionActVect[2]=tanh(VV2[0]*x1[0]+VV2[1]*x1[1]+VV2[2]*x1[2]+VV2[3]*x1[3]+VV2[4]*x1[4]+VV2[5]*x1[5])
    functionActVect[3]=tanh(V3[0]*x1[0]+V3[1]*x1[1]+V3[2]*x1[2]+V3[3]*x1[3]+V3[4]*x1[4]+V3[5]*x1[5])
    functionActVect[4]=tanh(V4[0]*x1[0]+V4[1]*x1[1]+V4[2]*x1[2]+V4[3]*x1[3]+V4[4]*x1[4]+V4[5]*x1[5])
    functionActVect[5]=tanh(V5[0]*x1[0]+V5[1]*x1[1]+V5[2]*x1[2]+V5[3]*x1[3]+V5[4]*x1[4]+V5[5]*x1[5])
    functionActVect[6]=tanh(V6[0]*x1[0]+V6[1]*x1[1]+V6[2]*x1[2]+V6[3]*x1[3]+V6[4]*x1[4]+V6[5]*x1[5])
    functionActVect[7]=tanh(V7[0]*x1[0]+V7[1]*x1[1]+V7[2]*x1[2]+V7[3]*x1[3]+V7[4]*x1[4]+V7[5]*x1[5])
    functionActVect[8]=tanh(V8[0]*x1[0]+V8[1]*x1[1]+V8[2]*x1[2]+V8[3]*x1[3]+V8[4]*x1[4]+V8[5]*x1[5])
    functionActVect[9]=tanh(V9[0]*x1[0]+V9[1]*x1[1]+V9[2]*x1[2]+V9[3]*x1[3]+V9[4]*x1[4]+V9[5]*x1[5])
    functionActVect[10]=tanh(V10[0]*x1[0]+V10[1]*x1[1]+V10[2]*x1[2]+V10[3]*x1[3]+V10[4]*x1[4]+V10[5]*x1[5])
    functionActVect[11]=tanh(V11[0]*x1[0]+V11[1]*x1[1]+V11[2]*x1[2]+V11[3]*x1[3]+V11[4]*x1[4]+V11[5]*x1[5])
    functionActVect[12]=tanh(V12[0]*x1[0]+V12[1]*x1[1]+V12[2]*x1[2]+V12[3]*x1[3]+V12[4]*x1[4]+V12[5]*x1[5])
    functionActVect[13]=tanh(V13[0]*x1[0]+V11[1]*x1[1]+V11[2]*x1[2]+V11[3]*x1[3]+V11[4]*x1[4]+V11[5]*x1[5])
    functionActVect[14]=tanh(V14[0]*x1[0]+V14[1]*x1[1]+V14[2]*x1[2]+V14[3]*x1[3]+V14[4]*x1[4]+V14[5]*x1[5])
    functionActVect[15]=tanh(V15[0]*x1[0]+V15[1]*x1[1]+V15[2]*x1[2]+V15[3]*x1[3]+V15[4]*x1[4]+V15[5]*x1[5])
    functionActVect[16]=tanh(V16[0]*x1[0]+V16[1]*x1[1]+V16[2]*x1[2]+V16[3]*x1[3]+V16[4]*x1[4]+V16[5]*x1[5])
    functionActVect[17]=tanh(V17[0]*x1[0]+V17[1]*x1[1]+V17[2]*x1[2]+V17[3]*x1[3]+V17[4]*x1[4]+V17[5]*x1[5])
    functionActVect[18]=tanh(V18[0]*x1[0]+V18[1]*x1[1]+V18[2]*x1[2]+V18[3]*x1[3]+V18[4]*x1[4]+V18[5]*x1[5])
    functionActVect[19]=tanh(V19[0]*x1[0]+V19[1]*x1[1]+V19[2]*x1[2]+V19[3]*x1[3]+V19[4]*x1[4]+V19[5]*x1[5])

    


    # Ley de adaptacion de la matrices de pesos q1
    w00[k+1]=w00[k]+Ts*(F[0]*functionActVect[0]*r[0]-Ka*norm_r*F[0]*w00[k])
    w01[k+1]=w01[k]+Ts*(F[1]*functionActVect[1]*r[0]-Ka*norm_r*F[1]*w01[k])
    w02[k+1]=w02[k]+Ts*(F[2]*functionActVect[2]*r[0]-Ka*norm_r*F[2]*w02[k])
    w03[k+1]=w03[k]+Ts*(F[3]*functionActVect[3]*r[0]-Ka*norm_r*F[3]*w03[k])
    w04[k+1]=w04[k]+Ts*(F[4]*functionActVect[4]*r[0]-Ka*norm_r*F[4]*w04[k])
    w05[k+1]=w05[k]+Ts*(F[5]*functionActVect[5]*r[0]-Ka*norm_r*F[5]*w05[k])
    w06[k+1]=w06[k]+Ts*(F[6]*functionActVect[6]*r[0]-Ka*norm_r*F[6]*w06[k])
    w07[k+1]=w07[k]+Ts*(F[7]*functionActVect[7]*r[0]-Ka*norm_r*F[7]*w07[k])
    w08[k+1]=w08[k]+Ts*(F[8]*functionActVect[8]*r[0]-Ka*norm_r*F[8]*w08[k])
    w09[k+1]=w09[k]+Ts*(F[9]*functionActVect[9]*r[0]-Ka*norm_r*F[9]*w09[k])
    w10[k+1]=w10[k]+Ts*(F[10]*functionActVect[10]*r[0]-Ka*norm_r*F[10]*w10[k])
    w11[k+1]=w11[k]+Ts*(F[11]*functionActVect[11]*r[0]-Ka*norm_r*F[11]*w11[k])
    w12[k+1]=w12[k]+Ts*(F[12]*functionActVect[12]*r[0]-Ka*norm_r*F[12]*w12[k])
    w13[k+1]=w13[k]+Ts*(F[13]*functionActVect[13]*r[0]-Ka*norm_r*F[13]*w13[k])
    w14[k+1]=w14[k]+Ts*(F[14]*functionActVect[14]*r[0]-Ka*norm_r*F[14]*w14[k])
    w15[k+1]=w15[k]+Ts*(F[15]*functionActVect[15]*r[0]-Ka*norm_r*F[15]*w15[k])
    w16[k+1]=w16[k]+Ts*(F[16]*functionActVect[16]*r[0]-Ka*norm_r*F[16]*w16[k])
    w17[k+1]=w17[k]+Ts*(F[17]*functionActVect[17]*r[0]-Ka*norm_r*F[17]*w17[k])
    w18[k+1]=w18[k]+Ts*(F[18]*functionActVect[18]*r[0]-Ka*norm_r*F[18]*w18[k])
    w19[k+1]=w19[k]+Ts*(F[19]*functionActVect[19]*r[0]-Ka*norm_r*F[19]*w19[k])

    # Ley de adaptacion de la matrices de pesos q2
    w20[k+1]=w20[k]+Ts*(F[0]*functionActVect[0]*r[1]-Ka*norm_r*F[0]*w20[k])
    w21[k+1]=w21[k]+Ts*(F[1]*functionActVect[1]*r[1]-Ka*norm_r*F[1]*w21[k])
    w22[k+1]=w22[k]+Ts*(F[2]*functionActVect[2]*r[1]-Ka*norm_r*F[2]*w22[k])
    w23[k+1]=w23[k]+Ts*(F[3]*functionActVect[3]*r[1]-Ka*norm_r*F[3]*w23[k])
    w24[k+1]=w24[k]+Ts*(F[4]*functionActVect[4]*r[1]-Ka*norm_r*F[4]*w24[k])
    w25[k+1]=w25[k]+Ts*(F[5]*functionActVect[5]*r[1]-Ka*norm_r*F[5]*w25[k])
    w26[k+1]=w26[k]+Ts*(F[6]*functionActVect[6]*r[1]-Ka*norm_r*F[6]*w26[k])
    w27[k+1]=w27[k]+Ts*(F[7]*functionActVect[7]*r[1]-Ka*norm_r*F[7]*w27[k])
    w28[k+1]=w28[k]+Ts*(F[8]*functionActVect[8]*r[1]-Ka*norm_r*F[8]*w28[k])
    w29[k+1]=w29[k]+Ts*(F[9]*functionActVect[9]*r[1]-Ka*norm_r*F[9]*w29[k])
    w30[k+1]=w30[k]+Ts*(F[10]*functionActVect[10]*r[1]-Ka*norm_r*F[10]*w30[k])
    w31[k+1]=w31[k]+Ts*(F[11]*functionActVect[11]*r[1]-Ka*norm_r*F[11]*w31[k])
    w32[k+1]=w32[k]+Ts*(F[12]*functionActVect[12]*r[1]-Ka*norm_r*F[12]*w32[k])
    w33[k+1]=w33[k]+Ts*(F[13]*functionActVect[13]*r[1]-Ka*norm_r*F[13]*w33[k])
    w34[k+1]=w34[k]+Ts*(F[14]*functionActVect[14]*r[1]-Ka*norm_r*F[14]*w34[k])
    w35[k+1]=w35[k]+Ts*(F[15]*functionActVect[15]*r[1]-Ka*norm_r*F[15]*w35[k])
    w36[k+1]=w36[k]+Ts*(F[16]*functionActVect[16]*r[1]-Ka*norm_r*F[16]*w36[k])
    w37[k+1]=w37[k]+Ts*(F[17]*functionActVect[17]*r[1]-Ka*norm_r*F[17]*w37[k])
    w38[k+1]=w38[k]+Ts*(F[18]*functionActVect[18]*r[1]-Ka*norm_r*F[18]*w38[k])
    w39[k+1]=w39[k]+Ts*(F[19]*functionActVect[19]*r[1]-Ka*norm_r*F[19]*w39[k])

    w00[k]=w00[k+1]
    w01[k]=w01[k+1]
    w02[k]=w02[k+1]
    w03[k]=w03[k+1]
    w04[k]=w04[k+1]
    w05[k]=w05[k+1]
    w06[k]=w06[k+1]
    w07[k]=w07[k+1]
    w08[k]=w08[k+1]
    w09[k]=w09[k+1]
    w10[k]=w10[k+1]
    w11[k]=w01[k+1]
    w12[k]=w02[k+1]
    w13[k]=w03[k+1]
    w14[k]=w04[k+1]
    w15[k]=w05[k+1]
    w16[k]=w06[k+1]
    w17[k]=w07[k+1]
    w18[k]=w08[k+1]
    w19[k]=w09[k+1]
    w20[k]=w10[k+1]
    w21[k]=w01[k+1]
    w22[k]=w02[k+1]
    w23[k]=w03[k+1]
    w24[k]=w04[k+1]
    w25[k]=w05[k+1]
    w26[k]=w06[k+1]
    w27[k]=w07[k+1]
    w28[k]=w08[k+1]
    w29[k]=w09[k+1]
    w30[k]=w10[k+1]
    w31[k]=w01[k+1]
    w32[k]=w02[k+1]
    w33[k]=w03[k+1]
    w34[k]=w04[k+1]
    w35[k]=w05[k+1]
    w36[k]=w06[k+1]
    w37[k]=w07[k+1]
    w38[k]=w08[k+1]
    w39[k]=w09[k+1]

    Pesos_estim=[w00,w01,w02,w03,w04,w05,w06,w07,w08,w09,w10,w11,w12,w13,w14,w15,w16,w17,w18,w19,w20,w21,w22,w23,w24,w25,w26,w27,w28,w29,w30,w31,w32,w33,w34,w35,w36,w37,w38,w39]
  
    #fnn=w00[k]*functionActVect[0]+w01[k]*functionActVect[1]
    
    fnn1[k]=fnn1[k]+(w00[k]*functionActVect[0]+w01[k]*functionActVect[1]+w02[k]*functionActVect[2]+w03[k]*functionActVect[3]
    +w04[k]*functionActVect[4]+w05[k]*functionActVect[5]+w06[k]*functionActVect[6]+w07[k]*functionActVect[7]+w08[k]*functionActVect[8]+w09[k]*functionActVect[9]
    +w10[k]*functionActVect[10]+w11[k]*functionActVect[11]+w12[k]*functionActVect[12]+w13[k]*functionActVect[13]+w14[k]*functionActVect[14]+w15[k]*functionActVect[15]
    +w16[k]*functionActVect[16]+w17[k]*functionActVect[17]+w18[k]*functionActVect[18]+w19[k]*functionActVect[19])

    fnn2[k]=fnn2[k]+(w20[k]*functionActVect[0]+w21[k]*functionActVect[1]+w22[k]*functionActVect[2]+w23[k]*functionActVect[3]
    +w24[k]*functionActVect[4]+w25[k]*functionActVect[5]+w26[k]*functionActVect[6]+w27[k]*functionActVect[7]+w28[k]*functionActVect[8]+w29[k]*functionActVect[9]
    +w30[k]*functionActVect[10]+w31[k]*functionActVect[11]+w32[k]*functionActVect[12]+w33[k]*functionActVect[13]+w34[k]*functionActVect[14]+w35[k]*functionActVect[15]
    +w36[k]*functionActVect[16]+w37[k]*functionActVect[17]+w38[k]*functionActVect[18]+w39[k]*functionActVect[19])








    #señal de control PD +NN
    t1[k]=PD[0]+fnn1[k]
    t2[k]=PD[1]+fnn2[k]

 
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



plt.figure(3)
plt.title("Robot 2GL  Velocidad")
plt.plot(t,fnn1,'b', label=r'$fnn_1(t) $')
plt.plot(t,fnn2,'r', label=r'$fnn_2(t) $')
plt.ylabel('[m\s]')
plt.xlabel('time')
plt.legend(loc='best')


plt.figure(4)
plt.title("Pesos Estimados $\hat{W}_1$")
for i in range(len(Pesos_estim)):
    plt.plot(t,Pesos_estim[i])

plt.ylabel('pesos')
plt.xlabel('time')
plt.legend(loc='best')

plt.show()