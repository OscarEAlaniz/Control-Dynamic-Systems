import numpy as np
import matplotlib.pyplot as plt


# parametros del modelo

K = 3
T = 4
a = -1/T
b = K/T

# parametros de simulacion

Ts = 0.1
Tstop = 30

uk = 1 # step response
yk = 0 # valor inicial

N = int(Tstop/Ts) # longitud de simulacion

data = []

data.append(yk)

# simulacion


for k in range(N):
    yk1 = (1+a*Ts)*yk + Ts*b*uk
    yk = yk1
    data.append(yk1)


# plot simualcion resultados
t = np.arange(0,Tstop+Ts,Ts)
plt.plot(t,data)

plt.title("1. Orde Dynamic system Discrete")
plt.xlabel('t[s]')
plt.ylabel('y(k+1)')
plt.grid()
plt.show()

