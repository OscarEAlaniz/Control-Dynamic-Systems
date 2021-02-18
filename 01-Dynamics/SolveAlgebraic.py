import numpy as np
import matplotlib.pyplot as plt

K = 3
T = 4

start = 0
stop = 30
increment = 0.1

t = np.arange(start,stop,increment)

y = K*(1-np.exp(-t/T))

plt.plot(t,y)

plt.title("1. Orde Dynamic system")
plt.xlabel('t[s]')
plt.ylabel('y(t)')
plt.grid()
plt.show()

