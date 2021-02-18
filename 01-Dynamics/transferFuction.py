import control
import numpy as np
import matplotlib.pyplot as plt

K = 3
T = 4

num = np.array([K])
den = np.array([T, 1])

G = control.tf(num, den)
print ('G(s)= ', G)

t, y = control.step_response(G)

plt.plot(t,y)
plt.title("1. Orde Dynamic system laplace")
plt.xlabel('t[s]')
plt.ylabel('y(t)')
plt.grid()
plt.show()

