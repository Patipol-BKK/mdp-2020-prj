import matplotlib.pyplot as plt
import numpy as np
x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 30, 40, 50, 70, 100]
y = [0, 1.25, 2.7, 3.95, 5.15, 6.1, 7.8, 8.25, 9.4, 10.55, 11.65, 16.2, 21.9, 26.55, 32.5, 42.5, 52.5, 72.75, 105]

ax = plt.gca()
ax.set_xticks(np.arange(0,101,10))
ax.set_yticks(np.arange(0,101,10))
ax.grid(which='major', alpha=0.2)

plt.plot(x, y)
plt.show()