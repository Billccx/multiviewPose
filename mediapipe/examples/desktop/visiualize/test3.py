import typing as tp
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot([1,2,3,2,1],[1,2,3,2,1],[1,2,3,4,1])
print(*[[1,2,3,2,1],[1,2,3,2,1],[1,2,3,4,1]])
#ax.plot(([1,2,3,2,1],[1,2,3,2,1],[1,2,3,4,5]))
plt.show()