#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
change = float
T_all = []
X_all = []
Y_all = []
legends = []
for j in range(2, 8):
    # Importing path
    T = []
    X = []
    Y = []
    with open(f"outputbalancing{j}.txt", "r") as f:
        lines = f.readlines()
        for i in lines[1:]:
            t, x, y = i.split(", ")
            T.append(change(t))
            X.append(change(x))
            Y.append(change(y))
    T_all.append(np.array(T))
    X_all.append(np.array(X))
    Y_all.append(np.array(Y))
    pathLength = 0
    for i in range(len(X)-1):
        pathLength += np.sqrt((X[i+1]-X[i])**2 + (Y[i+1]-Y[i])**2)
    legend = f"bot {j} :- pathLength = {pathLength:.3}"
    ax.plot(X, Y, label=legend)
ax.plot(0, 0,'ro',lw = 1.2, label= "Bot - 1")
ax.plot(14, 0,'go',lw = 1.2, label= "Bot - 8")
ax.legend()
plt.title(f"Trace of the path of all bots", fontsize=15)
plt.ylim(-0.0016, 0.0006)
plt.grid()
plt.ylabel("Global - y")
plt.xlabel("Global - x")
plt.savefig('robot_path.png',dpi=300, bbox_inches = 'tight')#pad_inches=0.5)


fig, ax = plt.subplots()
t = np.arange(0, 60)
x1 = np.zeros(60)
x8 = np.ones(60)*14
ax.plot(t, x1, label="Bot-1")
for i in range(6):
    ax.plot(T_all[i], X_all[i], label = f"Bot-{i+2}")

ax.plot(t, x8, label="Bot-8")
ax.legend()
plt.title(f"Robot x-coordinate w.r.t time", fontsize=15)
plt.grid()
plt.ylabel("Global - x (m)")
plt.xlabel("time (s)")
box = ax.get_position()
ax.set_position([box.x0, box.y0 + box.height * 0.1,
                 box.width, box.height * 0.9])

ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
          fancybox=True, shadow=True, ncol=5)
plt.savefig('posx_vs_time.png', dpi=300, bbox_inches='tight')
