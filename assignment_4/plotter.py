#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
change = float
T_all = []
X_all = []
Y_all = []
legends =[]
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
    # plt.Figure(1)
    ax.plot(X, Y, label=legend)
    # ax.plot(start[0], start[-1],'ro',goal[0],goal[-1], "go", lw = 1.2)
ax.plot(0, 0,'ro',lw = 1.2, label= "Bot - 1")
ax.plot(14, 0,'go',lw = 1.2, label= "Bot - 8")
ax.legend()
plt.title(f"Trace of the path of all bots", fontsize=15)
plt.ylim(-0.0016, 0.0006)
plt.grid()
plt.ylabel("Global - y")
plt.xlabel("Global - x")
# plt.autoscale()
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
# plt.ylim(-0.0016, 0.0006)
plt.grid()
plt.ylabel("Global - x (m)")
plt.xlabel("time (s)")
box = ax.get_position()
ax.set_position([box.x0, box.y0 + box.height * 0.1,
                 box.width, box.height * 0.9])

# Put a legend below current axis
ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
          fancybox=True, shadow=True, ncol=5)
# plt.autoscale()
plt.savefig('posx_vs_time.png', dpi=300, bbox_inches = 'tight') # pad_inches=0.5)
"""
# which the graph will be plotted
fig = plt.figure()

# marking the x-axis and y-axis
axis = plt.axes(xlim=(-1, 6), ylim=(-1, 5))

for P in obstacles:
    polygon = plt.Polygon(P, facecolor='k')
    axis.add_patch(polygon)
axis.plot(start[0], start[-1],'ro',goal[0],goal[-1], "go", lw = 1.2)
plt.title(f"Trace of the path\n Total length of path is {pathLength:.2f}m")
plt.xlabel("Global - x")
plt.ylabel("Global - y")
# initializing a line variable
line, = axis.plot([], [], lw=1.5)

# data which the line will
# contain (x, y)
n_steps = 7
def init():
    line.set_data(X[0:n_steps], Y[0:n_steps])
    return line,

max_frames = min([1500, len(X)-1-n_steps])
def animate(i):
    j = i
    if i > (len(X)-1-n_steps):
        j = len(X) - 1 - n_steps
    x = X[j:(j + n_steps)]
    y = Y[j:(j + n_steps)]
    line.set_data(x, y)
    return line,

anim = FuncAnimation(fig, animate, init_func = init,
					frames = max_frames, interval = 20, blit = True)


anim.save('PotentialFunction_ROS.mp4', writer = 'ffmpeg', fps = 10)

"""
