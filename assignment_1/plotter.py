#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
plot_polygons = True
change = float
if plot_polygons:
    with open("input.txt", "r") as f:
        start = [change(i) for i in f.readline().split(",")]
        goal = [change(i) for i in f.readline().split(",")]
        stepsize = change(f.readline())
        n = f.readline()  # newline charecter
        lines = f.readlines()
        i = 0
        P = []
        obstacles = []
        while i < len(lines):
            if lines[i] == n:
                obstacles.append(P)
                P = []
            else:
                x, y = [change(point) for point in lines[i].split(",")]
                P.append([x, y])
            i += 1
        obstacles.append(P)

    for P in obstacles:
        # plt.Figure(1)
        polygon = plt.Polygon(P, facecolor='k')
        ax.add_patch(polygon)
    # Importing path
    with open("output_1.txt", "r") as f:
        lines = f.readlines()
        X = []
        Y = []
        for i in lines:
            x, y = i.split(",")
            X.append(change(x))
            Y.append(change(y))
    # plt.Figure(1)
    ax.plot(X, Y)
    fig.savefig('./WorkingROS.png')
# initializing a figure in
# which the graph will be plotted
fig = plt.figure()

# marking the x-axis and y-axis
axis = plt.axes(xlim=(-1, 11),
				ylim=(-1, 11))

# initializing a line variable
line, = axis.plot([], [], lw=1.5)

# data which the line will
# contain (x, y)
n_steps = 10
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


anim.save('bug_1motion_ROS.mp4', writer = 'ffmpeg', fps = 10)


