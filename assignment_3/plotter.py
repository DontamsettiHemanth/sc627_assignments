
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
change = float
with open("/root/catkin_ws/src/sc627_assignments/assignment_3/output.txt", "r") as file:
    lines = file.readlines()
    X = []
    Y = []
    T = []
    for i in lines:
        t, x, y = i.split(", ")
        T.append(change(t))
        X.append(change(x))
        Y.append(change(y))

    pathLength = 0
    for i in range(len(X)-1):
        pathLength += np.sqrt((X[i+1]-X[i])**2 + (Y[i+1]-Y[i])**2)

    fig, ax = plt.subplots()
    ax.plot(X, Y)

    ax.plot(0, 0, 'ro', 5, 0, "go", lw=1.2)
    plt.title(f"Trace of the path\n Total length of path is {pathLength:.2f}m")
    plt.xlabel("Global - x")
    plt.ylabel("Global - y")
    fig.savefig('TraceOfTurtleBot.png')

    fig, ax = plt.subplots()
    ax.plot(T, X, label="X")
    ax.plot(T, Y, label="Y")
    plt.title("Change of position w.r.t. time")
    plt.xlabel("time (sec)")
    plt.ylabel("x or y (m)")
    plt.legend()
    fig.savefig('Pos_vs_time.png')
