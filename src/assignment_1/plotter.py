import matplotlib.pyplot as plt

with open("input_format.txt", "r") as f:
    change = float
    start = [change(i) for i in f.readline().split(", ")]
    goal = [change(i) for i in f.readline().split(", ")]
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
            x, y = [change(point) for point in lines[i].split(", ")]
            P.append([x, y])
        i += 1
    obstacles.append(P)

fig, ax = plt.subplots()
for P in obstacles:
    # plt.Figure(1)
    polygon = plt.Polygon(P, facecolor='k')
    ax.add_patch(polygon)
with open("output_1.txt", "r") as f:
    lines = f.readlines()
    X = []
    Y = []
    for i in lines:
        x, y = i.split(", ")
        X.append(change(x))
        Y.append(change(y))

# plt.Figure(1)
ax.plot(X, Y)
fig.savefig('./test.png')
