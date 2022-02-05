#! /usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np


class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __sub__(self, other):
        return point(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return point(self.x+other.x, self.y+other.y)

    def dot(self, q):
        return self.x*q.x + self.y*q.y

    def norm(self):
        return np.sqrt(self.x**2+self.y**2)

    def multi(self, c):
        return point(self.x*c, self.y*c)

    def dist(self, q):
        return np.sqrt((q.x-self.x)**2+(q.y-self.y)**2)

    def __str__(self):
        a = '{}, {}'.format(self.x, self.y)
        return a


def PlotPoly(P):
    poly = [[i.x, i.y] for i in P]
    plt.Figure(1)
    polygon = plt.Polygon(poly)
    plt.gca().add_patch(polygon)
    plt.show()


def computeLineThroughTwoPoints(p1, p2):
    slope = (p2.y-p1.y)/(p2.x-p1.x)
    k = np.sqrt(1/(1+(slope)**2))
    b = -k
    a = k*slope
    c = k*(p1.y-p1.x*slope)
    return a, b, c


def computeDistancePointToLine(q, p1, p2):
    a, b, c = computeLineThroughTwoPoints(p1, p2)
    d = a*(q.x-p1.x)+b*(q.y-p1.y)
    return abs(d)


def computeDistancePointToSegment(q, p1, p2):

    if (q-p1).dot(p2-p1) >= 0:
        if (q-p2).dot(p1-p2) >= 0:
            return computeDistancePointToLine(q, p1, p2), 0  # between two pints of linesegment
        else:
            return q.dist(p2), 2  # q,p1 <=90deg q,p2>90 deg
    else:  # q,p1 > 90deg
        return q.dist(p1), 1


def insidePoly(P, q):
    a, b, c = computeLineThroughTwoPoints(P[0], P[1])
    d = a*(q.x-P[0].x)+b*(q.y-P[0].y)
    dists = d
    for i in range(1, len(P)):
        j = (i + 1) % (len(P) - 1)
        a, b, c = computeLineThroughTwoPoints(P[i], P[j])
        d = a*(q.x-P[i].x)+b*(q.y-P[i].y)
        if dists*d < 0:
            return False
    # All signs of distances is towards same side
    return True


def computeDistancePointToPolygon(P, q):
    # check inside polygon
    if insidePoly(P, q):
        return 0
    # outside polygon
    dists = []
    for i in range(len(P)):
        d, _ = computeDistancePointToSegment(q, P[i], P[(i+1) % (len(P)-1)])
        dists.append(d)
    return min(dists)


def computeTangentVectorToPolygon(P, q):
    min_d = computeDistancePointToPolygon(P, q)
    if min_d > 0:
        for i in range(len(P)):
            d, w = computeDistancePointToSegment(q, P[i], P[(i+1) % (len(P)-1)])
            if min_d == d:
                if w == 0:
                    return (P[(i+1) % (len(P)-1)]-P[i]).multi(1/P[i].dist(P[(i+1) % (len(P)-1)]))
                elif w == 1:
                    d1 = P[i].dist(q)
                    d0 = P[(i+1)%(len(P)-1)].dist(P[i])
                    temp = (P[(i+1)%(len(P)-1)]-P[i]).dot(q-P[i])
                    k = temp/(d1**2)
                    vec = (P[(i+1)%(len(P)-1)]-P[i]) - (q-P[i]).multi(k)
                    return vec.multi(1/(vec.norm()))
                elif w == 2:
                    d1 = P[(i+1)%(len(P)-1)].dist(q)
                    d0 = P[(i+1)%(len(P)-1)].dist(P[i])
                    temp = (P[(i+1)%(len(P)-1)]-P[i]).dot(q-P[(i+1)%(len(P)-1)])
                    k = temp/(d1**2)
                    vec = (P[(i+1)%(len(P)-1)]-P[i]) - (q-P[(i+1)%(len(P)-1)]).multi(k)
                    return vec.multi(1/(vec.norm()))
    else :
        raise Exception("inside Virtual Obstacle!")
    pass 

def gen_Output(f,path): #check the correctness
    with open(f,'w') as file:
        file.write(str(path[0]))
        for p in path[1:]:
            file.write("\n")
            file.write(str(p))
            pass
