#! /usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np


class point:
    def __init__(self, x, y):
        self.x = x*1.0
        self.y = y*1.0

    def __sub__(self, other):
        return point(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return point(self.x + other.x, self.y + other.y)

    def dot(self, q):
        return self.x*q.x + self.y*q.y*1.0

    def norm(self):
        return np.sqrt(self.x**2+self.y**2)*1.0

    def multi(self, c):
        return point(self.x*c*1.0, self.y*c*1.0)

    def dist(self, q):
        return np.sqrt((q.x-self.x)**2+(q.y-self.y)**2*1.0)

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
    k = np.sqrt(1.0/((p2.y-p1.y)**2 + (p2.x-p1.x)**2))
    b = -k*(p2.x - p1.x)
    a = k*(p2.y - p1.y)
    c = k*(p1.y*(p2.x-p1.x)-p1.x*(p2.y-p1.y))
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


"""
def insidePoly(P,q):
    for i in range(len(P)):
        d0 = P[(i+1) % len(P)] - P[0]
        n0 = point(-d0.y, d0.x)
        pq = q - P[i]
        if pq.dot(n0) < 0:
            return False
    return True
"""


def insidePoly(P, q):
    a, b, c = computeLineThroughTwoPoints(P[0], P[1])
    d = a*(q.x-P[0].x)+b*(q.y-P[0].y)
    dists = d
    for i in range(1, len(P)):
        j = (i + 1) % len(P)
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
        d, w = computeDistancePointToSegment(q, P[i], P[(i+1) % len(P)])
        dists.append(d)
    return min(dists)


def sign(a):
    return (a >= 0)*1 - 1*(a < 0)


def computeTangentVectorToPolygon(P, q, tolerance):
    """
    Note : modified algo (compared to book)to avoid spiralling around the obstacle
    OUTPUT : vector , 1/0
           Arg 1) where 'vector' is tangent vector if the point is within 1.5*tolerance bound
            of polgon, else is a vector which goes close to the polygon.
           Arg 2) 1 if point q is outside polygon P , 0 if inside P
    """
    min_d = computeDistancePointToPolygon(P, q)
    num_P = len(P)
    if min_d > 0:
        far = False
        if min_d > 1.5*tolerance:
            far = True
            far_factor = 0.55
        for i in range(num_P):
            d, w = computeDistancePointToSegment(q, P[i], P[(i+1) % num_P])
            if min_d == d:
                d0 = (P[(i+1) % num_P]-P[i])
                d0 = d0.multi(1/d0.norm())
                n0 = point(-d0.y, d0.x)
                if w == 0:
                    if far:
                        new = d0+n0
                        return new.multi(far_factor/new.norm()), 1
                    return d0, 1
                elif w == 1:
                    d1 = q - P[i]
                    d1 = d1.multi(1.0/d1.norm())
                    n1 = point(-d1.y, d1.x)
                    if far:
                        new = n1 - d1
                        return new.multi(far_factor/new.norm()), 1
                        # is unit vector as n and d are perpendicular
                    # if not (n1.dot(n0) <= 0):
                    #     print "Didn't satisfy condition for w = 1"
                    #     return n1.multi(-1), 1
                    return n1, 1
                elif w == 2:
                    d2 = q - P[(i+1) % num_P]
                    d2 = d2.multi(1.0/d2.norm())
                    n2 = point(-d2.y, d2.x)
                    if far:
                        new = n2 - d2
                        return new.multi(far_factor/new.norm()), 1
                    # if not (n2.dot(n0) >= 0):
                    #     print "Didn't satisfy condition for w = 2"
                    #     return n2.multi(-1), 1
                    return n2, 1

    else:
        # Inside Virtual Obstacle! Moving towards the boundary
        d, w = computeDistancePointToSegment(q, P[0], P[1])
        minargs = [0, d]
        for i in range(1, num_P):
            d, w = computeDistancePointToSegment(q, P[i], P[(i+1) % num_P])
            if min_d > d:
                minargs = [i, d]
                pass
            pass

        d0 = P[(minargs[0]+1) % num_P] - P[minargs[0]]
        n0 = point(d0.y, -d0.x)
        n0 = n0.multi(1/n0.norm())
        # unit vector perpendicular to nearest edge of polygon
        return n0, 0


def gen_Output(f, path):  # check the correctness
    with open(f, 'w') as file:
        file.write(str(path[0]))
        for p in path[1:]:
            file.write("\n")
            file.write(str(p))
            pass