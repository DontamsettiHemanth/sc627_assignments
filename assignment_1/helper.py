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

    def __str__(self):
        a = '{},{}'.format(self.x, self.y)
        return a

    def dot(self, q):
        return self.x*q.x + self.y*q.y*1.0

    def norm(self):
        return np.sqrt(self.x**2+self.y**2)*1.0

    def multi(self, c):
        return point(self.x*c*1.0, self.y*c*1.0)

    def dist(self, q):
        return np.sqrt((q.x-self.x)**2+(q.y-self.y)**2*1.0)
    
    def normalize(self):
        n2 = self.norm()
        if n2==0:
            # print "norm = 0; didn't normalize ({})".format(str(self))
            return
        temp = self.multi(1.0/n2)
        self.x = temp.x
        self.y = temp.y
        return None


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

def computeNearestVectorTowardsPolygon(P,q):
    """Works if q is outside Polygon"""
    min_d = computeDistancePointToPolygon(P, q)
    num_P = len(P)
    for i in range(num_P):
        d, w = computeDistancePointToSegment(q, P[i], P[(i+1) % num_P])
        if min_d == d:
            d0 = (P[(i+1) % num_P]-P[i])
            d0.normalize()
            n0 = point(-d0.y, d0.x)
            if w == 0:
                return n0.multi(d)
            elif w == 1:
                d1 = q - P[i]
                d1.normalize()
                return d1.multi(-d)
            elif w == 2:
                d2 = q - P[(i+1) % num_P]
                d2.normalize()
                return d2.multi(-d)


def computeTangentVectorToPolygon(P, q, tolerance):
    """
    Note : modified algo (compared to book)to avoid spiralling around the obstacle
    OUTPUT : vector , 1/0
           Arg 1) where 'vector' is tangent vector if the point is within 1.5*tolerance bound
            of polgon, else is a vector which goes close to the polygon or away based 
            on where it far or inside polygon respectively.
           Arg 2) 1 if point q is outside polygon P , 0 if inside P
    """
    min_d = computeDistancePointToPolygon(P, q)
    num_P = len(P)
    if min_d > 0:
        far = False
        if min_d > 1.2*tolerance:
            far = True
            far_factor = 0.8
        for i in range(num_P):
            d, w = computeDistancePointToSegment(q, P[i], P[(i+1) % num_P])
            if min_d == d:
                d0 = (P[(i+1) % num_P]-P[i])
                d0.normalize()
                n0 = point(-d0.y, d0.x)
                if w == 0:
                    if far:
                        new = d0+n0
                        new.normalize()
                        return new.multi(far_factor), 1
                    return d0, 1
                elif w == 1:
                    d1 = q - P[i]
                    d1.normalize()
                    n1 = point(-d1.y, d1.x)
                    if far:
                        new = n1 - d1
                        new.normalize()
                        return new.multi(far_factor), 1
                    return n1, 1
                elif w == 2:
                    d2 = q - P[(i+1) % num_P]
                    d2.normalize()
                    n2 = point(-d2.y, d2.x)
                    if far:
                        new = n2 - d2
                        new.normalize()
                        return new.multi(far_factor), 1
                    return n2, 1

    else:
        # Inside Virtual Obstacle! Moving towards the boundary
        d, w = computeDistancePointToSegment(q, P[0], P[1])
        minargs = [0, d]
        for i in range(1, num_P):
            d, w = computeDistancePointToSegment(q, P[i], P[(i+1) % num_P])
            if minargs[1] > d:
                minargs = [i, d]
                pass
            pass

        d0 = P[(minargs[0]+1) % num_P] - P[minargs[0]]
        n0 = point(d0.y, -d0.x)
        n0.normalize()
        n0 = n0.multi(0.5)
        # unit vector perpendicular to nearest edge of polygon
        return n0, 0


def gen_Output(f, path):  # check the correctness
    with open(f, 'w') as file:
        file.write(str(path[0]))
        for p in path[1:]:
            file.write("\n")
            file.write(str(p))
            pass
