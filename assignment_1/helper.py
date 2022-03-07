#!/usr/bin/env python3
import math


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def computeLineThroughTwoPoints(p1, p2):
    if distance(p1, p2) < 10**(-8):
        print('Same Point')
        return None
    a = (p2[1] - p1[1])/distance(p1, p2)
    b = (p1[0] - p2[0])/distance(p1, p2)
    c = (p2[0]*p1[1] - p1[0]*p2[1])/distance(p1, p2)
    return a,b,c 


def computeDistancePointToLine(q, p1, p2):
    a, b, c = computeLineThroughTwoPoints(p1, p2)
    return math.abs(a*q[0] + b*q[1] + c)

def computeDistancePointToSegment(q, p1, p2):
    vec1 = (q[0] - p1[0], q[1] - p1[1])
    vec2 = (p1[0] - p2[0], p1[1] - p2[1])
    lmd = math.dot(vec1, vec2) / math.dot(vec2, vec2)
    if (lmd <= 0):
        dist = distance(q, p1)
        w = 1
    elif (lmd >= 1):
        dist = distance(q, p2)
        w = 2
    else:
        dist = computeDistancePointToLine(q, p1, p2)
        w = 0
    return dist, w


def computeDistancePointToPolygon (P, q):
    distances = []
    n = P.shape[0]
    for i in range(n):
        distances.append(computeDistancePointToSegment(P[i], P[i+1], q))
    return min(distances)


def computeTangentVectorToPolygon(P, q):
    segments = []
    n = P.shape[0]
    for i in range(n):
        d, _ = computeDistancePointToSegment(q, P[i], P[i+1])
        segments.append(d)
    nearest_segment = min(segments)
    dist, w = computeDistancePointToSegment(q, *nearest_segment)
    if w == 0:
        tangent = [
            (nearest_segment[1][0] - nearest_segment[0][0]), 
            (nearest_segment[1][1] - nearest_segment[0][1])     
        ] / distance(*nearest_segment)
    elif w == 1:
        tangent = [-q[1] + nearest_segment[0][1], 
        q[0] - nearest_segment[0][0]] / distance(nearest_segment[0], q)
    else:
        tangent = [
            (-q[1] + nearest_segment[1][1]),
            (q[0] - nearest_segment[1][0]) 
        ] / distance(nearest_segment[1], q)

