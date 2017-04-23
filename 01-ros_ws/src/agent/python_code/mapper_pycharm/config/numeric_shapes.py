import numpy as np
import math
from math import pi


def circle_shape():
    sx = lambda s, q: q[2] * np.cos(s * 2 * math.pi) + q[0]
    sy = lambda s, q: q[2] * np.sin(s * 2 * math.pi) + q[1]
    return sx, sy


def ellipse_shape(area=2000,enlarged=0):
    sx = lambda s, q: (q[2]+enlarged) * np.cos(s * 2 * math.pi) + q[0]
    sy = lambda s, q: (area / (math.pi * q[2])+enlarged) * np.sin(s * 2 * math.pi) + q[1]
    return sx, sy


def star_shape(points=5, rad=10):
    # sx = lambda s, q: .2 * np.cos(points * math.pi * s) * rad*np.cos(2 * math.pi * s) + q[0]
    # sy = lambda s, q: .2 * np.cos(points * math.pi * s) * rad*np.sin(2 * math.pi * s) + q[1]
    sx = lambda s, q: q[2] * np.cos(points * 2 * math.pi * s + q[3]) * np.cos(2 * math.pi * s) + rad * np.cos(
        2 * math.pi * s) + q[0]
    sy = lambda s, q: q[2] * np.cos(points * 2 * math.pi * s + q[3]) * np.sin(2 * math.pi * s) + rad * np.sin(
        2 * math.pi * s) + q[1]
    return sx, sy


def rotated_ellipse_shape(area=320):  # 160
    # x' = a*cos(t)*cos(theta) - b*sin(t)*sin(theta)         y' = a*cos(t)*sin(theta) + b*sin(t)*cos(theta)

    sx = lambda s, q: q[2] * np.cos(s * 2 * math.pi) * np.cos(q[3]) - (area / (math.pi * q[2])) * np.sin(
        s * 2 * math.pi) * np.sin(q[3]) + q[0]
    sy = lambda s, q: (area / (math.pi * q[2])) * np.sin(s * 2 * math.pi) * np.cos(q[3]) + q[2] * np.cos(
        s * 2 * math.pi) * np.sin(q[3]) + q[1]
    return sx, sy


def egg_shape(size=1, scale=1):
    # Area
    # A1 = 80 * (0.625 * math.pi * size ** 2)
    # p2 parameter to maintain the area
    # fp2 = lambda p1: 0.112837916709551 * math.sqrt(A1 - 78.5398163397448 * p1 ** 2)
    area = 0.625 * math.pi
    fp2 = lambda p1: math.sqrt(16. * area / (5. * math.pi) - p1 ** 2)

    x = lambda p1, p2, s: (p1 * np.sin(s * math.pi) ** 3 + p2 * np.cos(s * math.pi) ** 3) * np.cos(
        s * math.pi)
    y = lambda p1, p2, s: (p1 * np.sin(s * math.pi) ** 3 + p2 * np.cos(s * math.pi) ** 3) * np.sin(
        s * math.pi)

    def p_mapping(p):
        if p < 2:
            p -= 1  # Normalize for p in [-1,1]
            p1, p2 = p, fp2(p)
        elif p < 4:
            p -= 3  # Normalize for p in [-1,1]
            p1, p2 = fp2(p), -p
        elif p < 6:
            p -= 5  # Normalize for p in [-1,1]
            p1, p2 = -p, -fp2(p)
        elif p <= 8:
            p -= 7  # Normalize for p in [-1,1]
            p1, p2 = -fp2(p), p
        return p1, p2

    def sx(s, q):
        p = q[2]
        p1, p2 = p_mapping(p)
        return x(p1, p2, s) * scale + q[0]

    def sy(s, q):
        p = q[2]
        p1, p2 = p_mapping(p)

        return y(p1, p2, s) * scale + q[1]

    return sx, sy


def egg_shape2(size=15, scale=1):
    # Area
    A1 = 80 * (0.625 * math.pi * size ** 2)
    # p2 parameter to maintain the area
    fp2 = lambda p1: 0.112837916709551 * math.sqrt(A1 - 78.5398163397448 * p1 ** 2)
    # area = 31.5625 * math.pi * 5
    # fp2 = lambda p1: math.sqrt(16 * area / (5 * math.pi) - p1 ** 2)

    x = lambda p1, p2, s: (p1 * np.sin(s * math.pi) ** 3 + p2 * np.cos(s * math.pi) ** 3) * np.cos(
        s * math.pi)
    y = lambda p1, p2, s: (p1 * np.sin(s * math.pi) ** 3 + p2 * np.cos(s * math.pi) ** 3) * np.sin(
        s * math.pi)

    def p_mapping(p):
        if p < 2:
            p -= 1  # Normalize for p in [-1,1]
            tx, ty = 10 - abs(p) * 5, p * 5  # move to center of mass
            p *= size  # scale
            p1, p2 = p, fp2(p)
        elif p < 4:
            p -= 3  # Normalize for p in [-1,1]
            tx, ty = -5 * p, 10 - 5 * abs(p)
            p *= size
            p1, p2 = fp2(p), -p
        elif p < 6:
            p -= 5  # Normalize for p in [-1,1]
            tx, ty = -10 + abs(p) * 5, -5 * p
            p *= size
            p1, p2 = -p, -fp2(p)
        elif p <= 8:
            p -= 7  # Normalize for p in [-1,1]
            tx, ty = 5 * p, -10 + 5 * abs(p)
            p *= size
            p1, p2 = -fp2(p), p
        return p1, p2, tx, ty

    def sx(s, q):
        p = q[2]  # % 8
        p1, p2, tx, ty = p_mapping(p)

        return x(p1, p2, s) * scale + q[0] - tx

    def sy(s, q):
        p = q[2]  # % 8
        p1, p2, tx, ty = p_mapping(p)

        return y(p1, p2, s) * scale + q[1] - ty

    return sx, sy
