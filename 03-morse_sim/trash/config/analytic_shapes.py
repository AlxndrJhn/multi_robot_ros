import math
from sympy import cos, sin, Piecewise, sqrt, Symbol, And
from math import pi
from robotplanning.diffeq import s


def circle_shape():
    shapexy = lambda q: (q[2] * cos(2 * math.pi * s) + q[0], q[2] * sin(2 * math.pi * s) + q[1])
    return shapexy


def ellipse_shape(area=2000):  # 160
    shapexy = lambda q: (q[2] * cos(2 * math.pi * s) + q[0], (area / (math.pi * q[2])) * sin(2 * math.pi * s) + q[1])
    return shapexy


def rotated_ellipse_shape(area=160):
    # x' = a*cos(t)*cos(theta) - b*sin(t)*sin(theta)         y' = a*cos(t)*sin(theta) + b*sin(t)*cos(theta)
    shapexy = lambda q: (q[2] * cos(2 * math.pi * s) * cos(q[3]) - (area / (math.pi * q[2])) * sin(
        s * 2 * math.pi) * sin(q[3]) + q[0],
                         (area / (math.pi * q[2])) * sin(2 * math.pi * s) * cos(q[3]) +
                         q[2] * cos(s * 2 * math.pi) * sin(q[3]) + q[1])

    return shapexy


def egg_shape(scale=1):
    def egg(q):
        c = Symbol('c')
        area = 0.625 * math.pi
        f = sqrt(16 * area / (5 * pi) - c ** 2)

        r = Piecewise(
            ((q[2] - 1) * sin(pi * s) ** 3 + f.subs(c, q[2] - 1) * cos(pi * s) ** 3, And(q[2] >= 0, q[2] < 2)),
            (f.subs(c, q[2] - 3) * sin(pi * s) ** 3 - (q[2] - 3) * cos(pi * s) ** 3, And(q[2] >= 2, q[2] < 4)),
            ((5 - q[2]) * sin(pi * s) ** 3 - f.subs(c, q[2] - 5) * cos(pi * s) ** 3, And(q[2] >= 4, q[2] < 6)),
            (-f.subs(c, q[2] - 7) * sin(pi * s) ** 3 + (q[2] - 7) * cos(pi * s) ** 3,
             # True),
             And(q[2] >= 6, q[2] <= 8)),
            (0, True)
        )

        x = r * cos(pi * s) * scale + q[0]
        y = r * sin(pi * s) * scale + q[1]

        return x, y

    return egg


# def star_shape(points=5, rad=10):  # 160
#     # shapexy = lambda q: (.2 * cos(points * math.pi * s) * rad*cos(2 * math.pi * s) + q[0],
#     #                      .2 * cos(points * math.pi * s) * rad*sin(2 * math.pi * s) + q[1])
#     shapexy = lambda q: (10.2 * cos(points * math.pi * s) + rad * cos(2 * math.pi * s) + q[0],
#                          10.2 * cos(points * math.pi * s) + rad * sin(2 * math.pi * s) + q[1])
#     return shapexy
