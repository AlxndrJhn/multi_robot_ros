from sympy import *
from sympy import init_printing
import sympy as sp
import math

import pylab
import numpy as np
import matplotlib.pylab as plt

# from anomaly_common import theta, t
from scipy import integrate

yt = Symbol('y_t')
xt = Symbol('x_t')
yh = Symbol('y_\\theta')
xh = Symbol('x_\\theta')
v2 = var('v^2')
dot_theta = Symbol('\dot{ \\theta}')

#### Defining symbos t,and theta.
t = Symbol('t')
s = Symbol('s')
v2 = var('v^2')


def theta_path_general_ode():
    """
    General form of the differential equation for the robot path

    """
    inic = yt ** 2 + 2 * yt * yh * dot_theta + (yh * dot_theta) ** 2 + xt ** 2 + 2 * xt * xh * \
                                                                                 dot_theta + (xh * dot_theta) ** 2
    raiz_theta = solve(inic - v2, dot_theta)[0]
    return raiz_theta


def trajectory_ode(fx, fy):
    """
    Specific form of the differential equation for an anomaly function.
    """
    raiz_theta = theta_path_general_ode()
    # ordinary differential equation.
    ode = raiz_theta.subs({
        yt: fy.diff(t),
        xt: fx.diff(t),
        yh: fy.diff(s),
        xh: fx.diff(s),
        # t_final: 200
    })
    # f = simplify(f)

    return ode


def boundary_derivative(fx, fy, vel):
    # Derivative of S
    s_dot = trajectory_ode(fx, fy).subs(v2, vel)

    x_dot = fx.diff(t) + fx.diff(s) * s_dot
    y_dot = fy.diff(t) + fy.diff(s) * s_dot

    return x_dot, y_dot


#### Numerical methods ######

def _RK4(f):
    return lambda t, y, dt: (
        lambda dy1: (
            lambda dy2: (
                lambda dy3: (
                    lambda dy4: (dy1 + 2 * dy2 + 2 * dy3 + dy4) / 6
                )(dt * f(t + dt, y + dy3))
            )(dt * f(t + dt / 2, y + dy2 / 2))
        )(dt * f(t + dt / 2, y + dy1 / 2))
    )(dt * f(t, y))


###FIXME TMP  % (2. * math.pi) for range [0,2pi]
def _RK4_trunk(f):
    return lambda t, y, dt: (
        lambda dy1: (
            lambda dy2: (
                lambda dy3: (
                    lambda dy4: (dy1 + 2 * dy2 + 2 * dy3 + dy4) / 6
                )(dt * f(t + dt, (y + dy3) % (2 * pi)))
            )(dt * f(t + dt / 2, (y + dy2 / 2) % (2 * pi)))
        )(dt * f(t + dt / 2, (y + dy1 / 2) % (2 * pi)))
    )(dt * f(t, y))


def solve_rk4(ode1, vel2, theta0, time, trunk=True):
    """
    Solve the ode to obtain the robot path.
    :param ode: differential equation for an anomaly function.
    :param vel2: float, constant velocity
    :param theta0: initial theta
    :param ti: initial time
    :param tf: final time
    :param n: number of intervals.
    :return:
    """
    dt = time[1] - time[0]

    # function to evaluate the ode with constant velocity as vel2.
    fv = lambda t1, th1: ode1.subs(s, th1).subs(t, t1).subs(v2, vel2).evalf()

    # Run runge kutta
    if trunk:
        dy = _RK4_trunk(fv)
    else:
        dy = _RK4(fv)

    # convert the result
    atheta = [theta0]
    for t1 in time[:-1]:
        y1 = atheta[-1] + dy(t1, atheta[-1], dt)
        ###FIXME TMP  % (2. * math.pi) for range [0,2pi]
        if trunk:
            atheta.append(y1 % (2. * math.pi))
        else:
            atheta.append(y1)

    atheta = np.array(atheta).astype('float')

    return time, atheta


def solve_ode(ode1, vel2, theta0, time):
    """
    An better method to solve the Ordinary Differential Equation.
    :param ode: differential equation for an anomaly function.
    :param vel2: float, constant velocity
    :param theta0: initial theta
    :param time: time array
    """
    ode1 = ode1.subs(v2, vel2)
    lamode = lambdify((s, t), ode1)

    # def func(th1, t1):  # f(x) function
    #     return ode1.subs({t: t1, s: th1, v2: vel2}).evalf()

    atheta = integrate.odeint(lamode, theta0, time)
    return atheta


def robot_path(fx, fy, time, st):
    lambd_fx = lambdify((t, s), fx, np)
    lambd_fy = lambdify((t, s), fy, np)
    xx = [lambd_fx(tt, thetat) for tt, thetat in zip(time, st)]
    yy = [lambd_fy(tt, thetat) for tt, thetat in zip(time, st)]

    return xx, yy


def robot_vel(fx, fy, vel, time, st):
    # Boundary velocity
    vel_fx, vel_fy = boundary_derivative(fx, fy, vel)

    lambd_fx = lambdify((t, s), vel_fx, np)
    lambd_fy = lambdify((t, s), vel_fy, np)

    xx = [lambd_fx(tt, thetat) for tt, thetat in zip(time, st)]
    yy = [lambd_fy(tt, thetat) for tt, thetat in zip(time, st)]

    return xx, yy
