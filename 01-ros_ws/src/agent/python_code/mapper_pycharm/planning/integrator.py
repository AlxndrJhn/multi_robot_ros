import numpy as np


def integrator(lx, aqs, t1, t2):
    """
    :param lx: last state
    :param a:  acceleration
    """

    lx = np.copy(lx)
    # Degrees of freedom
    n = len(aqs)

    # continuous
    cfx = [[] for i in range(2 * n)]

    # Integration parameters
    params = []

    for i, aq in enumerate(aqs):
        # velocity
        bq = lx[n + i] - aq * t1
        vq = aq * t2 + bq

        # print "new v", i,vq, "aq", aq, "b",bq
        # position
        cq = lx[i] - (aq * (t1 ** 2) / 2 + bq * t1)
        q1 = aq * (t2 ** 2) / 2 + bq * t2 + cq

        params.append((aq, bq, cq))

        # update last state
        # print "new state", i, q1
        lx[i] = q1
        lx[n + i] = vq

    return lx, params


def params_to_quadratic(time, params):
    ax, bx, cx = params
    return ax * (time ** 2) / 2 + bx * time + cx
