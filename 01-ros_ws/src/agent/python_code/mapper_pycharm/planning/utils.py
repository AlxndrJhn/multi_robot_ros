import numpy as np


def emergency_brake(x, U):
    n = len(x)
    V = x[n/2:]

    final_p = []
    for p, v, (a_min,a_max) in zip(x[:n/2], V, U):
        #t = np.abs(v)/a_max
        #a = np.sign(v)*-1.
        d = v**2/(2*a_max)
        final_p.append(p+np.sign(v)*d)
        #final_p.append(p+t*v+1./2.*a*t**2)
    for i in range(n/2):
        final_p.append(0.)
    return final_p





#print emergency_brake([0,3,0,10],[(-1,1),(-1,1)])
