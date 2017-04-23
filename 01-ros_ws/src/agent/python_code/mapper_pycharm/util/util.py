import numpy as np
from operator import itemgetter

from shapely.geometry import Polygon

from config import numeric_shapes, analytic_shapes
from copy import copy

def shape_factory_numeric(shape_name):
    return getattr(numeric_shapes, shape_name)


def shape_factory_analytic(shape_name):
    return getattr(analytic_shapes, shape_name)


def shape2polygon(sx, sy, q, s):
    """
    Obtain a polygon from a shape function
    """
    psx = sx(s, q)
    psy = sy(s, q)
    return Polygon([(x, y) for x, y in zip(psx, psy)])

def Convert2Polar(x,y,xc,yc):
    if isinstance(x, float):
        x = [x]
        y = [y]
        n = 1
    else:
        n=len(x)
    rho = np.zeros(n)
    phi = np.zeros(n)
    for i in range(0,n):
        rho[i] = np.sqrt((x[i]-xc)**2.+(y[i]-yc)**2.)
        phi[i] = np.arctan2((y[i]-yc),(x[i]-xc))
        #if phi[i] < 0.:
        #    phi[i] += 2 * np.pi
    return rho, phi


def ComputeAllAvg(phi,failure=False,failure_num=None):
    n = len(phi)
    phi = list(phi)

    if failure:
        n -= 1
        phi.pop(failure_num)

    index, phi = map(list, zip(*sorted(enumerate(phi), key=itemgetter(1)))) # order the phi's

    # to make a ring like structure
    phi.append(phi[0])
    phi.insert(0, phi[-2])

    phi_av = np.zeros(n)
    for i in range(1,n+1):
        #angle_diff = np.arctan2(np.sin(phi[i+1] - phi[i-1]), np.cos(phi[i+1] - phi[i-1])) # minimal difference between angles
        angle_diff = phi[i+1]-phi[i-1]
        if angle_diff<0:
            angle_diff += 2*np.pi
        phi_av[i - 1] = angle_diff / 2. + phi[i - 1]

        if phi_av[i - 1]>2.*np.pi:
            phi_av[i - 1]-=2.*np.pi
        elif  phi_av[i - 1]<0.:
            phi_av[i - 1]+=2.*np.pi

    _, phi_av = map(list, zip(*sorted(zip(index,phi_av), key=itemgetter(0))))

    if failure:
        phi_av.insert(failure_num,0)

    return np.array(phi_av)


def toSecsNsecs(secs):
    s = int(secs)
    ns = (secs-s)*1e9
    return s,ns

def toSecs(Time):
    return Time.secs+Time.nsecs/1e9

def get_from_list(list, filter):
    # use with filter = lambda x: x.id == id
    i=0
    for x in list:
        if filter(x):
            return True,x,i
        i+=1
    # returns result,obj,idx
    return False,list[-1],len(list)-1

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def delete_children(nodes,id):
    new_list = copy(nodes)
    success,node,idx = get_from_list(new_list, lambda x: x.id == id)
    for child in node.children:
        new_list = delete_children(new_list, child.id)
    new_list.remove(node)
    node.parent.children.remove(node)
    return new_list

def shift_all_times_branch(list,id,dt):
    result, obj, idx = get_from_list(list, lambda x: x.id == id)
    for node in list[idx:]:
        if node.parent == None:
            node.time += dt
        else:
            node.time = node.parent.time+dt

def remove_branch(list,element):
    new_list = []
    ids_to_delete = [element.id]
    for ele in list:
        if ele.id in ids_to_delete:
            for child in ele.children:
                ids_to_delete.append(child.id)
        else:
            new_list.append(ele)
    return new_list
