from math import sqrt
import numpy as np
from config.maps import points_to_rectangle

distance2goal = lambda x, goal: sqrt((x[0] - goal[0]) ** 2 + (x[1] - goal[1]) ** 2)


def choose_closer_node_euclidean(nodes1, goal1, k=10):
    """
    Choose a node among the k closest to the goal.
    :param nodes1: set of nodes
    :param goal1: goal
    :param k: number of nodes to choose.
    :return: one of the k closest
    """
    # squared distance to the goal. Sqrt is not required, because it is proportional
    node2goal = lambda node: distance2goal(node.x, goal1)
    sorted_nodes = sorted(nodes1, key=node2goal)

    # return one of the k closest nodes.
    return np.random.choice(sorted_nodes[:k])


def check_collision(polygon, obstacles, environment, Obs_type=[]):
    """
    Check for collsions with obstacles and environmental limits.
    :param polygon: set of points of the shape
    :param obstacles: obstacles in the map
    :return: True if there is collision.
    """

    if Obs_type==[]:
        Obs_type = [0 for obs in obstacles]

    # polp1 = Polygon(pol1)
    for i,obs in enumerate(obstacles):
        if not Obs_type[i] and obs.intersects(polygon):
            return True

    return not polygon.union(environment).area == environment.area


# def close2goal(x, goal1, min_distance=10):
#     # Euclidean distance.
#     return math.sqrt((x[0] - goal1[0]) ** 2 + (x[1] - goal1[1]) ** 2) < min_distance


def check_in_state_space(x, X):
    for x1, (xmin, xmax) in zip(x, X):
        # Outside the interval defined by xs
        if x1 < xmin or x1 > xmax:
            return False
    return True


def grow_obstacles(Obs, incrase_param):
    new_obs = [obs.buffer(incrase_param) for obs in Obs]

    return new_obs
    

def kinodyn_metric(q_start,q_end,X,U):
    # see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.474.4758&rep=rep1&type=pdf equation (9) and following
    D = len(q_start)
    has_infeasible_time = np.zeros(D)
    lower_bounds = np.zeros(D)
    upper_bounds = np.zeros(D)

    T=np.zeros(D)
    for i in range(0,D/2):
        p1 = q_start[i]
        p2 = q_end[i]
        v1 = q_start[i+D/2]
        v2 = q_end[i+D/2]
        a_max = U[i][1]
        v_max = X[i+D/2][1]

        if (p1==p2 and v1==v2):
            continue

        delta_pacc = 1.0/2.0*(v1+v2)*np.abs(v2-v1)/a_max
        sigma=np.sign(p2-p1-delta_pacc)
        if sigma==0.0:
            sigma=-1.0
        a1=sigma*a_max
        a2=-a1
        v_limit=sigma*v_max
        roots = np.roots([a1, 2.0*v1, (v2**2.0-v1**2.0)/(2.0*a2)-(p2-p1)])
        t_a1 = max(roots)
        is_valid = True
        if sigma > 0.0:
            if t_a1 * a1 + v1 > v_limit:
                # velocity too high
                is_valid = False

        else:
            if t_a1 * a1 + v1 < v_limit:
                # velocity too low
                is_valid = False

        if is_valid:
            t_a2 = (v2 - v1) / a2 + t_a1
            T[i] = t_a1 + t_a2
        else:
            # see equations (15)-(17)
            t_a1=(v_limit-v1)/a1
            t_v=(v1**2.0+v2**2.0-2.0*v_limit**2.0)/(2.0*v_limit*a1)+(p2-p1)/v_limit
            t_a2=(v2-v_limit)/a2
            T[i] = t_a1 + t_v + t_a2

    return max(T)
    #     # Infeasible time interval, too slow
    #     # infeasible time for region I
    #     if np.sign(v1) != np.sign(v2) or v1==0.0 or v2==0.0:
    #         continue
    #
    #     if np.sign(v1) != np.sign(p2-p1): # region V
    #         continue
    #
    #     t_ac = (np.abs(v2-v1)/a_max)
    #     #diff_p = 1.0/2.0*a_max*t**2
    #     p_V2 = np.sign(v2-v1) * 1.0 / 2.0 * a_max * t_ac ** 2
    #     p_V1 = (v2-v1)*t_ac
    #     if p_V1-p_V2>(p2-p1): # region V
    #         continue
    #
    #     # region II
    #     t_1 = np.abs(v1)/a_max
    #     t_2 = np.abs(v2)/a_max
    #     p_II = 1.0 / 2.0 * a_max * (t_1 ** 2 + t_2 ** 2)
    #     if p_II < np.abs(p2 - p1):
    #         continue
    #
    #
    #     # switching a1 a2 vlimit signs
    #     a1 = -a1
    #     a2 = -a2
    #     v_limit = -v_limit
    #     roots = np.roots([a1, 2.0*v1, (v2**2.0-v1**2.0)/(2.0*a2)-(p2-p1)])
    #     t_lower_bound = min(roots)
    #     t_upper_bound = max(roots)
    #
    #     if T[i]>=t_lower_bound or t_lower_bound<=0.0 or t_upper_bound<=0.0:
    #         continue
    #
    #     has_infeasible_time[i] = True
    #
    #     is_valid = True
    #     if a1 < 0.0:
    #         if t_upper_bound * a1 + v1 < v_limit:
    #             # velocity too low
    #             is_valid = False
    #     else:
    #         if t_upper_bound * a1 + v1 > v_limit:
    #             # velocity too high
    #             is_valid = False
    #
    #     if not is_valid:
    #         # see equations (15)-(17)
    #         t_a1=(v_limit-v1)/a1
    #         t_v=(v1**2.0+v2**2.0-2.0*v_limit**2.0)/(2.0*v_limit*a1)+(p2-p1)/v_limit
    #         t_a2=(v2-v_limit)/a2
    #         t_upper_bound = t_a1 + t_v + t_a2
    #
    #     lower_bounds[i] = t_lower_bound
    #     upper_bounds[i] = t_upper_bound
    #
    # if has_infeasible_time.any():
    #     T_lowest = max(T)
    #     repeat=True
    #     while repeat:
    #         repeat=False
    #         for lower,upper in zip(lower_bounds[has_infeasible_time==True],upper_bounds[has_infeasible_time==True]):
    #             if T_lowest > lower and T_lowest<upper:
    #                 T_lowest = upper
    #                 repeat = True
    #     return T_lowest
    # else:
    #     return max(T)

def saturate(q_start, q_end, X, U, t1, t2):
    # Steers from q_start to q_end for t2-t1 seconds while respecting the state space and input restrictions
    # see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.474.4758&rep=rep1&type=pdf equation (9) and following
    D = len(q_start)
    q_sat = np.zeros(D)
    dt = t2-t1

    for i in range(0, D / 2):
        p1 = q_start[i]
        p2 = q_end[i]
        v1 = q_start[i + D / 2]
        v2 = q_end[i + D / 2]
        a_max = U[i][1]
        v_max = X[i + D / 2][1]

        if (p1 == p2 and v1 == v2):
            q_sat[i] = p1
            q_sat[i + D / 2] = v1
            continue

        delta_pacc = 1.0 / 2.0 * (v1 + v2) * np.abs(v2 - v1) / a_max
        sigma = np.sign(p2 - p1 - delta_pacc)
        if sigma==0.0:
            sigma=-1.0
        a1 = sigma * a_max
        a2 = -a1
        v_limit = sigma * v_max
        roots = np.roots([a1, 2.0*v1, (v2**2.0-v1**2.0)/(2.0*a2)-(p2-p1)])

        # t_a1=max(np.hstack((roots,0.0)))
        t_a1 = max(roots)
        is_valid = True
        if sigma > 0.0:
            if t_a1 * a1 + v1 > v_limit:
                # velocity too high
                is_valid = False

        else:
            if t_a1 * a1 + v1 < v_limit:
                # velocity too low
                is_valid = False

        if is_valid:
            t_a2 = (v2 - v1) / a2 + t_a1
            #T = t_a1 + t_a2

            # for the case of bang bang
            if t_a1 >= dt:
                q_sat[i] = p1 + 0.5 * a1 * dt ** 2 + dt*v1
                q_sat[i + D / 2] = v1 + a1 * dt
            elif t_a1+t_a2>=dt: # t_a1 < dt
                partial_t_a2 = dt - t_a1 # partial_t_a2<=t_a2
                q_sat[i] = p1 + 0.5 * a1 * t_a1 ** 2 + t_a1*v1 + 0.5 * a2 * partial_t_a2 ** 2 + partial_t_a2*(v1 + a1 * t_a1)
                q_sat[i + D / 2] = v1 + a1 * t_a1 + a2 * partial_t_a2
            else:#t_a1+t_a2<dt and t_a1 < dt
                rest_time = dt-t_a1-t_a2
                unwanted_motion = rest_time*(v1 + a1 * t_a1 + a2 * t_a2)
                q_sat[i] = p1 + 0.5 * a1 * t_a1 ** 2 + t_a1*v1 + 0.5 * a2 * t_a2 ** 2 + t_a2*(v1 + a1 * t_a1) + unwanted_motion
                q_sat[i + D / 2] = v1 + a1 * t_a1 + a2 * t_a2
        else:
            # see equations (15)-(17)
            t_a1 = (v_limit - v1) / a1
            t_v = (v1 ** 2.0 + v2 ** 2.0 - 2.0 * v_limit ** 2.0) / (2.0 * v_limit * a1) + (p2 - p1) / v_limit
            t_a2 = (v2 - v_limit) / a2
            #T = t_a1 + t_v + t_a2

            # for the case of bang wait bang
            if t_a1 >= dt:
                q_sat[i] = p1 + 0.5 * a1 * dt ** 2 + dt*v1
                q_sat[i + D / 2] = v1 + a1 * dt
            elif t_a1+t_v>=dt: # t_a1 < dt
                partial_t_v = dt - t_a1 # partial_t_v<=t_v
                q_sat[i] = p1 + 0.5 * a1 * t_a1 ** 2 + t_a1*v1 + partial_t_v * (v1 + a1 * t_a1)
                q_sat[i + D / 2] = v1 + a1 * t_a1
            elif t_a1+t_v+t_a2>=dt:
                partial_t_a2 = dt - (t_a1+t_v)  # partial_t_a2<=t_a2
                q_sat[i] = p1 + 0.5 * a1 * t_a1 ** 2 + t_a1*v1 + t_v * (v1 + a1 * t_a1) + 0.5 * a2 * partial_t_a2 ** 2 + partial_t_a2 * (v1 + a1 * t_a1)
                q_sat[i + D / 2] = v1 + a1 * t_a1 + a2*partial_t_a2
            else:#t_a1+t_v+t_a2<dt
                rest_time = dt-t_a1-t_a2-t_v
                unwanted_motion = rest_time*(v1 + a1 * t_a1 + a2 * t_a2)
                q_sat[i] = p1 + 0.5 * a1 * t_a1 ** 2 + t_a1*v1 + t_v * (v1 + a1 * t_a1) + 0.5 * a2 * t_a2 ** 2 + t_a2 * (v1 + a1 * t_a1) + unwanted_motion
                q_sat[i + D / 2] = v1 + a1 * t_a1 + a2*t_a2

    return q_sat

def saturate_params(q_start, q_end, X, U, t1, t2):
    # Steers from q_start in direction of q_end for t2-t1 seconds
    # see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.474.4758&rep=rep1&type=pdf equation (9) and following
    D = len(q_start)
    q_sat = np.zeros(D)
    dt = t2 - t1
    q_params = []

    for i in range(0, D / 2):
        p1 = q_start[i]
        p2 = q_end[i]
        v1 = q_start[i + D / 2]
        v2 = q_end[i + D / 2]
        a_max = U[i][1]
        v_max = X[i + D / 2][1]

        if (p1 == p2 and v1 == v2):
            q_sat[i] = p1
            q_sat[i + D / 2] = v1
            q_params.append((0.0, v1, p1-v1*t1))
            continue

        delta_pacc = 1.0 / 2.0 * (v1 + v2) * np.abs(v2 - v1) / a_max
        sigma = np.sign(p2 - p1 - delta_pacc)
        if sigma == 0.0:
            sigma = -1.0
        a1 = sigma * a_max
        a2 = -a1
        v_limit = sigma * v_max
        roots = np.roots([a1, 2.0*v1, (v2**2.0-v1**2.0)/(2.0*a2)-(p2-p1)])

        t_a1 = max(roots)

        is_valid = True
        if sigma > 0.0:
            if t_a1 * a1 + v1 > v_limit:
                # velocity too high
                is_valid = False

        else:
            if t_a1 * a1 + v1 < v_limit:
                # velocity too low
                is_valid = False

        if is_valid:
            t_a2 = (v2 - v1) / a2 + t_a1
            T = t_a1 + t_a2

        else:
            # see equations (15)-(17)
            t_a1 = (v_limit - v1) / a1
            t_v = (v1 ** 2.0 + v2 ** 2.0 - 2.0 * v_limit ** 2.0) / (2.0 * v_limit * a1) + (p2 - p1) / v_limit
            t_a2 = (v2 - v_limit) / a2
            T = t_a1 + t_v + t_a2

        if t_a1<dt:
            if t_a1 < dt-(t_a1 + t_a2):
                a1 = -a1*min(dt-t_a1, t_a2)/dt
            else:
                a1 = t_a1 / dt * a1

        if np.abs(dt * a1 + v1)>v_max:
            a1 = (v_limit - v1) / dt


        # for the case of bang bang
        q_sat[i] = p1 + 0.5 * a1 * dt ** 2 + dt * v1
        q_sat[i + D / 2] = v1 + a1 * dt

        #cq = p1 - (a1 * (t1 ** 2) / 2 + v1 * t1)
        #bq = v1 - a1 * t1

        bq = v1 - a1 * t1
        cq = p1 - (a1 * (t1 ** 2) / 2 + bq * t1)
        #q1 = a1 * (t2 ** 2) / 2 + bq * t2 + cq
        #vq = a1 * t2 + bq
        #print q_sat[i]-q1
        #print q_sat[i + D / 2] - vq
        #print np.abs(a1/2*t1**2+bq*t1+cq-p1)
        #print np.abs(a1/2*t2**2+bq*t2+cq-q1)

        q_params.append((a1, bq, cq))

    return q_sat,q_params
    
    
    
    
    
    
    
    
    
    
