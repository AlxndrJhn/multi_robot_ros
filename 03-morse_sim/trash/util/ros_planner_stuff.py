import rospy
import numpy as np
from agent.msg import bid
from nav_msgs.msg import OccupancyGrid
from shapely.geometry import Polygon,Point
from ros_map_server import Map

distance2goal = lambda x, goal: np.sqrt((x[0] - goal[0]) ** 2 + (x[1] - goal[1]) ** 2)

class Bid(object):
    def __init__(self,id=None, cost=None, params=None, add_info = ""):

        self.bid = bid()

        self.bid.stamp = rospy.Time.now() #time
        self.robot_id = id #uint32
        self.add_info = add_info #string
        self.cost = cost #float32
        self.params = params #float32[] -> list

def shift_grid(grid,offset,axis):
    new_grid = np.roll(grid, offset, axis=axis)
    if axis==0:
        if offset==1:
            new_grid[-1, :] = 0
        else:
            new_grid[0, :] = 0
    else:
        if offset==1:
            new_grid[:, -1] = 0
        else:
            new_grid[:, 0] = 0
    return new_grid

def grow_obstacles(map,dist):
    """

    :type dist: float
    :type map: Map
    """
    num_cells = int(np.ceil(dist/map.resolution))

    # go through all cells, if occupied then occupucy all adjacent nodes (up down left right)
    grid_temp = map.grid
    grid_result = map.grid
    for i in range(num_cells):
        grid_result += shift_grid(grid_temp, 1, axis=0)  # offset original map by 1 north and add to temp map
        grid_result += shift_grid(grid_temp, -1, axis=0) # offset original map by 1 south and add to temp map
        grid_result += shift_grid(grid_temp, 1, axis=1)  # offset original map by 1 west and add to temp map
        grid_result += shift_grid(grid_temp, -1, axis=1) # offset original map by 1 east and add to temp map
        grid_temp = grid_result

    grid_result[(grid_result > 1)] = 1 # reset all large cells to 1
    return grid_result

def check_in_state_space(x, X):
    for x1, (xmin, xmax) in zip(x, X):
        # Outside the interval defined by xs
        if x1 < xmin or x1 > xmax:
            return False
    return True

def collision_check_oq(polygon, map_obj):
    """

    :type map: Map
    :type polygon: Polygon
    """

    bnds = map(lambda x: int(round(x / map_obj.resolution)), polygon.bounds)
    if bnds[0] < 0 or bnds[1] < 0 or bnds[2] + 1 > map_obj.width / map_obj.resolution or \
       bnds[3] + 1 > map_obj.height / map_obj.resolution:
        return True
    # sub_map = self.map_exp[range(bnds[1], bnds[3] + 1),range(bnds[0], bnds[2] + 1)]
    sub_map = map_obj.grid[bnds[1]:bnds[3] + 1, bnds[0]:bnds[2] + 1]

    # check if some occupied point is within the polygon, if not, valid state
    if sub_map[(sub_map == 1)]:
        return True
    return False

    # get small sub-grid depending on the bounds of the polygon
    bnds = map(lambda x: int(round(x / map.resolution)), polygon.bounds)
    if bnds[0] < 0 or bnds[1] < 0 or bnds[2] + 1 > map.width or bnds[3] + 1 > map.height:
        return True
    sub_map = map.grid[range(bnds[1], bnds[3] + 1)][range(bnds[0], bnds[2] + 1)]

    # check if any occupied cell (==1) from sub_map is inside the polyon
    for (x, y), value in np.ndenumerate(sub_map):
        if value:
            y = (bnds[1] + y) * map.resolution
            x = (bnds[0] + x) * map.resolution
            if polygon.contains(Point((x,y))):
                return True
    return False


