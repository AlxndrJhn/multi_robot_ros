import rospy
import numpy as np
from agent.msg import bid
from nav_msgs.msg import OccupancyGrid
from shapely.geometry import Polygon,Point
from ros_map_server import Map
from itertools import chain
from copy import copy
distance2goal = lambda x, goal: np.sqrt((x[0] - goal[0]) ** 2 + (x[1] - goal[1]) ** 2)

def shift_grid(grid,offset,axis):
    new_grid = np.roll(grid, offset, axis=axis)

    # the roll method shifts the elements from one side around to the other side, to avoid distortions, the wrapped
    # values are reset
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
    temp_map = copy(map)
    # go through all cells, if occupied then occupucy all adjacent nodes (up down left right)
    grid_temp = copy(temp_map.grid)
    grid_result = copy(temp_map.grid)
    for i in range(num_cells):
        grid_result += shift_grid(grid_temp, 1, axis=0)  # offset original map by 1 north and add to temp map
        grid_result += shift_grid(grid_temp, -1, axis=0) # offset original map by 1 south and add to temp map
        grid_result += shift_grid(grid_temp, 1, axis=1)  # offset original map by 1 west and add to temp map
        grid_result += shift_grid(grid_temp, -1, axis=1) # offset original map by 1 east and add to temp map
        grid_temp = grid_result

    grid_result[(grid_result > 1)] = 1 # reset all large cells to 1
    temp_map.grid = grid_result
    return temp_map


def check_in_state_space(x, X):
    for x1, (xmin, xmax) in zip(x, X):
        # Outside the interval defined by xs
        if x1 < xmin or x1 > xmax:
            return False
    return True

def quick_collision_check(statev,map_obj):
    """

    :type map_obj: Map
    """
    res = map_obj.resolution
    x,y = statev[0:2]
    x_disc = int(round(x/res))
    y_disc = int(round(y/res))
    if x_disc<0 or y_disc<0 or y_disc>=map_obj.height/res or x_disc>=map_obj.width/res:
        return True
    return map_obj.get_cell(x_disc,y_disc)


def collision_check_oq(polygon, map_obj):
    """
    :type map: Map
    :type polygon: Polygon
    """
    sparsity_factor = 2 # only every i-th column and row is checked, default 1, 2->four times less checks

    bnds = map(lambda x: int(round(x / map_obj.resolution)), polygon.bounds)
    if bnds[0] < 0 or bnds[1] < 0 or bnds[2] >= map_obj.width / map_obj.resolution or \
       bnds[3] >= map_obj.height / map_obj.resolution:
        return True
    # sub_map = self.map_exp[range(bnds[1], bnds[3] + 1),range(bnds[0], bnds[2] + 1)]
    sub_map = map_obj.grid[bnds[1]:bnds[3] + 1, bnds[0]:bnds[2] + 1]
    if sub_map.any():
        for y in range(0,sub_map.shape[0]-1,sparsity_factor):
            for x in range(0,sub_map.shape[1]-1,sparsity_factor):
                if sub_map[y][x]>0:
                    if polygon.contains(Point(((x + bnds[0])* map_obj.resolution, (y + bnds[1])* map_obj.resolution))):
                        return True

    #polygon.contains(Point((bnds[1]*map_obj.resolution+)
    # check if some occupied point is within the polygon, if not, valid state

    #if len(sub_map[(sub_map == 1)])>0:
    #    return True
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


