import numpy as np
import rospy
from shapely.geometry import Polygon, Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Quaternion

import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

class Map(object):
    """ 
    The Map class stores an occupancy grid as a two dimensional
    numpy array. 
    
    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.
        
    
    Note that x increases with increasing column number and y increases
    with increasing row number. 
    """

    def __init__(self, origin_x=None, origin_y=None, resolution=None,
                 width=None, height=None,basis=None,occupancy_grid=None,all_ones=False):
        """ Construct an empty occupancy grid.
        
        Arguments: origin_x, 
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells 
                                in meters.
                   width, 
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
                                
         The default arguments put (0,0) in the center of the grid. 
         :type occupancy_grid: OccupancyGrid
         :type basis: Map

        """

        if basis != None:
            self.origin_x = origin_x
            self.origin_y = origin_y
            self.resolution = resolution
            self.width = width
            self.height = height
            self.grid = np.zeros((height/resolution, width/resolution))

            # some map is given as a basis, mapping from polygon based representation to occupancy grid
            E, Obs, start, goal = basis
            for O in Obs:
                bnds = map(lambda x: int(round(x)), O.bounds)
                for x in range(bnds[0],bnds[2]+1):
                    for y in range(bnds[1], bnds[3]+1):
                        #point = Point((x,y))
                        #if point.within(O): #TODO: so far all obstacles are exact rectangles, no expensive polygon check necessary
                        self.set_cell(x, y, 1.)
                        #self.grid[x][y] = 1.
        elif occupancy_grid!=None:
            self.origin_x = occupancy_grid.info.origin.position.x
            self.origin_y = occupancy_grid.info.origin.position.y
            self.resolution = occupancy_grid.info.resolution
            self.width = occupancy_grid.info.width*self.resolution
            self.height = occupancy_grid.info.height*self.resolution
            self.grid = np.array(occupancy_grid.data).reshape((self.height/self.resolution, self.width/self.resolution))/100
        else:
            if origin_x==None or origin_y==None or resolution==None or width==None or height==None:
                raise ValueError('if no Map or OccupancyGrid object is given, all values are necessary')
            self.origin_x = origin_x
            self.origin_y = origin_y
            self.resolution = resolution
            self.width = width
            self.height = height
            if all_ones:
                self.grid = np.ones((height/self.resolution, width/self.resolution))
            else:
                self.grid = np.zeros((height / self.resolution, width / self.resolution))


    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = int(self.width/self.resolution)
        grid_msg.info.height = int(self.height/self.resolution)
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.
        grid_msg.info.origin.orientation = Quaternion(0, 0, 0, 1)

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        grid_msg.data = np.array(self.grid.reshape((self.grid.size,)) * 100, dtype=np.int8)

        #grid_msg.data = [int(num* 100) for num in self.grid.reshape((self.grid.size,))]
        return grid_msg

    def set_cell(self, x, y, val):
        """ Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method!  Feel free to throw out
        point that land outside of the grid. 
        """
        if self.width/self.resolution <= x or self.height/self.resolution <= y or x<0 or y<0:
            return
            #raise ValueError("cannot set value outside the grid")
        self.grid[y][x] = val

    def get_cell(self,x,y):
        return self.grid[y][x]

