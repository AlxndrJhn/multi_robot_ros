import numpy as np
import rospy
from shapely.geometry import Polygon, Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Quaternion
from planning.heuristic import kinodyn_metric
from planning.heuristic import saturate,saturate_params
import numpy.random as rnd
import sys
from util.ros_planner_stuff import grow_obstacles, check_in_state_space, collision_check_oq, distance2goal
from util.ros_map_server import Map
from util.util import shape2polygon, shape_factory_numeric
import matplotlib.pyplot as plt
from planning.Node import Node
from util.draw import draw_shape

class dimt_rrt(object):
    def __init__(self,exp):
        """

        :type exp: Experiment
        :type map: Map
        """

        # Mapping input variables to members
        self.exp = exp
        self.map = None
        self.map_exp = None
        self.mask = None # will be received from map_server node
        self.start_node = exp.q_init # extracted from experiment object

        # objects to return
        self.cost = None
        self.params = None
        self.has_solution = False

        # Shape definition
        self.s = np.linspace(0, 1, 20)
        self.A = self.exp.shape_params['area']
        self.sx, self.sy = shape_factory_numeric(exp.shape_name)(**exp.shape_params)

        # State and input space
        self.X = self.exp.X # State space
        self.n = len(self.start_node)+1 # Degrees of freedom, number of inputs in U, state space is n*2 dimensions
        self.U = self.exp.U# Control space.

        # Nodes
        self.nodes = [Node(state_vector=self.start_node,parent=None,q_params=None,time=rospy.get_time(),distance=None)]

        # RRT parameters
        self.goal = self.exp.q_final
        self.goal_bias = self.exp.other_params['goal_bias']
        self.dt = self.exp.tree_edge_dt
        self.goal_dist_eucl = self.exp.goal_distance  # Termination criteria

        # Visualization
        self.fig = None

    def update_mask(self,mask):
        self.mask = mask

    def update_map(self,new_map):
        self.map_exp = grow_obstacles(new_map,self.exp.robot_size)
        self.map = new_map

    def update_start_node(self,new_node):
        # check if start node changes

        # try to reuse the tree
        pass

    def extend(self):
        if self.map == None or self.mask== None:
            print "Can't extend tree if map or map_mask is missing"
            return
        # random free sample x_rnd from X
        while True:
            x_rnd = [x1 + (x2 - x1) * rnd.rand() for (x1, x2) in self.X]
            polygon = shape2polygon(self.sx, self.sy, x_rnd[:self.n], self.s)
            bnds = map(lambda x: int(round(x/self.exp.resolution)), polygon.bounds)
            if bnds[0]<0 or bnds[1]<0 or bnds[2]+1>self.map.width/self.map.resolution or bnds[3]+1>self.map.height/self.map.resolution:
                continue
            #sub_map = self.map_exp[range(bnds[1], bnds[3] + 1),range(bnds[0], bnds[2] + 1)]
            sub_map = self.map_exp[bnds[1]:bnds[3] + 1, bnds[0]:bnds[2] + 1]

            # check if some occupied point is within the polygon, if not, valid state
            if sub_map[(sub_map == 1)]:
                continue

            #TODO: check if state lays in unavoidable collision region

            # otherwise, valid sample
            break

        # find closest node x_nearest in the tree
        shortest_dist = np.Inf
        shortest_node_id = -1
        selected_node = None
        for i, node in enumerate(self.nodes):  # skip root
            #dist = kinodyn_metric(node.x, x_rnd, self.X,
            #                      self.U)  # this metric is the min time solution to steer from node in the tree to xr
            dist = distance2goal(node.x, x_rnd)
            if dist <= shortest_dist:
                selected_node = node
                shortest_dist = dist
                shortest_node_id = i

        # extend x_nearest towards x_rnd
        while True:
            t1 = selected_node.time
            t2 = t1 + self.dt

            # TODO: fix saturate
            #x_new, q_params = saturate_params(selected_node.x, x_rnd, self.X, self.U, t1, t2)
            ptr = x_rnd[0:2]-selected_node.x[0:2]
            x_new = np.concatenate((ptr/np.linalg.norm(ptr)*0.03+selected_node.x[0:2],selected_node.x[2:]), axis=1)
            q_params = [(0.,0.,0.),(0.,0.,0.),(0.,0.,0.)]

            # Check if the shape satisfies the state space
            if not check_in_state_space(x_new, self.X):
                print "outside of X"
                break

            # Convert current shape to a polygon.
            polygon = shape2polygon(self.sx, self.sy, x_new[:self.n], self.s)

            ## Check collision with obstacles
            if collision_check_oq(polygon=polygon, map_obj=self.map):
                print "crashed"
                break #crashed
            else:
                new_node = Node(x_new)
                new_node.parent = selected_node
                # new_node.q_params = q_params
                new_node.time = t2
                #new_node.distance = kinodyn_metric(new_node.x, xf, X, U)
                # new_node.distance=distance2goal(new_node.x,xf)
                new_node.q_params = q_params
                self.nodes.append(new_node)
                print "node appended"

                # Goal check
                dist = distance2goal(new_node.x, self.goal)
                print "dist",dist
                if dist <= self.goal_dist_eucl:
                    self.solution = new_node.get_node_sequence()
                    print "solution found"
                    self.has_solution = True
                    self.cost = self.solution[-1].time-self.solution[0].time
                    self.params = self.solution[1].q_params
                    break

                if distance2goal(new_node.x, x_rnd)<= self.goal_dist_eucl:
                    print "arrived at x_rnd"
                    break

                # continue with the newly added node, go further in direction of the sample
                selected_node = new_node

    def draw_nodes(self,solution=False):
        if self.fig == None:
            self.fig = plt.figure()
            ax2 = self.fig.add_subplot(111)

        if solution:
            nodes = self.solution
        else:
            nodes = self.nodes
        for node in nodes:
            draw_shape(self.sx, self.sy, node.x[:self.n])
            plt.plot(node.x[0], node.x[1], '.')  # center dot

        plt.show(block=True)

