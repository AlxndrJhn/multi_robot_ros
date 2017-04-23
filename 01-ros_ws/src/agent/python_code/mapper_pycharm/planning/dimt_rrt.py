import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd

from planning.Node import Node
from planning.utils import emergency_brake
from util.draw import draw_shape
from util.ros_map_server import Map
from util.ros_planner_stuff import grow_obstacles, check_in_state_space, collision_check_oq, quick_collision_check, distance2goal
from util.util import shape2polygon, shape_factory_numeric,remove_branch
from util.util import toSecsNsecs, toSecs, delete_children, get_from_list, shift_all_times_branch
from copy import copy
from genpy import Time
import rospy
from scipy.spatial.distance import cdist
from time import sleep

from planning.heuristic import kinodyn_metric, saturate_params


class dimt_rrt(object):
    def __init__(self, exp, name, verbose=0, ignorePrior=False):
        """

        :type exp: Experiment
        :type map: Map
        """

        # Mapping input variables to members
        self.exp_obj = exp
        self.map = None
        self.map_exp = None
        self.mask_exp = None  # will be received from map_server node
        self.map_no_prior_exp = None
        self.map_no_prior_exp_collision_lookup_lb = None
        self.map_no_prior_exp_collision_lookup_ub = None
        self.map_exp_collision_lookup_lb = None
        self.map_exp_collision_lookup_ub = None
        self.mask_exp_collision_lookup_lb = None
        self.mask_exp_collision_lookup_ub = None
        self.start_conf = exp.q_init  # extracted from experiment object
        self.name = name
        self.ignorePrior = ignorePrior  # chance to ignore prior map completely
        self.verbose = verbose

        # objects to return
        self.cost = None
        self.params = None
        self.has_solution = False
        self.solution_node = None
        self.first_node = None
        self.second_node = None

        self.best_candidate = None

        # Shape definition
        self.s = np.linspace(0, 1, 10)
        self.A = self.exp_obj.shape_params['area']
        self.sx, self.sy = shape_factory_numeric(exp.shape_name)(**exp.shape_params)

        # State and input space
        self.X = self.exp_obj.X  # State space
        self.n = self.exp_obj.n_dim / 2  # Degrees of freedom, number of inputs in U, state space is n*2 dimensions
        self.U = self.exp_obj.U  # Control space.
        # self.ideal_velocity = np.sqrt(self.X[3][1] ** 2 + self.X[4][1] ** 2) # TODO: the ideal velocity leads to slow
        #                                                                        # exploration
        #self.ideal_velocity = 1
        # Nodes
        self.nodes = []  # Node(state_vector=self.start_node,parent=None,q_params=None,time=rospy.get_time(),distance=None)

        # RRT parameters
        self.goal = self.exp_obj.q_final
        self.goal_bias = self.exp_obj.other_params['goal_bias']
        self.dt = rospy.Duration(self.exp_obj.tree_edge_dt)
        self.goal_dist_eucl = self.exp_obj.goal_distance  # Termination criteria
        self.id_counter = -1
        self.ignore_prior = False
        self.upper_limit = np.Inf
        self.k_reduced_ratio = 5 # roughly the speedup factor

        # Cost function
        self.cost_heur_factor = 1 # 1 := as it should be, it always underestimates the true cost

        # Expansion of maps to avoid collision
        self.expand_factor_map = 1.  # times the robot size
        self.expand_factor_mask = 1.  # times the robot size, should be bigger or equal to self.expand_factor_map
        a_min,a_max = self.X[2]
        A = self.exp_obj.shape_params['area']
        b_min = A / (np.pi * a_max)
        b_max = A / (np.pi * a_min)
        self.R_min = min(a_min,b_min)   # thats the expansion of the obstacle map to have a fast lookup
                                        # that just uses the center point coordinates, it's sufficient that collision is guaranteed
        self.R_max = max(a_max,b_max)   # and checking a collision with a map that is additionally expanded by this R_max
                                        # is guranteed to be free, whatever the shape might look like

        # Visualization
        self.fig = None

        self.last_returned_candidate = None

        # Flags
        self.notification_missing_maps = False

    def update_start_time(self, time):
        self.nodes[0].time = time
        time = rospy.Duration(time.secs, time.nsecs)
        # diff_time = toSecs(time)
        for node in self.nodes[1:]:
            node.time += time
            # node.q_params = [(aq, bq - aq * diff_time, cq - bq * diff_time + aq / 2 * diff_time ** 2) for aq, bq, cq in node.q_params]
            # print 'node', node.id
            # print node.time
            # for i in range(0,3):
            #    aq, bq, cq = node.q_params[i]
            #    print aq * toSecs(node.parent.time) ** 2 / 2 + bq * toSecs(node.parent.time) + cq - node.parent.x[i],aq * toSecs(node.time) ** 2 / 2 + bq * toSecs(node.time) + cq - node.x[i]

    def change_map_margin(self, new_val=-1):
        if new_val == -1:
            self.map_exp = grow_obstacles(self.map, self.exp_obj.robot_size * self.expand_factor_map)
            self.map_no_prior_exp = grow_obstacles(self.map_no_prior, self.exp_obj.robot_size * self.expand_factor_map)
        else:
            self.map_exp = grow_obstacles(self.map, self.exp_obj.robot_size * new_val)
            self.map_no_prior_exp = grow_obstacles(self.map_no_prior, self.exp_obj.robot_size * new_val)

    def update_mask(self, mask):
        self.mask_exp = grow_obstacles(mask, self.exp_obj.robot_size * self.expand_factor_mask)
        self.mask_exp_collision_lookup_lb = grow_obstacles(self.mask_exp, self.R_min)
        self.mask_exp_collision_lookup_ub = grow_obstacles(self.mask_exp, self.R_max)
        self.mask_no_extension = mask

    def update_map(self, new_map):
        self.map_exp = grow_obstacles(new_map, self.exp_obj.robot_size * self.expand_factor_map)
        self.map_exp_collision_lookup_lb = grow_obstacles(self.map_exp, self.R_min)
        self.map_exp_collision_lookup_ub = grow_obstacles(self.map_exp, self.R_max)
        self.map = new_map

    def update_map_np(self, new_map_np):
        self.map_no_prior_exp = grow_obstacles(new_map_np, self.exp_obj.robot_size * self.expand_factor_map)
        self.map_no_prior_exp_collision_lookup_lb = grow_obstacles(self.map_no_prior_exp, self.R_min)
        self.map_no_prior_exp_collision_lookup_ub = grow_obstacles(self.map_no_prior_exp, self.R_max)

        self.map_no_prior = new_map_np

    def update_start_node(self, pos, time):
        pos = np.array(pos)  # in case its a tuple

        if isinstance(time, float):
            time = Time(*toSecsNsecs(time))


        # hard reset everytime, as the algorithm runs much faster now
        # delete all nodes
        new_start_node = Node(pos, time=time)
        new_start_node.time = time
        # new_start_node.dist = distance2goal(new_start_node.x, self.goal)
        new_start_node.dist = kinodyn_metric(new_start_node.x, self.goal, self.X, self.U)
        new_start_node.cost = 0
        new_start_node.id = self.get_next_id()
        new_start_node.q_params = [(0., 0., new_start_node.x[0]), (0., 0., new_start_node.x[1]),
                                   (0., 0., new_start_node.x[2])]
        new_start_node.was_exp_to_goal = False
        self.upper_limit = np.Inf
        self.nodes = [new_start_node]
        self.current_node = new_start_node
        self.best_candidate = new_start_node

        # reset the solution flag
        self.has_solution = False
        self.solution_node = None
        return



        delete_all = True
        if self.last_returned_candidate is not None:
            solution, _, _, _, _, _ = self.last_returned_candidate
            if np.array_equal(pos, solution[0].x):
                delete_all = False
                print self.name, 'the received node is the same as my own'
                # set end node id correctly

                found = False
                # first, check if the node appears in the solution found
                temp_nodes = []
                for node in solution[::-1]:
                    temp_nodes.append(node)
                    if np.array_equal(pos, node.x):
                        found = True
                        self.current_node = node
                        node.parent = None
                        self.best_candidate = node
                        self.has_solution = False
                        self.solution_node = None
                        self.upper_limit = np.Inf
                        print self.name, 'found the node in the solution'
                        # the node was found in the solution, lets delete all other nodes but the the solution nodes
                        self.nodes = temp_nodes[::-1]

                        # if self.verbose >= 2:
                        # fn = self.nodes[0]
                        # print self.name, 'time of first node', fn.id#,'number of nodes',len(self.nodes)
                        break

                if not found:
                    print self.name, 'did not find the node in the solution'
                    # if it's not in the solution, find it in the node list
                    found = False
                    temp_nodes = []
                    for node in self.nodes[::-1]:
                        temp_nodes.append(node)
                        if np.array_equal(pos, node.x):
                            found = True
                            # print self.name,'id',node.id,toSecs(node.time-self.start_time)
                            node.parent = None
                            self.current_node = node
                            self.received_start_node = node
                            self.best_candidate = node
                            self.has_solution = False
                            self.solution_node = None
                            self.upper_limit = np.Inf
                            self.nodes = temp_nodes[::-1]
                            print self.name, 'found the node in the list of nodes'  # which has',len(self.nodes),'elements'

                            break

                    if not found:
                        print self.name, 'did not find the node in the list of nodes'
                        delete_all = True

        if delete_all or self.last_returned_candidate == None:
            # delete all nodes
            new_start_node = Node(pos, time=time)
            new_start_node.time = time
            # new_start_node.dist = distance2goal(new_start_node.x, self.goal)
            new_start_node.dist = kinodyn_metric(new_start_node.x, self.goal, self.X, self.U)
            new_start_node.cost = 0
            new_start_node.id = self.get_next_id()
            new_start_node.q_params = [(0., 0., new_start_node.x[0]), (0., 0., new_start_node.x[1]),
                                       (0., 0., new_start_node.x[2])]
            new_start_node.was_exp_to_goal = False
            self.upper_limit = np.Inf
            self.nodes = [new_start_node]
            self.current_node = new_start_node
            self.best_candidate = new_start_node

            # reset the solution flag
            self.has_solution = False
            self.solution_node = None
        else:
            # set the time to
            self.current_node.time = time

            # if self.verbose >= 2:
            # fn = self.nodes[0]
            # print self.name, 'time of first node', fn.id#,'number of nodes',len(self.nodes)

            for node in self.nodes:
                if node.parent is not None:
                    node.time = node.parent.time + self.dt
                else:
                    try:
                        node.time = node.children[0].time - self.dt
                    except IndexError:
                        node.time = time

    def extend(self):
        if self.has_solution:
            sleep(0.01)
            # no need to increase the search tree if some solution hase been found
            return

        if self.map == None or self.mask_exp == None or self.map_no_prior_exp == None or \
                        self.map_no_prior_exp_collision_lookup_lb == None or \
                        self.map_no_prior_exp_collision_lookup_ub == None or \
                        self.map_exp_collision_lookup_lb == None or \
                        self.map_exp_collision_lookup_ub == None or \
                        self.mask_exp_collision_lookup_lb == None or \
                        self.mask_exp_collision_lookup_ub == None:
            if not self.notification_missing_maps:
                print "Can't extend tree if map or map_mask is missing or no start"
                self.notification_missing_maps = True
            sleep(0.01)
            return

        if len(self.nodes) == 0:
            print "Can't extend tree when start node is missing"
            sleep(0.01)
            return

        # random free sample x_rnd from X

        if rnd.rand() < self.goal_bias:
            x_rnd = self.goal
            goal_is_chosen = True
        else:
            goal_is_chosen = False
            while True:
                x_rnd = [x1 + (x2 - x1) * rnd.rand() for (x1, x2) in self.X]

                # quickest lookup, if no collision in this case, can skip the polygon stuff
                completely_free = False
                if self.ignorePrior:
                    if not quick_collision_check(statev=x_rnd, map_obj=self.map_no_prior_exp_collision_lookup_ub):
                        completely_free = True
                else:
                    if not quick_collision_check(statev=x_rnd, map_obj=self.map_exp_collision_lookup_ub):
                        completely_free = True

                if not completely_free:
                    # quick lookup, if loopup returns True, the shape is guaranteed to be collided
                    if self.ignorePrior:
                        if quick_collision_check(statev=x_rnd, map_obj=self.map_no_prior_exp_collision_lookup_lb):
                            continue
                    else:
                        if quick_collision_check(statev=x_rnd, map_obj=self.map_exp_collision_lookup_lb):
                            continue
                    # check more detailed, slow
                    polygon = shape2polygon(self.sx, self.sy, x_rnd[:self.n], self.s)

                    if self.ignorePrior:
                        if collision_check_oq(polygon=polygon, map_obj=self.map_no_prior_exp):
                            continue
                    else:
                        if collision_check_oq(polygon=polygon, map_obj=self.map_exp):
                            continue

                # valid sample
                break

        # find closest node x_nearest in the tree
        # firstly, use a quick heuristic to get the k-closest neighbours (test showed that the MRPT is really slow)
        n_nodes = len(self.nodes)
        n_nodes_reduced = int(np.ceil(n_nodes / float(self.k_reduced_ratio)))
        if n_nodes_reduced<n_nodes:
            data = [node.x for node in self.nodes]
            closestNeighboursIdx = np.argsort(cdist([x_rnd], data))[0, :n_nodes_reduced]
            reduced_nodes = [self.nodes[i] for i in closestNeighboursIdx]
        else:
            reduced_nodes = self.nodes

        # secondly, use slow distance metric as proper metric
        shortest_dist = np.Inf
        selected_node = None
        if goal_is_chosen:
            for node in reduced_nodes:
                if node.was_exp_to_goal:
                    continue
                if node.dist <= shortest_dist:
                    selected_node = node
                    shortest_dist = node.dist
            if selected_node == None:
                return
            selected_node.was_exp_to_goal = True
        else:
            for node in reduced_nodes:
                dist = kinodyn_metric(node.x, x_rnd, self.X,
                                      self.U)  # this metric is the min time solution to steer from node in the tree to xr

                # simple euclidian distance function
                # dist = distance2goal(node.x, x_rnd)

                if dist <= shortest_dist:
                    selected_node = node
                    shortest_dist = dist
                    # shortest_node_id = i

        # extend x_nearest towards x_rnd
        while True:
            t1 = selected_node.time
            t2 = t1 + self.dt

            x_new, q_params = saturate_params(selected_node.x, x_rnd, self.X, self.U, 0., toSecs(self.dt))
            x_emg_stopped_node = emergency_brake(x_new, self.U) # to check if state lays in unavoidable collision region

            # simple linear interpolation
            # ptr = x_rnd[0:2] - selected_node.x[0:2]
            # length = np.linalg.norm(ptr)
            # if length > step:
            #    diff = ptr / length * step
            # else:
            #    diff = ptr

            # x_new = np.concatenate((diff + selected_node.x[0:2], selected_node.x[2:]))

            # t = toSecs(selected_node.time)
            # dt = toSecs(self.dt)
            # slope_x = diff[0] / dt
            # offset_x = slope_x*t
            # slope_y = diff[1] / dt
            # offset_y = slope_y*t
            # q_params = [(0., 0, selected_node.x[0]), (0., 0, selected_node.x[1]), (0., 0., selected_node.x[2])]
            # q_params = [(0., slope_x, selected_node.x[0]), (0., slope_y, selected_node.x[1]),
            #            (0., 0., selected_node.x[2])]
            # q_params = [(0., slope_x, selected_node.x[0]-offset_x), (0., slope_y, selected_node.x[1]-offset_y),
            #            (0., 0., selected_node.x[2])]


            # Check if the shape satisfies the state space

            if not check_in_state_space(x_new, self.X):
                # print "outside of X"
                break

            if not check_in_state_space(x_emg_stopped_node, self.X):
                break

            # quickest lookup, guaranteed to be free
            completely_free = False
            if self.ignorePrior:
                if not quick_collision_check(statev=x_new, map_obj=self.map_no_prior_exp_collision_lookup_ub):
                    completely_free = True
            else:
                if not quick_collision_check(statev=x_new, map_obj=self.map_exp_collision_lookup_ub):
                    completely_free = True

            if not completely_free:
                # quick lookup
                if self.ignorePrior:
                    if quick_collision_check(statev=x_new, map_obj=self.map_no_prior_exp_collision_lookup_lb):
                        break
                else:
                    if quick_collision_check(statev=x_new, map_obj=self.map_exp_collision_lookup_lb):
                        break

                # Convert current shape to a polygon.
                polygon = shape2polygon(self.sx, self.sy, x_new[:self.n], self.s)

                ## Check collision with obstacles
                if self.ignorePrior:
                    if collision_check_oq(polygon=polygon, map_obj=self.map_no_prior_exp):
                        # print "crashed"
                        break
                else:
                    if collision_check_oq(polygon=polygon, map_obj=self.map_exp):
                        # print "crashed"
                        break

            ### same for the emergency brake node
            # quickest lookup, guaranteed to be free
            completely_free = False
            if self.ignorePrior:
                if not quick_collision_check(statev=x_emg_stopped_node, map_obj=self.map_no_prior_exp_collision_lookup_ub):
                    completely_free = True
            else:
                if not quick_collision_check(statev=x_emg_stopped_node, map_obj=self.map_exp_collision_lookup_ub):
                    completely_free = True

            if not completely_free:
                # quick lookup
                if self.ignorePrior:
                    if quick_collision_check(statev=x_emg_stopped_node, map_obj=self.map_no_prior_exp_collision_lookup_lb):
                        break
                else:
                    if quick_collision_check(statev=x_emg_stopped_node, map_obj=self.map_exp_collision_lookup_lb):
                        break

                # Convert current shape to a polygon.
                polygon = shape2polygon(self.sx, self.sy, x_emg_stopped_node[:self.n], self.s)

                ## Check collision with obstacles
                if self.ignorePrior:
                    if collision_check_oq(polygon=polygon, map_obj=self.map_no_prior_exp):
                        # print "crashed"
                        break
                else:
                    if collision_check_oq(polygon=polygon, map_obj=self.map_exp):
                        # print "crashed"
                        break

            new_node = Node(x_new)
            new_node.was_exp_to_goal = goal_is_chosen #
            new_node.parent = selected_node
            new_node.cost = selected_node.cost + toSecs(self.dt)

            # new_node.q_params = q_params
            new_node.time = new_node.parent.time + self.dt

            new_node.dist = kinodyn_metric(new_node.x, self.goal, self.X, self.U)
            # print self.name,'dist',round(new_node.dist,2)
            # new_node.dist = distance2goal(new_node.x, self.goal)
            new_node.q_params = q_params
            new_node.id = self.get_next_id()
            if new_node.cost + new_node.dist > self.upper_limit:
                break
            selected_node.children.append(new_node)
            self.nodes.append(new_node)
            # print 'added'

            if self.best_candidate == None or self.best_candidate.dist > new_node.dist:
                self.best_candidate = new_node

            # Goal check
            # dist = distance2goal(new_node.x, self.goal)
            if distance2goal(new_node.x, self.goal) <= self.goal_dist_eucl:
                # this goal solves the problem
                total_cost = new_node.cost+new_node.dist
                if self.solution_node == None or total_cost < (self.solution_node.cost+self.solution_node.dist)*0.99:
                    print self.name, 'found solution with cost', round(total_cost,2)
                    self.upper_limit = total_cost
                    self.prune_tree()
                    self.solution_node = new_node
                self.has_solution = True

                break

            if distance2goal(new_node.x, x_rnd) <= self.goal_dist_eucl:
                # print "arrived at x_rnd"
                break
            # one try only, to keep iteration time short
            #break
            selected_node = new_node

    def prune_tree(self):
        new_nodes = copy(self.nodes)
        something_changed = True
        starting_idx = 0
        while something_changed:
            something_changed = False
            for node in new_nodes[starting_idx:]:
                total_cost = node.cost + node.dist
                if total_cost > self.upper_limit*1.01:
                    # cut the whole branch
                    new_nodes = remove_branch(new_nodes,node)
                    something_changed = True
                    break
                else:
                    starting_idx += 1
        self.nodes = new_nodes





    def draw_nodes(self, solution=False):
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

    def set_current_node_last_returned(self):
        _, _, _, _, second_node, _ = self.last_returned_candidate
        self.current_node = second_node

    def get_path_best_candidate(self):
        # print 'current_node.id',self.current_node.id
        if self.has_solution:
            # return solution
            solution = self.solution_node.get_node_sequence(end_id=self.current_node.id)
            cost = float((len(solution) - 1) * toSecs(self.dt) + solution[-1].dist)
        else:
            # return best candidate
            solution = self.best_candidate.get_node_sequence(end_id=self.current_node.id)
            cost = float((len(solution) - 1) * toSecs(self.dt) + solution[-1].dist * self.cost_heur_factor)
        starting_node = solution[0]

        if len(solution) == 1:
            # in case there are no nodes in the
            next_intermediate_goal = copy(starting_node)
            next_intermediate_goal.x[self.n:] = 0
            next_intermediate_goal.id = self.get_next_id()
            self.best_candidate = next_intermediate_goal
            # adjust the params of the old child
            # for i in range(self.n):
            #    next_intermediate_goal.q_params[i] = (0.,
            #                                          0.,
            #                                          starting_node.x[i])

            next_intermediate_goal.parent = starting_node
            starting_node.children = [next_intermediate_goal]
            next_intermediate_goal.time += self.dt
            next_intermediate_goal.cost = next_intermediate_goal.parent.cost + toSecs(self.dt)
            self.upper_limit = np.Inf

            next_intermediate_goal_params = [0., 0., starting_node.x[0],
                                             0., 0., starting_node.x[1],
                                             0., 0., starting_node.x[2]]

            # cost = float(toSecs(self.dt) + starting_node.dist / self.ideal_velocity * self.cost_heur_factor)
            cost = float(toSecs(self.dt) + starting_node.dist * self.cost_heur_factor)

            solution = [starting_node, next_intermediate_goal]
            self.current_node = starting_node
            self.nodes = [starting_node, next_intermediate_goal]

        else:
            # some sequence is available

            next_intermediate_goal_params = [x for xs in solution[1].q_params for x in xs]
            next_intermediate_goal = solution[1]

            # print toSecs(next_intermediate_goal.time) - toSecs(starting_node.time)

            # Only propose solutions that are explored
            # Check the furthest possible: next_intermediate_goal
            if collision_check_oq(polygon=shape2polygon(self.sx, self.sy, next_intermediate_goal.x[:self.n], self.s),
                                  map_obj=self.mask_exp):
                #print self.name, 'not explored, adding intermediate node to give more time to explore'
                # new node in between necessary
                new_node_inbetween = copy(starting_node)
                new_node_inbetween.x[self.n:] = 0
                new_node_inbetween.id = self.get_next_id()
                new_node_inbetween.parent = starting_node
                new_node_inbetween.children = [next_intermediate_goal]
                new_node_inbetween.time += self.dt
                for i in range(self.n):
                    new_node_inbetween.q_params[i] = (0.,
                                                      0.,
                                                      starting_node.x[i])
                next_intermediate_goal_params = [x for xs in new_node_inbetween.q_params for x in xs]

                # set new parent for the old child
                next_intermediate_goal.parent = new_node_inbetween
                _, q_params = saturate_params(new_node_inbetween.x, next_intermediate_goal.x,
                                              self.X, self.U, 0., toSecs(self.dt))
                next_intermediate_goal.q_params = q_params

                # add new child
                starting_node.children = [new_node_inbetween]

                # shift all times in the children nodes
                shift_all_times_branch(self.nodes, next_intermediate_goal.id, self.dt)

                # insert the new_node_inbetween in the list of nodes
                result, obj, idx = get_from_list(self.nodes,
                                                 lambda x: x.id == next_intermediate_goal.id)

                self.nodes.insert(idx, new_node_inbetween)

                next_intermediate_goal = new_node_inbetween

                cost = cost + toSecs(self.dt) + 10.
            else:
                # check if the next node is actually free
                if collision_check_oq(polygon=shape2polygon(self.sx, self.sy, next_intermediate_goal.x[:self.n],
                                                            self.s), map_obj=self.map_no_prior_exp):
                    print self.name, 'obstacle detected, deleting RRT'

                    # the alternative a staying at the starting node
                    next_intermediate_goal = copy(starting_node)
                    next_intermediate_goal.x[self.n:] = 0
                    next_intermediate_goal.id = self.get_next_id()
                    next_intermediate_goal.parent = starting_node
                    next_intermediate_goal.q_params = [(0., 0., starting_node.x[0]),
                                                       (0., 0., starting_node.x[1]),
                                                       (0., 0., starting_node.x[2])]
                    starting_node.children = [next_intermediate_goal]
                    next_intermediate_goal.time = next_intermediate_goal.parent.time + self.dt
                    next_intermediate_goal.cost = next_intermediate_goal.parent.cost + toSecs(self.dt)

                    next_intermediate_goal_params = [0., 0., starting_node.x[0],
                                                     0., 0., starting_node.x[1],
                                                     0., 0., starting_node.x[2]]

                    self.has_solution = False
                    self.solution_node = None
                    self.upper_limit = np.Inf
                    # if starting_node.dist == None:
                    #    starting_node.dist = distance2goal(starting_node.x, self.goal)
                    # cost = float(toSecs(self.dt) + starting_node.dist / self.ideal_velocity * self.cost_heur_factor)
                    cost = float(toSecs(self.dt) + starting_node.dist * self.cost_heur_factor)
                    solution = [starting_node, next_intermediate_goal]
                    self.nodes = [starting_node, next_intermediate_goal]

        self.last_returned_candidate = (
            solution, cost, next_intermediate_goal_params, starting_node, next_intermediate_goal, self.has_solution)
        return self.last_returned_candidate

    def get_next_id(self):
        self.id_counter += 1
        return self.id_counter

    def increase_time(self, dt):
        for node in self.nodes:
            node.time += dt
