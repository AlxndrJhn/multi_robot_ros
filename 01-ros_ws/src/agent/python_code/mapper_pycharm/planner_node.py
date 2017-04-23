#!/usr/bin/env python
import time

import rospy
from agent.msg import bid
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from config.experiments import experiments
from planning.dimt_rrt import dimt_rrt
from util.ros_map_server import Map
from util.util import toSecs,clamp
#import genpy
import numpy as np
from datetime import datetime

class planner:
    """
    verboseness is incremental
    verbose = 1 means the node says what kind of
    verbose = 2 means verbose = 1 and the node says what kind of bids it sends and receives
    """
    def __init__(self):
        # register node
        self.ID = 0
        self.name = 'planner_{}'.format(self.ID)
        rospy.init_node(self.name)
        self.name = rospy.get_name()
        print self.name,"registered"

        # Parameters from ros call
        self.IS_SIM = rospy.get_param('simulated_robots', True)  # True := Morse Simulator
        self.SIM_FACTOR = rospy.get_param('simulator_factor', 10.)  # True := Morse Simulator
        self.ID = rospy.get_param('~ID', self.ID) # ID of the agent
        self.verbose = rospy.get_param('~verbose', 0)
        self.EXPNUM = rospy.get_param('EXPNUM', 0) # number of the experiment object, see the config/experiments.py file
        r = rospy.Rate(rospy.get_param('~RATE', 50))  # hz
        self.RVIZ = True # publish tree for rviz

        # Load the maps through python file
        self.exp_obj = experiments[self.EXPNUM] # has the two maps inside

        # initial bid
        self.start_node = self.exp_obj.q_init
        self.bid_for_next_interval = bid()
        self.sent_node = self.start_node

        # pre start time
        self.pre_time = self.exp_obj.other_params['pre_start_time']

        # time management
        self.iteration = 0.
        self.iteration_time = rospy.Duration(self.exp_obj.tree_edge_dt)
        self.iteration_planning_time = self.iteration_time*(1.-self.exp_obj.other_params['consensus_percent']*3)

        # Init tree
        self.ignore_the_pior_map = False
        self.rrt_tree = dimt_rrt(exp=self.exp_obj,name=self.name,verbose=self.verbose,ignorePrior=self.ignore_the_pior_map)
        self.rrt_tree.update_start_node(self.start_node, 0.)

        # Syncing topic
        self.run = False
        self.start_time = None

        # Init maps
        self.map = None
        self.map_mask = None
        self.map_np = None
        self.changed_the_margin = False # for the case that the group gets a little bit stuck
        self.my_confidence_in_the_map = 100 # 100 is high, 0 is no confidence, the confidence lowers when there is
                                            # no solution
        self.I_lost_my_confidence = False

        # Logging
        self.logfilename = '/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/03-results/logs/' + \
                           "{:%Y %m %d_%H %M %S_}".format(datetime.now()) + 'plnnr_robot' + str(self.ID) + '.log'
        self.logfile = open(self.logfilename, "w+")
        self.logfile.write("exp %d \r\n" % (self.EXPNUM))
        self.logfile.write("rospy.Time.now() iteration winner_id winner_cost own_cost nodes\r\n")


        # register publisher for fusioned map
        #s = rospy.Service('get_fusioned_map', GetMap, self.handle_map_request)
        self.Pub_bid = rospy.Publisher('~bid', bid, latch=True,queue_size=1)
        #self.Pub_map_without_prior = rospy.Publisher('~map_no_prior', OccupancyGrid, latch=True,queue_size=1)
        self.Pub_mask_expanded = rospy.Publisher('~mask_expanded', OccupancyGrid, latch=True,queue_size=1)
        self.Pub_map_expanded = rospy.Publisher('~map_expanded', OccupancyGrid, latch=True,queue_size=1)


        if self.RVIZ:
            self.Pub_rviz = rospy.Publisher('/vsl_mrkr', Marker, latch=True,queue_size=1)

        # register Subscriber to get positions s.t. the node updates the fusioned map self.map
        #s = rospy.Service('set_position', SetMap, self.handle_pos_message)
        self.Sub_map = rospy.Subscriber('/map_server_{}/map'.format(self.ID),OccupancyGrid, self.handle_map_message)
        #self.Sub_map_diff = rospy.Subscriber('~map_diff', OccupancyGrid, self.handle_map_diff_message)
        self.Sub_map_mask = rospy.Subscriber('/map_server_{}/map_mask'.format(self.ID),OccupancyGrid, self.handle_map_mask_message)
        self.Sub_map_np = rospy.Subscriber('/map_server_{}/map_np'.format(self.ID),OccupancyGrid, self.handle_map_np_message)

        self.Sub_start_node = rospy.Subscriber('/consensus_{}/committed_bid'.format(self.ID),bid, self.handle_start_node_message)
        self.Sub_sync_cmd = rospy.Subscriber('/sync_cmd',String,self.handle_sync_message)

        self.rviz_plot_solution(clear=True)

        if self.pre_time > 0.:
            self.pre_time_routine()
        self.rviz_plot_nodes()
        while not rospy.is_shutdown():
            self.extend_dimt_rrt()
            #r.sleep()

    # callbacks
    def handle_sync_message(self,str):
        str = str.data
        if str=='start':
            self.start_time = rospy.Time.now()
            self.rrt_tree.update_start_time(self.start_time+self.iteration_time)
            self.run = True
        elif str=='pause':
            self.run = False


    def handle_map_message(self, req):
        #print self.name,"received new map"
        self.map = Map(occupancy_grid=req)
        self.rrt_tree.update_map(self.map)


    def handle_map_mask_message(self, req):
        #print self.name, "received new mask"
        self.map_mask = Map(occupancy_grid=req)
        self.rrt_tree.update_mask(self.map_mask)
        if self.rrt_tree.mask_exp is not None:
            self.Pub_mask_expanded.publish(self.rrt_tree.mask_exp.to_message())


    def handle_map_np_message(self,req):
        self.map_np = Map(occupancy_grid=req)
        self.rrt_tree.update_map_np(self.map_np)
        if self.rrt_tree.map_exp is not None:
            self.Pub_map_expanded.publish(self.rrt_tree.map_exp.to_message())

    def handle_start_node_message(self, req):
        if not self.run:
            return
        # check if that was the submitted plan
        #print self.name, "received new start"#,rospy.Time.now().secs,rospy.Time.now().nsecs
        if self.verbose >= 2:
            print self.name,'received',round(toSecs(req.start_t-self.start_time)),\
                round(toSecs(req.end_t-self.start_time)),'pos',[round(num,2) for num in req.start_q], \
                'to',[round(num, 2) for num in req.end_q]
            print  self.name, 'received', [round(x,4) for x in req.params]
        #self.start_plan = req
        #self.start_node = req.end_q

        #print self.name, 'received', (req.start_t - self.start_time) / 1e9, (req.end_t - self.start_time) / 1e9

        # req.
        # cost = cost,
        # params = params,
        # robot_id = self.ID,
        # start_t = first_node.time,
        # start_q = first_node.x,
        # end_t = second_node.time,
        # end_q = second_node.x,
        # add_info = str(flag_solution)

        self.logfile.write("%f %d %d %1.4f %1.4f %d\r\n" % (toSecs(rospy.Time.now()),self.iteration, req.robot_id,
                                                            req.cost, self.last_submitted_cost,len(self.rrt_tree.nodes)))
        if req.robot_id == self.ID:
            print self.name,'won consensus of iteration #',self.iteration,'with cost',self.last_submitted_cost
            self.rrt_tree.set_current_node_last_returned()
            self.rviz_plot_solution(clear=False)
        else:
            self.rrt_tree.update_start_node(pos=req.end_q, time=req.end_t)
            self.rviz_plot_solution(clear=True)
        self.start_plan = req
        self.start_node = req.end_q

    def pre_time_routine(self):
        print self.name, "waiting for maps to start preparing", self.pre_time, "secs"
        while self.map == None or self.map_mask == None or self.map_np == None:
            time.sleep(1)
        print self.name, "starting pre time planning for", self.pre_time, "secs"
        start_time = rospy.get_time()
        cnter = 20
        while rospy.get_time()<start_time+self.pre_time:
            if rospy.is_shutdown():
                exit(0)
            self.rrt_tree.extend()
            if cnter==0:
                cnter = 20
                self.rviz_plot_nodes()
            cnter -= 1
        print self.name, "pre time finished with",len(self.rrt_tree.nodes),"nodes"

    def extend_dimt_rrt(self):
        if not self.run:
            time.sleep(0.010)
            return
        if self.map == None or self.map_mask == None or self.map_np == None:
            print self.name, "can't extend without maps"
            return
        #print self.name, "extending tree"
        # TODO: if the agent knows its plan was accepted, run an optimization algorithm instead of RRT

        # extend for the time given
        #print self.name, 'number of nodes before', len(self.rrt_tree.nodes)
        while rospy.Time.now()<self.start_time+self.iteration_time*self.iteration+self.iteration_planning_time:
            self.rrt_tree.extend()
        self.iteration += 1

        #print self.name, 'number of nodes after', len(self.rrt_tree.nodes)

        # make a bid from best candidate OR solution
        solution, cost, params, first_node, second_node, flag_solution = self.rrt_tree.get_path_best_candidate()
        #print self.name, 'number of nodes after getting best path', len(self.rrt_tree.nodes)
        #print 'bid',rospy.Time.now().secs,first_node.time.secs,second_node.time.secs
        self.last_submitted_solution = solution
        self.last_submitted_cost = cost

        # the maps are about confidence, it deteriorates as the solution is missing, and builds up
        if not flag_solution:
            self.my_confidence_in_the_map -= 1
        else:
            self.my_confidence_in_the_map += 1
        self.my_confidence_in_the_map = clamp(self.my_confidence_in_the_map,0,100)

        if len(solution)<=2 and not self.changed_the_margin:
            print '\t',self.name,'seems like the group is stuck, make the safety marging a little bit smaller'
            self.rrt_tree.change_map_margin(self.rrt_tree.expand_factor_map*0.85)
            self.changed_the_margin = True
        else:
            if self.changed_the_margin:
                print '\t',self.name, 'seems like the group is stuck, make the safety marging a little bit smaller'
                self.changed_the_margin = False
                self.rrt_tree.change_map_margin() #default

        # print 'time',rospy.Time.now().secs-first_node.time.secs
        if not self.ignore_the_pior_map:
            if self.my_confidence_in_the_map <= 40 and not self.I_lost_my_confidence:
                print '\t',self.name,'the prior map is bad!'
                self.I_lost_my_confidence = True
                self.rrt_tree.ignorePrior = True
            if self.my_confidence_in_the_map >=90 and self.I_lost_my_confidence:
                print '\t',self.name,'I\'m ok with the prior map now!'
                self.I_lost_my_confidence = False
                self.rrt_tree.ignorePrior = False

        #if self.RVIZ:
        #    # plot the solution in a special color
        #    for node in solution:


        Bid = bid(cost=cost,
                  params=params,
                  robot_id=self.ID,
                  start_t=self.start_time+self.iteration_time*self.iteration,#first_node.time,
                  start_q=first_node.x,
                  end_t=self.start_time+self.iteration_time*(self.iteration+1),#second_node.time,
                  end_q=second_node.x,
                  add_info=str(flag_solution))
        if self.verbose >=2:
            print self.name, 'sending time',round(toSecs(first_node.time-self.start_time)), \
                round(toSecs(second_node.time-self.start_time)),'pos',[round(num,2) for num in first_node.x],'to',[round(num, 2) for num in second_node.x]
            fn = self.rrt_tree.nodes[0]
            print self.name, 'at concensus, first node id', fn.id,'time', round(toSecs(fn.time - self.start_time)),\
                'number of nodes',len(self.rrt_tree.nodes)
            print  self.name, 'sending', [round(x, 4) for x in params]

        #print self.name,'sent',(first_node.time- self.start_time)/1e9,(second_node.time- self.start_time)/1e9
        #print self.name,first_node.x,second_node.x
        #print self.name,params

        self.sent_node = second_node

        if self.run:
            try:
                self.Pub_bid.publish(Bid)
            except rospy.exceptions.ROSSerializationException:
                print self.name,'something is wrong with the nodes'
                self.rrt_tree.update_start_node(pos=self.start_node, time=self.start_plan.end_t)
                pass
            if self.verbose >= 1:
                if flag_solution:
                    print self.name, "published solution with cost:", round(cost,2),'id',second_node.id#,rospy.Time.now().secs,rospy.Time.now().nsecs
                else:
                    print self.name, "published candidate with dist:", round(cost,2),'id',second_node.id#,rospy.Time.now().secs,rospy.Time.now().nsecs

            self.rviz_plot_nodes()
            time.sleep(toSecs(self.start_time + self.iteration_time*self.iteration - rospy.Time.now()))

    def rviz_plot_nodes(self):
        if self.RVIZ:
            # draw all the nodes from the tree
            points = Marker()
            points.ns = self.name
            points.id = 0
            points.header.frame_id = "map"  # odom TODO map?
            points.header.stamp = rospy.Time.now()
            points.type = Marker.POINTS

            points.scale.x = 0.01
            points.scale.y = 0.01
            points.scale.z = 1.

            points.pose.position.x = 0
            points.pose.position.y = 0
            points.pose.position.z = 0.1

            points.pose.orientation.x = 0
            points.pose.orientation.y = 0
            points.pose.orientation.z = 0
            points.pose.orientation.w = 1

            points.color.a = 1.0
            points.color.r, points.color.g, points.color.b = self.exp_obj.colors_robots[self.ID]

            for node in self.rrt_tree.nodes:
                point = Point()
                point.x = node.x[0]
                point.y = node.x[1]
                point.z = 0

                points.points.append(point)
            self.Pub_rviz.publish(points)

    def rviz_plot_solution(self,clear=False):
        if self.RVIZ:

            # prepare empty message
            points = Marker()
            points.ns = self.name
            points.id = 1
            points.header.frame_id = "map"  # odom TODO map?
            points.header.stamp = rospy.Time.now()
            points.type = Marker.POINTS

            if not clear and self.last_submitted_solution is not None:
                # draw all the nodes from the solution
                points.scale.x = 0.015
                points.scale.y = 0.015
                points.scale.z = 1.

                points.pose.position.x = 0
                points.pose.position.y = 0
                points.pose.position.z = 0.15

                points.pose.orientation.x = 0
                points.pose.orientation.y = 0
                points.pose.orientation.z = 0
                points.pose.orientation.w = 1

                points.color.a = 1.0
                points.color.r, points.color.g, points.color.b = self.exp_obj.colors_robots[self.ID]

                for node in self.last_submitted_solution:
                    point = Point()
                    point.x = node.x[0]
                    point.y = node.x[1]
                    point.z = 0

                    points.points.append(point)
            self.Pub_rviz.publish(points)



if __name__ == "__main__":
    try:
        planner = planner()
    except rospy.ROSInterruptException:
        pass