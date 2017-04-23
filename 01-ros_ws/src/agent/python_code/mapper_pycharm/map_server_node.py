#!/usr/bin/env python
import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String,Float32MultiArray

from config.experiments import experiments
from util.ros_map_server import Map
from agent.msg import bid
import tf
from util.real_robots_exp import map_trans_orient_to_PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers


class map_server:
    def __init__(self):
        # register node
        self.ID = 0
        self.name = 'map_server_{}'.format(self.ID)
        rospy.init_node(self.name)
        self.name = rospy.get_name()
        print self.name,"registered"

        # Parameters from ros call
        self.IS_SIM = rospy.get_param('simulated_robots', True)  # True := Morse Simulator
        self.SIM_FACTOR = rospy.get_param('simulator_factor', 10.)  # True := Morse Simulator
        self.ID = rospy.get_param('~ID', self.ID) # ID of the agent
        self.EXPNUM = rospy.get_param('EXPNUM', 0) # number of the experiment object, see the config/experiments.py file
        self.epucknum = rospy.get_param('~epucknum', 0)

        # Thresholding
        self.threshold = 0.01 # in meter, this offset is needed at least before map is adjusted and rebroadcasted
        self.old_pos = Point()

        # using the shape to expand the mask
        self.adjustment_of_shape_size = 0.5 #times the robot-size, <0 means not the whole area covered by the shape is
                                           #assumed to be discovered, 0 means just the area is assumed to be free of
                                           #obstacles,>0 means a bigger area than the shape itself is assumed to be
                                           #free of obstacles

        # Syncing topic
        self.run = False

        # Load the maps through python file
        self.exp_obj = experiments[self.EXPNUM] # has the two maps inside

        # generate occupancy grids
        self.map_init = Map(origin_x=0., origin_y=0., resolution=.01,
                        width=self.exp_obj.X[0][1], height=self.exp_obj.X[1][1],basis=self.exp_obj.map_init)
        self.map_real = Map(origin_x=0., origin_y=0., resolution=.01,
                        width=self.exp_obj.X[0][1], height=self.exp_obj.X[1][1],basis=self.exp_obj.map_real)
        self.map_mask = Map(origin_x=0., origin_y=0., resolution=.01,
                        width=self.exp_obj.X[0][1], height=self.exp_obj.X[1][1],all_ones=True)
        self.map_output = Map(origin_x=0., origin_y=0., resolution=.01,
                        width=self.exp_obj.X[0][1], height=self.exp_obj.X[1][1])
        self.map_np = Map(origin_x=0., origin_y=0., resolution=.01,
                        width=self.exp_obj.X[0][1], height=self.exp_obj.X[1][1])

        #print self.name,'

        # register publisher for fusioned map
        #s = rospy.Service('get_fusioned_map', GetMap, self.handle_map_request)
        self.Pub = rospy.Publisher('~map', OccupancyGrid,latch=True,queue_size=1)
        self.Pub.publish(self.map_init.to_message())
        print self.name, "published initial map"

        self.Pub_mask = rospy.Publisher('~map_mask', OccupancyGrid,latch=True,queue_size=1)
        self.Pub_mask.publish(self.map_mask.to_message())
        print self.name, "published initial mask"

        self.Pub_np = rospy.Publisher('~map_np', OccupancyGrid,latch=True,queue_size=1)
        self.Pub_np.publish(self.map_np.to_message())
        print self.name, "published initial map without the prior"

        self.Pub_tf = tf.TransformBroadcaster()

        # register Subscriber to get positions s.t. the node updates the fusioned map self.map
        #s = rospy.Service('set_position', SetMap, self.handle_pos_message)
        if self.IS_SIM:
            self.Sub = rospy.Subscriber('/robot_{}/PoseStamped'.format(self.ID),PoseStamped,
                                        self.handle_pos_message,queue_size=1)
        else:
            self.Sub_tf = tf.TransformListener()
            self.Sub_pos = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,self.handle_all_pose_messages)

        self.Sub_state = rospy.Subscriber('/controller_{}/State'.format(self.ID), Float32MultiArray,
                                          self.handle_state_message, queue_size=1)
        self.Sub_committed_bid = rospy.Subscriber('/consensus_{}/committed_bid'.format(self.ID),bid,
                                                  self.handle_committed_node_message)

        self.Sub_sync_cmd = rospy.Subscriber('/sync_cmd',String,self.handle_sync_message)


        # send frequent tf messages
        self.timer_tf = rospy.Timer(rospy.Duration(0.08),self.send_tf, oneshot=False)

        rospy.spin()

    # callbacks
    def handle_sync_message(self,str):
        str = str.data

        if str=='start':
            self.run = True
        elif str=='pause':
            self.run = False

    def handle_committed_node_message(self,req):
        if not self.run:
            return
        self.update_mask_from_state(req.end_q)
        # TODO: intermediate states are missing

    def handle_state_message(self, state):
        self.update_mask_from_state(state.data)

    def update_mask_from_state(self,state_vec):
        if self.exp_obj.shape_name=='ellipse_shape':
            x,y,a,xdot,ydot,adot = state_vec
            A = self.exp_obj.shape_params['area']
            b = A / (np.pi * a)

            # discretization for the gridmap
            # the shape is expanded by self.adjustment_of_shape_size times the exp_obj.robot_size parameter, assuming
            # the robots are with their center point on the perimeter
            a_range = int(math.floor((a+self.exp_obj.robot_size*(self.adjustment_of_shape_size)) / self.map_output.resolution))
            x_center = int(round(x / self.map_output.resolution))
            x_max = self.exp_obj.w
            y_center = int(round(y / self.map_output.resolution))
            y_max = self.exp_obj.h
            for x_offset in range(-a_range, a_range):
                x = x_center + x_offset
                if x<0 or x>x_max:
                    continue

                y_range = int(math.floor(np.sqrt(1-(x_offset * self.map_output.resolution/(a+self.exp_obj.robot_size*(self.adjustment_of_shape_size)))**2)*
                                    (b+self.exp_obj.robot_size*(self.adjustment_of_shape_size)) / self.map_output.resolution))
                for y_offset in range(-y_range,y_range):
                    y = y_center + y_offset
                    if y<0 or y>y_max:
                        continue
                    self.map_mask.set_cell(x, y, 0.) # 'discover' a cell
        time.sleep(0.5)



    def point_to_point(self,a,b):
        a = np.array((a.x,a.y))
        b = np.array((b.x,b.y))
        return np.linalg.norm(b-a)

    def handle_all_pose_messages(self, AlvarMsgs):
        for marker in AlvarMsgs.markers:
            # to output all current positions
            #try:
            #    (trans, orient) = self.Sub_tf.lookupTransform('/map', '/ar_marker_{}'.format(marker.id), rospy.Time(0))
            #    posestamped = map_trans_orient_to_PoseStamped(trans, orient)
            #    print 'id',marker.id,'x',posestamped.pose.position.x,'y',posestamped.pose.position.y
            #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #    continue

            if marker.id == self.epucknum:
                try:
                    (trans, orient) = self.Sub_tf.lookupTransform('/map', '/ar_marker_{}'.format(self.epucknum), rospy.Time(0))
                    self.handle_pos_message(map_trans_orient_to_PoseStamped(trans, orient))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

    def handle_pos_message(self,req):
        if not self.run:
            return
        if math.isnan(req.pose.position.x):
            return
        new_pos = req.pose.position
        if self.IS_SIM:
            new_pos.x /= self.SIM_FACTOR
            new_pos.y /= self.SIM_FACTOR
        if self.point_to_point(new_pos,self.old_pos)<self.threshold:
            return
        self.old_pos = new_pos
        #print self.name,"received pose:\n", new_pos, "\n",self.name,"publishing new map"
        # adjust the mask self.map_mask.grid, 1.0 where the robot "sensed", depends on shape
        sense_shape = self.exp_obj.other_params['robot_sense_type']
        sense_range = self.exp_obj.other_params['robot_sense_range']
        if sense_shape=='circle':
            # for x_offset in range(-x_range,x_range):
            #     x = int(round(new_pos.x / self.map_output.resolution))+x_offset
            #     if x<0:
            #         continue
            #     y_range = np.sqrt(sense_range ** 2 - (x_offset * self.map_output.resolution) ** 2)
            #     for y in range(int(round((new_pos.y-y_range)/self.map_output.resolution)),
            #                    int(round((new_pos.y+y_range)/self.map_output.resolution))):
            #         if y < 0:
            #             continue
            #         self.map_mask.set_cell(x, y, 0.)


            x_range = int(math.floor(sense_range/ self.map_output.resolution))
            x_center = int(round(new_pos.x / self.map_output.resolution))
            x_max = self.exp_obj.w
            y_center = int(round(new_pos.y / self.map_output.resolution))
            y_max = self.exp_obj.h
            for x_offset in range(-x_range, x_range):
                x = x_center + x_offset
                if x<0 or x>x_max:
                    continue

                y_range = int(math.floor(np.sqrt(round(sense_range ** 2 - (x_offset * self.map_output.resolution) ** 2,3))
                                    / self.map_output.resolution))
                for y_offset in range(-y_range,y_range):
                    y = y_center + y_offset
                    if y<0 or y>y_max:
                        continue
                    self.map_mask.set_cell(x, y, 0.) # 'discover' a cell


        self.combine_maps(self.map_init, self.map_real, self.map_mask, self.map_output)
        self.Pub.publish(self.map_output.to_message())
        self.Pub_mask.publish(self.map_mask.to_message())

        self.map_np.grid = np.multiply(self.map_real.grid, (1 - self.map_mask.grid))
        self.Pub_np.publish(self.map_np.to_message())

        time.sleep(0.5)

    def send_tf(self,event):
        self.Pub_tf.sendTransform((0, 0, 0),
                                  tf.transformations.quaternion_from_euler(0, 0, 0),
                                  rospy.Time.now(),
                                  "map",
                                  "world")

        # publishing the tf for the camera frame
        self.Pub_tf.sendTransform((self.exp_obj.X[0][1]/2, self.exp_obj.X[1][1]/2, 3.48),
                                  tf.transformations.quaternion_from_euler(np.pi+0.07, -0.07, 0),#tf.transformations.quaternion_from_euler(np.pi+0.1, -0.1, 0),
                                  rospy.Time.now(),
                                  "usb_cam_frame",
                                  "world")
        #print 'sent tf'

    def combine_maps(self,map_init,map_real,map_mask,map_output):
        map_output.grid = np.multiply(map_init.grid,map_mask.grid)+ \
                          np.multiply(map_real.grid, (1-map_mask.grid))



if __name__ == "__main__":
    try:
        map_server = map_server()
    except rospy.ROSInterruptException:
        pass