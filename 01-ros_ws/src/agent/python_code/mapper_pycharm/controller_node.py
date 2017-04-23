#!/usr/bin/env python
import math
import time

import numpy as np
import rospy
from agent.msg import bid
from geometry_msgs.msg import Twist, PoseStamped, Pose,Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String,Float32MultiArray,MultiArrayDimension

from tf.transformations import euler_from_quaternion,quaternion_from_euler
import tf
from visualization_msgs.msg import Marker
from copy import copy

from config.experiments import experiments
from util.util import Convert2Polar
from util.util import toSecsNsecs,toSecs
from util.real_robots_exp import adjust_pose,map_trans_orient_to_PoseStamped

from util.ros_planner_stuff import  distance2goal
import logging
from datetime import datetime

class controller_node:
    def __init__(self):
        # register node
        self.ID = 0
        self.name = 'controller_{}'.format(self.ID)
        rospy.init_node(self.name)
        self.name = rospy.get_name()
        print self.name, "registered"

        # Local Parameters from ros call
        self.ID = rospy.get_param('~ID',self.ID)  # ID of the agent
        self.epucknum = rospy.get_param('~epucknum', self.ID)  # ID of the agent



        # Global Parameters from ros call
        self.EXPNUM = rospy.get_param('EXPNUM',
                                      0)  # number of the experiment object, see the config/experiments.py file
        self.IS_SIM = rospy.get_param('simulated_robots', True)  # True := Morse Simulator
        self.SIM_FACTOR = rospy.get_param('simulator_factor', 10.)  # True := Morse Simulator
        self.verbose = rospy.get_param('~verbose', 0)

        # Logging
        self.logfilename = '/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/03-results/logs/'+\
                       "{:%Y %m %d_%H %M %S_}".format(datetime.now())+'cntrlr_robot'+str(self.ID)+'.log'
        self.logfile = open(self.logfilename, "w+")
        self.logfile.write("exp %d \r\n" % (self.EXPNUM))
        self.logfile.write("rospy.Time.now() (ref_rho - rho) phi_error v w\r\n")
        #
        #self.logfile.close()
        #exit(0)
        # Initialization
        self.init_pos = rospy.get_param('init_pos', False)
        self.init_tolerance = 0.02 #4cm is ok
        #self.init_factor_p = 1
        self.init_rate = rospy.Rate(5)  # 5Hz


        self.RVIZ = True

        # Load the maps through python file
        self.exp_obj = experiments[self.EXPNUM]  # has the two maps inside


        # initial bid
        self.committed_bid = bid()

        # flag to toggle between 'publishing' and 'just accepting' the consensus process
        self.is_publishing = True

        # Neighbors
        self.NeighborLeft = (self.ID - 1) % self.exp_obj.n_robots
        self.NeighborRight = (self.ID + 1) % self.exp_obj.n_robots
        if not self.IS_SIM:
            self.epuckNeighborLeft = rospy.get_param('~leftepucknum', self.NeighborLeft)
            self.epuckNeighborRight = rospy.get_param('~rightepucknum', self.NeighborRight)
            print self.name, "left epuck:", self.epuckNeighborLeft, "right epuck:", self.epuckNeighborRight

        print self.name,"left:", self.NeighborLeft, "right:", self.NeighborRight

        # controller parameters
        self.dt_fine = self.exp_obj.controller_params['dt_fine']
        self.kp = self.exp_obj.controller_params['kp']
        self.kphi = self.exp_obj.controller_params['kphi']
        self.omeg = self.exp_obj.controller_params['Omeg']
        self.current_plan = bid()
        self.current_plan.params =self.exp_obj.params_as_array()
        self.current_plan.start_q = self.exp_obj.q_init
        self.interval = self.exp_obj.tree_edge_dt
        self.n_robots = self.exp_obj.n_robots
        self.feedback_d = self.exp_obj.controller_params['feedback_d']
        print self.name,'omeg',self.omeg,'kp',self.kp,'kphi',self.kphi

        # variables
        self.yaw = 0
        self.own_pose = None
        self.left_pose = None
        self.right_pose = None
        self.next_plan = None
        self.cnter = 0 # just for occasional outputs


        # limits
        self.X = self.exp_obj.X


        # Syncing topic
        self.run = False

        # Flags for notifications to appear only once
        self.notification_no_pos = True

        # register publisher Twist message
        if self.IS_SIM:
            self.Pub_twist = rospy.Publisher('/robot_{}/Twist'.format(self.ID), Twist, latch=True, queue_size=1)
        else:
            self.Pub_twist = rospy.Publisher('/epuck{}/cmd_vel'.format(self.ID), Twist, latch=True, queue_size=1)

        if self.RVIZ:
            self.Pub_rviz = rospy.Publisher('/vsl_mrkr', Marker, latch=True,queue_size=1)

        self.Pub_state = rospy.Publisher('/controller_{}/State'.format(self.ID), Float32MultiArray, latch=True, queue_size=1)
        self.Pub_cmd = rospy.Publisher('/sync_cmd', String, latch=True, queue_size=1)

        # register Subscribers to get poses
        if self.IS_SIM:
            self.Sub_pos_left = rospy.Subscriber('/robot_{}/PoseStamped'.format(self.NeighborLeft), PoseStamped,
                                                 self.handle_left_pose_message)
            self.Sub_pos_right = rospy.Subscriber('/robot_{}/PoseStamped'.format(self.NeighborRight), PoseStamped,
                                                  self.handle_right_pose_message)
            self.Sub_pos_self = rospy.Subscriber('/robot_{}/PoseStamped'.format(self.ID), PoseStamped,
                                                 self.handle_own_pose_message)
        else:
            self.Sub_tf = tf.TransformListener()
            self.Sub_pos = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,self.handle_all_pose_messages)

        # register Subscriber to get the committed bid
        self.Sub_next_plan = rospy.Subscriber('/consensus_{}/committed_bid'.format(self.ID), bid,self.handle_bid_message)
        self.Sub_sync_cmd = rospy.Subscriber('/sync_cmd',String,self.handle_sync_message)

        self.timer_plan = None
        self.timer_control = None
        self.timer_goal = None

        self.Goal_reached = False

        self.draw_shape(0.001, 0.001, 0.001, 0.001)

        while self.are_prerquisites_incomplete():
            if rospy.is_shutdown():
                exit(0)
            time.sleep(0.01)
        # show first cofiguration

        if self.init_pos:
            print self.name,'starting initialization'
            # goal for this robot
            goal = np.array([self.exp_obj.controller_params['init x'][self.ID],
                             self.exp_obj.controller_params['init y'][self.ID]])



            # check if error is small
            flag_msg_once = True
            sparse_msgs = 10
            counter = sparse_msgs
            while True:
                # current position and orientation
                pos = np.array([self.own_pose.position.x, self.own_pose.position.y])
                (_, _, phi) = euler_from_quaternion([self.own_pose.orientation.x,
                                                          self.own_pose.orientation.y,
                                                          self.own_pose.orientation.z,
                                                          self.own_pose.orientation.w])

                error_pos = goal - pos

                if rospy.is_shutdown():
                    exit(0)

                if self.run:
                    break

                #check
                dist = np.linalg.norm(error_pos)
                if dist < self.init_tolerance:
                    if flag_msg_once:
                        print self.name,'arrived at initial point'
                        flag_msg_once = False
                    self.Pub_twist.publish(Twist())
                    self.init_rate.sleep()
                    continue
                else:
                    if not flag_msg_once:
                        print self.name, 'left initial, returning'
                        flag_msg_once = True



                # control
                ref_phi = np.arctan2(error_pos[1],error_pos[0])
                error_phi = np.arctan2(np.sin(ref_phi-phi), np.cos(ref_phi-phi))

                twist = Twist()
                twist.angular.z = error_phi*0.5
                twist.linear.x = np.linalg.norm(error_pos)*20*(1-np.abs(error_phi/np.pi))
                self.Pub_twist.publish(twist)

                counter -= 1
                if counter==0:
                    counter = sparse_msgs
                    if twist.angular.z>0:
                        print self.name, "dist %0.3fm, angle diff %04ddeg, speed %.3f, turnrate  %.3f, AR %d" % \
                                         (round(dist, 3), np.round(error_phi / np.pi * 180), round(twist.linear.x, 3),
                                          round(twist.angular.z, 3), self.epucknum)
                    else:
                        print self.name, "dist %0.3fm, angle diff %04ddeg, speed %.3f, turnrate %.3f, AR %d" % \
                                         (round(dist, 3),np.round(error_phi / np.pi * 180),round(twist.linear.x,3),
                                          round(twist.angular.z,3),self.epucknum)


                self.init_rate.sleep()
        print self.name,'initialization finished'
        rospy.spin()


    # callbacks
    def handle_sync_message(self, str):
        str = str.data
        if str == 'start':
            self.current_plan.start_t = rospy.Time.now()
            self.notification_no_pos = True
            if self.timer_plan is not None:
                self.timer_plan.shutdown()
                #self.timer_control.shutdown()
                self.timer_goal.shutdown()
            self.timer_plan = rospy.Timer(rospy.Duration(*toSecsNsecs(self.interval)), self.trigger_plan)
            #self.timer_control = rospy.Timer(rospy.Duration(*toSecsNsecs(self.dt_fine)), self.trigger_control)
            self.timer_goal = rospy.Timer(rospy.Duration(1), self.check_goal_condition)
            self.run = True
        elif str == 'pause':
            self.run = False
            print self.name,'paused'
            if self.timer_plan is not None:
                self.timer_plan.shutdown()
                #self.timer_control.shutdown()
                self.timer_goal.shutdown()
                self.timer_plan = None
                #self.timer_control = None
                self.timer_goal = None
            self.Pub_twist.publish(Twist())

    def check_goal_condition(self,event):
        dist = distance2goal(self.current_plan.start_q, self.exp_obj.q_final)
        #if self.ID == 0:
        #    print dist
        if dist<self.exp_obj.goal_distance:
            self.stop_experiment()
            print self.name, 'GOAL REACHED'
            #self.timer_plan.shutdown()
            #self.timer_control.shutdown()

    def stop_experiment(self):
        self.Pub_cmd.publish("pause")


    def handle_bid_message(self,req):
        self.next_plan = req
        #print self.name, "received a committed node"

    def handle_left_pose_message(self, stampedpose):
        #print self.name, "received left neighbor pose"
        if self.IS_SIM:
            self.left_pose = stampedpose.pose
            self.left_pose.position.x /= self.SIM_FACTOR
            self.left_pose.position.y /= self.SIM_FACTOR
        else:
            self.left_pose = adjust_pose(stampedpose.pose)

    def handle_right_pose_message(self, stampedpose):
        #print self.name, "received right neighbor pose"
        if self.IS_SIM:
            self.right_pose = stampedpose.pose
            self.right_pose.position.x /= self.SIM_FACTOR
            self.right_pose.position.y /= self.SIM_FACTOR
        else:
            self.right_pose = adjust_pose(stampedpose.pose)

    def handle_own_pose_message(self, stampedpose):
        #print self.name, "received own pose"
        if self.IS_SIM:
            self.own_pose = stampedpose.pose
            self.own_pose.position.x /= self.SIM_FACTOR
            self.own_pose.position.y /= self.SIM_FACTOR
        else:
            self.own_pose = adjust_pose(stampedpose.pose)

        (_, _, self.yaw) = euler_from_quaternion([  self.own_pose.orientation.x,
                                                    self.own_pose.orientation.y,
                                                    self.own_pose.orientation.z,
                                                    self.own_pose.orientation.w])
        self.draw_robot()
        self.trigger_control(event=None)

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
                    self.handle_own_pose_message(map_trans_orient_to_PoseStamped(trans, orient))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            elif marker.id == self.epuckNeighborRight:
                try:
                    (trans, orient) = self.Sub_tf.lookupTransform('/map', '/ar_marker_{}'.format(self.epuckNeighborRight), rospy.Time(0))
                    self.handle_right_pose_message(map_trans_orient_to_PoseStamped(trans, orient))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            elif marker.id == self.epuckNeighborLeft:
                try:
                    (trans, orient) = self.Sub_tf.lookupTransform('/map', '/ar_marker_{}'.format(self.epuckNeighborLeft), rospy.Time(0))
                    self.handle_left_pose_message(map_trans_orient_to_PoseStamped(trans, orient))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

    def are_prerquisites_incomplete(self):
        if (self.left_pose is not None or self.right_pose is not None) and self.own_pose is None:
            self.just_go_back()
        return self.left_pose == None or self.right_pose == None or self.own_pose == None or \
                math.isnan(self.own_pose.position.x) or self.current_plan == None

    def trigger_control(self, event):
        """
        The actual controller routine, uses local information to generate the control variable of type Twist
        """
        #first = rospy.get_rostime()
        if not self.run:
            return

        if self.are_prerquisites_incomplete():
            # missing some information to calculate the the control variable
            if self.notification_no_pos:
                self.notification_no_pos = False
                print self.name,'no positions received, if the simulation/localization running?'
                self.stop_experiment()

            self.Pub_twist.publish(Twist())
            return

        self.notification_no_pos = True # reset the notification, in case something stops working
        # get the analytical value of the state vector q at time ti for the shape
        ti = toSecs(rospy.Time.now()-self.current_plan.start_t)
        coefficients = self.current_plan.params
        #if self.ID==0:
        #    print self.name,ti
        coefficients = [(coefficients[0],coefficients[1],coefficients[2]),
                        (coefficients[3], coefficients[4], coefficients[5]),
                        (coefficients[6], coefficients[7], coefficients[8])]
        q = np.array([aq * (ti ** 2) / 2. + bq * ti + cq for aq, bq, cq in coefficients])  # q = [x y a]
        q_dot = np.array([aq * ti + bq for aq, bq, cq in coefficients])

        # saturate the variables... bad way to fix issues, the planner module should deal with it
        for i in range(self.exp_obj.n_dim/2):
            q[i] = max(q[i],self.X[i][0])
            q[i] = min(q[i], self.X[i][1])

        x_shape, y_shape, a = q
        x_dot, y_dot, a_dot = q_dot
        #if self.ID==0:
        #    print q,self.current_plan.start_q

        # Publish the current state of the shape to update the explored area (the covered area is always assumed to be
        # free of obstacles)
        state = Float32MultiArray()
        state.layout.dim.append(MultiArrayDimension())
        state.layout.dim[0].label = "state"
        state.layout.dim[0].size = len(self.exp_obj.q_init)
        state.data = np.append(q,q_dot)
        self.Pub_state.publish(state)


        rho, phi = Convert2Polar(self.own_pose.position.x, self.own_pose.position.y, x_shape, y_shape)

        # Calculate the reference distance for each agent (depends on the shape parameters a and b and the current angle phi
        A = self.exp_obj.shape_params['area']
        b = A / (np.pi * a)
        ref_rho = a * b / (np.sqrt(b ** 2 * np.cos(phi) ** 2 + a ** 2 * np.sin(phi) ** 2))  # reference rho for each

        self.draw_shape(x=x_shape,y=y_shape,a=a,b=b)

        # Calculate the reference phi, average between its neighbors and an additional rotation term
        # phi[i+1] left
        # phi[i-1] right


        #if self.ID==0:
        #    print self.name,'yaw',round(self_yaw/np.pi*180)



        #print self_yaw
        _,left_phi = Convert2Polar(self.left_pose.position.x, self.left_pose.position.y, x_shape, y_shape)
        _,right_phi = Convert2Polar(self.right_pose.position.x, self.right_pose.position.y, x_shape, y_shape)
        angle_diff = np.arctan2(np.sin(left_phi-right_phi), np.cos(left_phi-right_phi))
        if angle_diff<0:
            angle_diff+=2*np.pi
        phi_av = angle_diff / 2. + right_phi
        if phi_av<-np.pi:
            phi_av+=2*np.pi
        if phi_av>np.pi:
            phi_av-=2*np.pi
        phi_error = np.arctan2(np.sin(phi_av - phi), np.cos(phi_av - phi))
        #if self.ID==0:
        #    print self.name,'phi ref',round(phi_av/np.pi*180),'phi error',round(phi_error/np.pi*180),'phi diff',round(angle_diff/np.pi*180)

        dot_phi = self.omeg + self.kphi * phi_error
        if self.verbose>=2:
            print self.name,'left phi',left_phi,'phi',phi,'right phi',right_phi,'angle diff',angle_diff,'phi_av',phi_av,'phi_error',phi_error

        partial_derivative_a = (-(A * (2 * a * np.sin(phi) ** 2 - 2 * A ** 2 * np.cos(phi) ** 2 / (np.pi ** 2 * a ** 3))) /
        (2 * np.pi * (A ** 2 * np.cos(phi) ** 2 / (np.pi ** 2 * a ** 2) + a ** 2 * np.sin(phi) ** 2) ** (3 / 2)))
        partial_derivative_phi = (- (A * (a ** 4 - A ** 2 / np.pi ** 2) * np.sin(2 * phi)) /
                                  (2 * np.pi * a ** 2 * (
                                  (A ** 2 * np.cos(phi) ** 2) / (np.pi ** 2 * a ** 2) + a ** 2 * np.sin(phi) ** 2) ** (
                                   3 / 2)))
        ref_rho_dot = partial_derivative_a * a_dot + partial_derivative_phi * dot_phi

        dot_rho = ref_rho_dot + self.kp * (ref_rho - rho)



        #print self.name,'(ref_rho - rho)',round(ref_rho - rho,3)
        vel_vec = np.zeros(2)
        vel_vec[0] = dot_rho * np.cos(phi) - rho * dot_phi * np.sin(phi) + x_dot
        vel_vec[1] = dot_rho * np.sin(phi) + rho * dot_phi * np.cos(phi) + y_dot

        # feedback linearization
        d = self.feedback_d
        v = np.cos(self.yaw)*vel_vec[0]+np.sin(self.yaw)*vel_vec[1]
        #if v<0.:
        #    v=0.
        if np.abs(v)>self.exp_obj.robot_vel:
            v = self.exp_obj.robot_vel*np.sign(v)


        w = -np.sin(self.yaw)/d*vel_vec[0]+np.cos(self.yaw)/d*vel_vec[1]
        if np.abs(w)>self.exp_obj.robot_rot:
            w = self.exp_obj.robot_rot*np.sign(w)
        if self.cnter==0:
            self.cnter = 30

            print self.name, 'v', round(v, 3),'w',round(w,3), 'e_rho',round(self.kp * (ref_rho - rho),3),'e_phi',\
                round(self.kphi * phi_error,3),'angle(v)',round(np.arctan2(vel_vec[1],vel_vec[0])/np.pi*180)
        self.cnter -= 1

        self.logfile.write("%f %f %f %f %f\r\n" % (toSecs(rospy.Time.now()),(ref_rho - rho), phi_error,v,w))

        # publish
        new_cmd = Twist()
        new_cmd.linear.x = v
        new_cmd.angular.z = w
        if self.IS_SIM:
            new_cmd.linear.x *= 10
        self.Pub_twist.publish(new_cmd)
        #print self.name,'time diff',round(toSecs(rospy.get_rostime()-first),5)

    def draw_shape(self,x,y,a,b):
        if self.RVIZ:
            shape_marker = Marker()
            shape_marker.ns = self.name
            shape_marker.id = 1
            shape_marker.header.frame_id = "map"  # odom TODO map?
            shape_marker.header.stamp = rospy.Time.now()
            shape_marker.type = Marker.CYLINDER

            shape_marker.scale.x = a*2
            shape_marker.scale.y = b*2
            shape_marker.scale.z = self.exp_obj.robot_size/5

            shape_marker.pose.position.x = x
            shape_marker.pose.position.y = y
            shape_marker.pose.position.z = self.exp_obj.robot_size/10

            shape_marker.pose.orientation.x = 0
            shape_marker.pose.orientation.y = 0
            shape_marker.pose.orientation.z = 0
            shape_marker.pose.orientation.w = 1

            shape_marker.color.a = 1
            shape_marker.color.r,shape_marker.color.g,shape_marker.color.b = self.exp_obj.colors_robots[self.ID]
            shape_marker.color.r *= 0.1
            shape_marker.color.g *= 0.1
            shape_marker.color.b *= 0.1

            self.Pub_rviz.publish(shape_marker)

    def draw_robot(self):
        if self.RVIZ:
            # draw all the nodes from the tree
            point = Marker()
            point.ns = self.name
            point.id = 0
            point.header.frame_id = "map"  # odom TODO map?
            point.header.stamp = rospy.Time.now()
            point.type = Marker.CUBE

            point.scale.x = self.exp_obj.robot_size
            point.scale.y = self.exp_obj.robot_size
            point.scale.z = self.exp_obj.robot_size

            point.pose.position.x = self.own_pose.position.x
            point.pose.position.y = self.own_pose.position.y
            point.pose.position.z = self.exp_obj.robot_size/2

            point.pose.orientation.x = 0
            point.pose.orientation.y = 0
            point.pose.orientation.z = self.yaw
            point.pose.orientation.w = 1

            point.color.a = 1.0
            point.color.r, point.color.g, point.color.b = self.exp_obj.colors_robots[self.ID]

            self.Pub_rviz.publish(point)

    # triggers that the plan gets progressed
    def trigger_plan(self, event):
        if self.next_plan == None:
            #print "keep current node"
            return
        #print self.name, "shifted plan to next committed bid"
        self.current_plan = self.next_plan
        self.next_plan = None

    def just_go_back(self):
        import numpy.random as rnd
        print self.name, 'going back into view of camera'
        new_twist = Twist()
        if rnd.rand()<0.5:
            new_twist.linear.x = -10
        else:
            new_twist.linear.x =  10

        if rnd.rand()<0.5:
            new_twist.angular.z = -1.5
        else:
            new_twist.angular.z =  1.5
        self.Pub_twist.publish(new_twist)
        time.sleep(1.5)
        self.Pub_twist.publish(Twist())
        print self.name, 'better?'
        time.sleep(5)


if __name__ == "__main__":
    try:
        controller_node = controller_node()
    except rospy.ROSInterruptException:
        pass
