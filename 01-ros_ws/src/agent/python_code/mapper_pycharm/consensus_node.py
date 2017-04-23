#!/usr/bin/env python
import numpy as np
import rospy
from agent.msg import bid
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from util.util import toSecsNsecs

from config.experiments import experiments


class consensus_node:
    def __init__(self):
        # register node
        self.ID = 0
        self.name = 'consensus_{}'.format(self.ID)
        rospy.init_node(self.name)
        self.name = rospy.get_name()
        print self.name, "registered"

        # Parameters from ros call
        self.IS_SIM = rospy.get_param('simulated_robots', True)  # True := Morse Simulator
        self.verbose = rospy.get_param('~verbose', 0)  # True := Morse Simulator
        self.SIM_FACTOR = rospy.get_param('simulator_factor', 10.)  # True := Morse Simulator
        self.ID = rospy.get_param('~ID', self.ID)  # ID of the agent
        self.verbose = rospy.get_param('~verbose', 0)
        self.EXPNUM = rospy.get_param('EXPNUM',
                                      0)  # number of the experiment object, see the config/experiments.py file
        self.RVIZ = True

        # Load the maps through python file
        self.exp_obj = experiments[self.EXPNUM] # has the two maps inside

        # initial bid
        self.committed_bid = bid()
        self.committed_bid.add_info = 'False' #is not a solution
        self.committed_bid.cost = np.Inf
        self.committed_bid.params = [0.,0.,self.exp_obj.q_init[0],0.,0.,self.exp_obj.q_init[1],0.,0.,self.exp_obj.q_init[2]]
        self.committed_bid.robot_id = -1
        self.committed_bid.end_q = self.exp_obj.q_init
        self.committed_bid.start_q = self.exp_obj.q_init
        #self.reset_bid()


        # flag to toggle between 'publishing' and 'just accepting' the consensus process
        self.is_publishing = True

        # Syncing topic
        self.run = False

        # time management
        self.iteration = 0.
        self.iteration_time = self.exp_obj.tree_edge_dt
        self.iteration_planning_time = self.iteration_time*(1.-self.exp_obj.other_params['consensus_percent']) # 20%
        self.iteration_time_commit = self.iteration_time*(1.-self.exp_obj.other_params['consensus_percent']/2.) # 10%
        #print self.iteration_time,self.iteration_planning_time,self.iteration_time_commit

        # Neighbors
        self.NeigborLeft = (self.ID - 1) % self.exp_obj.n_robots
        self.NeigborRight = (self.ID + 1) % self.exp_obj.n_robots
        #print self.name,"left:",self.NeigborLeft,"right:",self.NeigborRight

        # consensus parameters
        #self.consensus_interval = self.exp_obj.tree_edge_dt
        #self.consensus_settle = self.exp_obj.other_params['consensus_percent'] # 0%..100%
        #self.consensus_interval_publish = self.consensus_interval*(1-self.consensus_settle)
        #self.consensus_interval_settle = self.consensus_interval-self.consensus_interval_publish

        # register publisher current Bid
        self.Pub_bid = rospy.Publisher('~bid', bid, latch=True,queue_size=1)
        self.Pub_committed_bid = rospy.Publisher('~committed_bid', bid, latch=True,queue_size=1)
        if self.RVIZ:
            self.Pub_rviz = rospy.Publisher('/vsl_mrkr', Marker, latch=True,queue_size=1)


        # register Subscriber to get positions s.t. the node updates the fusioned map self.map
        self.Sub_bids_left = rospy.Subscriber('/consensus_{}/bid'.format(self.NeigborLeft),bid, self.handle_bid_message)
        self.Sub_bids_right = rospy.Subscriber('/consensus_{}/bid'.format(self.NeigborRight),bid, self.handle_bid_message)
        self.Sub_bids_self = rospy.Subscriber('/planner_{}/bid'.format(self.ID),bid, self.handle_own_bid_message)
        self.Sub_sync_cmd = rospy.Subscriber('/sync_cmd',String,self.handle_sync_message)


        # for toggling the state
        self.timer_toggle_on = None
        self.timer_toggle_off = None
        self.timer_commit = None
        self.has_set_timer_off = False
        self.has_set_timer_commit = False

        rospy.spin()



    # callbacks
    def handle_sync_message(self,str):
        str = str.data
        if str=='start':
            #print 'on',rospy.Time.now().secs,rospy.Time.now().nsecs
            #self.start_time = rospy.get_time()
            if self.timer_toggle_on is not None:
                self.timer_toggle_on.shutdown()
                self.timer_toggle_off.shutdown()
                self.timer_commit.shutdown()

            self.timer_toggle_off = rospy.Timer(rospy.Duration(*toSecsNsecs(self.iteration_planning_time)), self.toggle_state_off, oneshot=True)
            self.timer_commit = rospy.Timer(rospy.Duration(*toSecsNsecs(self.iteration_time_commit)), self.commit_plan, oneshot=True)
            self.timer_toggle_on = rospy.Timer(rospy.Duration(*toSecsNsecs(self.iteration_time)), self.toggle_state_on, oneshot=False)
            self.run = True
            self.is_publishing = True


        elif str=='pause':
            self.run = False
            self.is_publishing=False
            if self.timer_toggle_on is not None:
                self.timer_toggle_on.shutdown()
                self.timer_toggle_off.shutdown()
                self.timer_commit.shutdown()


    def handle_bid_message(self,Bid):
        if not self.run:
            return

        if self.committed_bid == None:
            self.committed_bid = Bid
            if self.verbose >= 1:
                print self.name,'bid accepted and broadcasted as no other bid was there before from',Bid.robot_id
            self.Pub_bid.publish(Bid)
            return

        if self.committed_bid.add_info == 'True' and Bid.add_info == 'True':
            if self.committed_bid.cost > Bid.cost:
                if self.verbose >= 1:
                    print self.name, 'bid of solution accepted and broadcasted because it was better from',Bid.robot_id,'than from',self.committed_bid.robot_id
                self.committed_bid = Bid
                self.Pub_bid.publish(Bid)
            else:
                if self.verbose >= 1:
                    print self.name, 'bid of solution rejected because it was worse from',Bid.robot_id,'than the existing from',self.committed_bid.robot_id

        if self.committed_bid.add_info == 'True' and Bid.add_info == 'False':
            if self.verbose >= 1:
                print self.name, 'bid of solution rejected because it is a candidate from', Bid.robot_id, 'than the existing solution from', self.committed_bid.robot_id
            return

        if self.committed_bid.add_info == 'False' and Bid.add_info == 'True':
            if self.verbose >= 1:
                print self.name, 'bid of solution accepted and broadcasted because it is a solution from', Bid.robot_id, 'than candidate from', self.committed_bid.robot_id
            self.committed_bid = Bid
            self.Pub_bid.publish(Bid)
            return

        if self.committed_bid.add_info == 'False' and Bid.add_info == 'False':
            if self.committed_bid.cost > Bid.cost:
                if self.verbose >= 1:
                    print self.name, 'bid of candidate accepted and broadcasted because it was better from',Bid.robot_id,'than from',self.committed_bid.robot_id
                self.committed_bid = Bid
                self.Pub_bid.publish(Bid)
            else:
                if self.verbose >= 1:
                    print self.name, 'bid of candidate rejected because it was worse from',Bid.robot_id,'than from',self.committed_bid.robot_id



    def handle_own_bid_message(self,Bid):
        if not self.run:
            print self.name,'is not running'
            return
        if not self.is_publishing:
            print self.name,'received late bid from itself'
            return
        self.handle_bid_message(Bid)



    # toggles the state on
    def toggle_state_on(self,event):
        #print 'on',rospy.Time.now().secs,rospy.Time.now().nsecs
        if self.ID == 0:
            print '' # separator for intervals
        self.is_publishing = True

    def toggle_state_off(self,event):
        #print 'off',rospy.Time.now().secs,rospy.Time.now().nsecs
        self.is_publishing = False
        if not self.has_set_timer_off:
            self.timer_toggle_off = rospy.Timer(rospy.Duration(*toSecsNsecs(self.iteration_time)),
                                                self.toggle_state_off,oneshot=False)
            self.has_set_timer_off = True

    def commit_plan(self,event):
        if self.run:
            #print self.name,'commit!',rospy.Time.now().secs,rospy.Time.now().nsecs
            self.Pub_committed_bid.publish(self.committed_bid)
            self.committed_bid = None

            if self.verbose >= 1:
                print self.name, 'iteration',self.iteration
            self.iteration += 1
            #self.committed_bid.cost = np.Inf
            if not self.has_set_timer_commit:
                self.timer_commit = rospy.Timer(rospy.Duration(*toSecsNsecs(self.iteration_time)), self.commit_plan,
                                                oneshot=False)
                self.has_set_timer_commit = True


if __name__ == "__main__":
    try:
        consensus_node = consensus_node()
    except rospy.ROSInterruptException:
        pass