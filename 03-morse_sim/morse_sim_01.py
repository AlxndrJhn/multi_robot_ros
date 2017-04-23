#! /usr/bin/env python3

# How to simulate
# 1. start 'roscore' first
# start by running 'morse run -g 200x200 /home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/morse_sim/morse_sim_01.py'

from morse.builder import *
import os
import sys
import numpy as np
#sys.path.append(os.path.dirname(__file__))
sys.path.append('/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/python_code/mapper_pycharm/')
from config.experiments import experiments


# Load the experiment
exp_obj = experiments[3]  # has the two maps inside
filename = exp_obj.map_real_path

# Take image, generate adequate blender file from it
os.system("blender --background --python blender.py -- "+filename+" > /dev/null")
#os.system("blender --background --python blender.py -- "+filename)
#exit(0)

E, Obs, start, goal = exp_obj.map_real
factor = 10
env = Environment("exp/exp_env",fastmode=False)#
env.set_camera_location([E.bounds[2]/2.*exp_obj.resolution*factor, E.bounds[3]/2.*exp_obj.resolution*factor, 2.5*factor])
env.set_camera_rotation([0, 0, 0])
size = exp_obj.robot_size


agents_x = exp_obj.controller_params['init x']
print(agents_x)
agents_y = exp_obj.controller_params['init y']
for i in range(0,exp_obj.n_robots):
    # Take experiment object and place the n robots in the environment
    atrv = Robot('exp/atrv_no_bound')
    atrv.scale = (1./1.14*size/np.sqrt(2)*factor,size/np.sqrt(2)*factor,1./0.831*.1*factor)
    atrv.translate(agents_x[i]*factor,agents_y[i]*factor,0.0125*factor)
    atrv.add_default_interface('ros')

    # atuator
    motion = MotionVW()
    motion.translate(z=0.1*factor)
    atrv.append(motion)

    # sensor
    pose = Pose()
    pose.translate(z=0.1*factor)
    atrv.append(pose)

    # Initialize the ros nodes for each agent
    pose.add_stream('ros', topic="/robot_{}/PoseStamped".format(i))
    motion.add_stream('ros', topic="/robot_{}/Twist".format(i))