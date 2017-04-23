from util.svg_to_map import svg_to_map
import numpy as np

class Experiment:
    #def __init__(self, map, X, U, q_init, shape_name, goal_distance, n_robots, robot_vel, tree_edge_dt=1, robot_size=0,
    #             shape_params={},controller_params={}, other_params = {}):
    def __init__(self, *args, **kwargs):
        self.shape_params =  kwargs.get('shape_params',{})
        self.robot_size =  kwargs.get('robot_size',0.)
        self.robot_vel =  kwargs.get('robot_vel')
        self.n_robots =  kwargs.get('n_robots')
        self.goal_distance =  kwargs.get('goal_distance')
        self.shape_name =  kwargs.get('shape_name')
        self.U =  kwargs.get('U')
        self.X =  kwargs.get('X')  # state_space
        self.map_init =  kwargs.get('map_init')
        self.map_real =  kwargs.get('map_real')
        self.controller_params =  kwargs.get('controller_params',{})
        self.tree_edge_dt = kwargs.get('tree_edge_dt',1.)
        self.other_params = kwargs.get('other_params',{})
        self.resolution = kwargs.get('resolution',0.01)

        if len(self.map_init)==4:
            E, Obs, start, goal = self.map_init
        else:
            E, Obs, Obs_type, start, goal = self.map_init

        ## map values
        self.q_init = np.zeros(2*len(self.U))
        self.q_init[0], self.q_init[1] = start
        self.q_init[0] *= self.resolution
        self.q_init[1] *= self.resolution
        self.q_init[2] = np.mean(self.X[2])

        self.q_final = np.zeros(2 * len(self.U))
        self.q_final[0], self.q_final[1] = goal
        self.q_final[0] *= self.resolution
        self.q_final[1] *= self.resolution
        self.q_final[2] = np.mean(self.X[2])

        # state space based on map boundaries
        w, h = E.bounds[2], E.bounds[3]
        self.X[0] = (0, w*self.resolution)
        self.X[1] = (0, h*self.resolution)

experiments = \
    [
        # 1 Jornal extension, static map with discoverable obstacle (blue) and fixed obstacles (black), slightly more difficult
        Experiment(map_init = svg_to_map('01-simple_init.svg'),
                   map_real = svg_to_map('01-simple_real.svg'),
                   resolution = 0.01,
                   X=[(0, -1), (0, -1), (0.20, 0.40),
                      (-0.05, 0.05), (-0.05, 0.05), (-0.10, 0.10)],
                   U=((-0.035, 0.035), (-0.035, 0.035), (-0.035, 0.035)),
                   shape_name='ellipse_shape',
                   shape_params={'area': 0.282},  # = pi*0.3**2
                   goal_distance=0.2,
                   robot_vel=0.05,
                   n_robots=5,
                   robot_size=0.1,
                   controller_params={'dt_fine': 0.1, 'kp': 45.05, 'kphi': 20.5, 'Omeg': 4.84,
                                      'init x': [0.48,0.4,0.66,0.92,0.69], 'init y': [0.56,0.32,0.26,0.49,0.68],
                                      'feedback_d':0.05},
                   tree_edge_dt=1.,
                   other_params={'robot_sense_range': 0.5, 'robot_sense_type':'circle', 'goal_bias':0.15,
                                 'consensus_percent':0.2}# percentage of tree_edge_dt to allow consensus to settle
                   ),

    ]
