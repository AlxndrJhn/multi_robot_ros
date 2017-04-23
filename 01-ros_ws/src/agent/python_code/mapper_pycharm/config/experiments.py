import numpy as np

from util.svg_to_map import svg_to_map


class Experiment:
    #def __init__(self, map, X, U, q_init, shape_name, goal_distance, n_robots, robot_vel, tree_edge_dt=1, robot_size=0,
    #             shape_params={},controller_params={}, other_params = {}):
    def __init__(self, *args, **kwargs):
        self.shape_params =  kwargs.get('shape_params',{})
        self.robot_size =  kwargs.get('robot_size',0.)
        self.robot_vel =  kwargs.get('robot_vel')
        self.robot_rot = kwargs.get('robot_rot')
        self.n_robots =  kwargs.get('n_robots')
        self.goal_distance =  kwargs.get('goal_distance')
        self.shape_name =  kwargs.get('shape_name')
        self.U =  kwargs.get('U')
        self.X =  kwargs.get('X')  # state_space
        self.n_dim = len(self.X)
        self.map_init =  kwargs.get('map_init')
        self.map_real =  kwargs.get('map_real')
        self.controller_params =  kwargs.get('controller_params',{})
        self.tree_edge_dt = kwargs.get('tree_edge_dt',1.)
        self.other_params = kwargs.get('other_params',{})
        self.resolution = kwargs.get('resolution',0.01)
        self.colors_robots = kwargs.get('colors_robots',[])
        self.map_real_path = kwargs.get('map_real_path','')

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

        # params that the controller can use to start circulating the initial configuration q_init
        self.params = [(0.,0.,self.q_init[0]),(0.,0.,self.q_init[1]),(0.,0.,self.q_init[2])]

        # state space based on map boundaries
        self.w, self.h = E.bounds[2], E.bounds[3]
        self.X[0] = (0, self.w*self.resolution)
        self.X[1] = (0, self.h*self.resolution)

    def params_as_array(self):
        return [x for xs in self.params for x in xs]

experiments = \
    [
        # 0 Jornal extension, static map
        Experiment(map_init = svg_to_map('01-simple_init.svg'),
                   map_real = svg_to_map('01-simple_real.svg'),
                   map_real_path = "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/python_code/mapper_pycharm/maps/01-simple_real.svg",
                   resolution = 0.01,
                   X=[(0, -1), (0, -1), (0.20, 0.40),
                      (-0.014, 0.014), (-0.014, 0.014), (-0.10, 0.10)],
                   U=((-0.035, 0.035), (-0.035, 0.035), (-0.035, 0.035)),
                   shape_name='ellipse_shape',
                   shape_params={'area': 0.282},  # = pi*0.3**2
                   goal_distance=0.2,
                   robot_vel=0.06,
                   robot_rot=1.57,
                   n_robots=5,
                   robot_size=0.1,
                   controller_params={'dt_fine': 0.1, 'kp': 10., 'kphi': 20., 'Omeg': 1.50,
                                      'init x': [0.48,0.4,0.66,0.92,0.69], 'init y': [0.76,0.52,0.46,0.69,0.88],
                                      'feedback_d':0.05},
                   colors_robots=[(0.5,0.5,0.5),(0.,0.,1.),(0.,1.,0.),(1.,0.,0.),(1.,1.,0.)],
                   tree_edge_dt=5.,
                   other_params={'robot_sense_range': 0.35, 'robot_sense_type':'circle', 'goal_bias':0.25,
                                 'consensus_percent':0.2, 'pre_start_time':1.}# fraction of tree_edge_dt to allow consensus to settle
                   ),
        # 1 Jornal extension, static map with discoverable obstacle slightly more difficult
        Experiment(map_init=svg_to_map('02-medium_init.svg'),
                   map_real=svg_to_map('02-medium_real.svg'),
                   map_real_path="/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/python_code/mapper_pycharm/maps/02-medium_real.svg",
                   resolution=0.01,
                   X=[(0, -1), (0, -1), (0.1, 0.2),
                      (-0.007, 0.007), (-0.007, 0.007), (-0.10, 0.10)],
                   U=((-0.0035, 0.0035), (-0.0035, 0.0035), (-0.035, 0.035)),
                   shape_name='ellipse_shape',
                   shape_params={'area': 0.07065},  # = pi*0.15**2
                   goal_distance=0.2,
                   robot_vel=0.03,
                   robot_rot=1.57,
                   n_robots=5,
                   robot_size=0.05,
                   controller_params={'dt_fine': 0.1, 'kp': 10., 'kphi': 20., 'Omeg': 1.50,
                                      'init x': [0.18, 0.32, 0.68, 0.53, 0.18], 'init y': [1.26, 1.48, 1.23, 1.02, 1.00],
                                      'feedback_d': 0.025},
                   colors_robots=[(0.5, 0.5, 0.5), (0., 0., 1.), (0., 1., 0.), (1., 0., 0.), (1., 1., 0.)], #RGB
                            #           Grey            Blue        Green           Red        Yellow
                   tree_edge_dt=5.,
                   other_params={'robot_sense_range': 0.175, 'robot_sense_type': 'circle', 'goal_bias': 0.25,
                                 'consensus_percent': 0.2, 'pre_start_time': 1.}
                   # fraction of tree_edge_dt to allow consensus to settle
                   ),

        # 2 Jornal extension, static map with discoverable obstacle slightly more difficult
        Experiment(map_init=svg_to_map('03-tricky_init.svg'),
                   map_real=svg_to_map('03-tricky_real.svg'),
                   map_real_path="/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/python_code/mapper_pycharm/maps/03-tricky_real.svg",
                   resolution=0.01,
                   X=[(0, -1), (0, -1), (0.1, 0.2),
                      (-0.007, 0.007), (-0.007, 0.007), (-0.10, 0.10)],
                   U=((-0.0035, 0.0035), (-0.0035, 0.0035), (-0.035, 0.035)),
                   shape_name='ellipse_shape',
                   shape_params={'area': 0.07065},  # = pi*0.15**2
                   goal_distance=0.2,
                   robot_vel=0.03,
                   robot_rot=1.57,
                   n_robots=5,
                   robot_size=0.05,
                   controller_params={'dt_fine': 0.1, 'kp': 10., 'kphi': 20., 'Omeg': 1.50,
                                      'init x': [0.18, 0.32, 0.68, 0.53, 0.18],
                                      'init y': [1.26, 1.48, 1.23, 1.02, 1.00],
                                      'feedback_d': 0.025},
                   colors_robots=[(1., 0., 1.), (0., 0., 1.), (0., 1., 0.), (1., 0., 0.), (1., 1., 0.)],  # RGB
                   #                 Purple            Blue        Green           Red        Yellow
                   tree_edge_dt=3.,
                   other_params={'robot_sense_range': 0.2, 'robot_sense_type': 'circle', 'goal_bias': 0.05,
                                 'consensus_percent': 0.2, 'pre_start_time': 1.}
                   # fraction of tree_edge_dt to allow consensus to settle
                   ),

        # 3 Jornal extension, static map with discoverable obstacles complex
        Experiment(map_init=svg_to_map('05-complex_init.svg'),
                   map_real=svg_to_map('05-complex_real.svg'),
                   map_real_path="/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/python_code/mapper_pycharm/maps/05-complex_real.svg",
                   resolution=0.01,
                   X=[(0, -1), (0, -1), (0.11, 0.19),
                      (-0.008, 0.008), (-0.008, 0.008), (-0.04, 0.04)],
                   U=((-0.0035, 0.0035), (-0.0035, 0.0035), (-0.005, 0.005)),
                   shape_name='ellipse_shape',
                   shape_params={'area': 0.07065},  # = pi*0.15**2
                   goal_distance=0.2,
                   robot_vel=0.03,
                   robot_rot=1.57,
                   n_robots=5,
                   robot_size=0.05,
                   controller_params={'dt_fine': 0.1, 'kp': 10., 'kphi': 20., 'Omeg': 1.50,
                                      'init x': [0.18, 0.32, 0.68, 0.53, 0.18],
                                      'init y': [1.26, 1.48, 1.23, 1.02, 1.00],
                                      'feedback_d': 0.025},
                   colors_robots=[(1., 0., 1.), (0., 0., 1.), (0., 1., 0.), (1., 0., 0.), (1., 1., 0.)],  # RGB
                   #                 Purple            Blue        Green           Red        Yellow
                   tree_edge_dt=5.,
                   other_params={'robot_sense_range': 0.3, 'robot_sense_type': 'circle', 'goal_bias': 0.15,
                                 'consensus_percent': 0.12, 'pre_start_time': 0.}
                   # fraction of tree_edge_dt to allow consensus to settle
                   ),
        # 4 Jornal extension, static map with discoverable obstacles complex, real robots
        Experiment(map_init=svg_to_map('06-VerlabTable_init.svg'),
                   map_real=svg_to_map('06-VerlabTable_real.svg'),
                   map_real_path="/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/python_code/mapper_pycharm/maps/06-VerlabTable_real.svg",
                   resolution=0.01,
                   X=[(0, -1), (0, -1), (0.07, 0.13),
                      (-0.015, 0.015), (-0.006, 0.006), (-0.01, 0.01)],
                   U=((-0.0035, 0.0035), (-0.0035, 0.0035), (-0.002, 0.002)),
                   shape_name='ellipse_shape',
                   shape_params={'area': 0.0314},  # = pi*0.10**2; 0.15 -> 0.07065
                   goal_distance=0.05,
                   robot_vel=1.8,
                   robot_rot=0.8,
                   n_robots=5,
                   robot_size=0.05,
                   controller_params={'dt_fine': 0.05, 'kp': 20., 'kphi': 30., 'Omeg': 6.,
                                      'init x': [0.10,0.11,0.38,0.31,0.21],
                                      'init y': [0.67,0.40,0.39,0.69,0.91],
                                      'feedback_d': 1.0},
                   colors_robots=[(1., 0., 1.), (0., 0., 1.), (0., 1., 0.), (1., 0., 0.), (1., 1., 0.)],  # RGB
                   #                 Purple            Blue        Green           Red        Yellow
                   tree_edge_dt=2,
                   other_params={'robot_sense_range': 0.15, 'robot_sense_type': 'circle', 'goal_bias': 0.05,
                                 'consensus_percent': 0.09, 'pre_start_time': 2.}
                   # fraction of tree_edge_dt to allow consensus to settle
                   ),
    ]
