import matplotlib.pyplot as plt
from matplotlib.colors import ColorConverter
from shapely.geometry import Polygon
import numpy as np
from matplotlib.collections import LineCollection


BLUE = '#6699cc'
GRAY = '#F3F2F2'

OBS_COLOR = 'black'
OBS_DISC_COLOR = 'blue'



def draw_sgoal(start, goal, drawer=plt):
    drawer.plot(start[0], start[1], 'o')  # Start
    drawer.plot(goal[0], goal[1], 'x')  # Goal


def draw_shape(sx, sy, q, s=np.linspace(0, 1, 20), pars={'linestyle':'-','color':'black'}, drawer=plt):
    return drawer.plot(sx(s, q), sy(s, q), **pars)


def draw_polygon(pol, color1=BLUE):
    ax = plt.axes()
    patch = PolygonPatch(pol, fc=color1, ec=color1, alpha=0.5, zorder=2)
    ax.add_patch(patch)

def tree_to_segments(nodes):
    #x = [(node.parent.x[0], node.x[0]) for node in nodes[1:]]
    #y = [(node.parent.x[1], node.x[1]) for node in nodes[1:]]
    segments = [((node.parent.x[0], node.parent.x[1]), (node.x[0], node.x[1])) for node in nodes[1:]]
    #line_segments = LineCollection(segments, linestyles='solid',colors=ColorConverter(color))

    return segments
