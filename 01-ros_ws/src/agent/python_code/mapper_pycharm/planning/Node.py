from util.util import shape2polygon
import numpy as np


class Node(object):
    def __init__(self, state_vector,parent=None,q_params=None,time=None,distance=None):
        self.x = state_vector
        self.id = 0
        self.parent = parent
        self.children = []

        # Integration parameters
        self.q_params = q_params
        if time ==None:
            time = 0.
        self.time = time

        # Distance to goal
        self.distance = distance

        # center of mass
        self._centroid = None


    def get_node_sequence(self,end_id=0):
        node_sequence = []
        last_node = self

        while True:
            node_sequence.insert(0,last_node)
            if last_node.parent == None or last_node.id==end_id:
                break
            last_node = last_node.parent

        return node_sequence

    def get_centroid(self, sx, sy, n):
        if self._centroid is None:
            poly = shape2polygon(sx, sy, self.x[:n], np.linspace(0, 1, 20))
            self._centroid = (poly.centroid.x, poly.centroid.y)
        return self._centroid

    def __str__(self):
        return "node(" + str(self.x) + ")"
