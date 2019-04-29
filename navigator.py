import math
import carla
import numpy as np
import pandas as pd

class Navigator(object):
    def __init__(self, other_self):
        self.other_self = other_self

    def make_vertex_list(self, edge_list):
        self = self.other_self

        distances = pd.DataFrame()
        vertex_list = []
        player_vertex = []

        for edge in edge_list:

            start = edge[0].transform.location
            end = edge[1].transform.location
            distances.loc[str(start), str(end)] = self.knowledge.distance(start, end)
            if str(start) not in vertex_list:
                vertex_list.append(str(start))
        return distances
