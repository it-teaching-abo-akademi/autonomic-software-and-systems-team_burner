# UNDER HEAVY CONSTRUCTION

# TODO: Move all pathfinding-stuff here and tidy it up a whole lot.
# Having the navigator in its own class makes the code easier to read and manage.
#
# self.vertices: a list of relevant waypoints, mainly intersections. After a search is made, this also holds the paths.
# self.edges: a list of the roads between the waypoints.

import math
import pandas as pd


class Navigator(object):
    def __init__(self, knowledge, topology):
        self.knowledge = knowledge
        pd.options.display.max_colwidth = 200

        # Read the topology of the map and parse the edges. Each edge has a starting and ending waypoint.transform and
        # a length. The waypoints are stored as transform-components and not as Waypoints because the naming clashes
        # with a pandas.DataFrame-method, which causes a lot of problematic situations.
        self.edges = pd.DataFrame(topology)
        self.edges = self.edges.rename(index=str, columns={0: 'start', 1: 'end'})
        self.edges['start'] = [row.start.transform for index, row in self.edges.iterrows()]
        self.edges['end'] = [row.end.transform for index, row in self.edges.iterrows()]
        self.edges['length'] = [row.start.location.distance(row.end.location) for index, row in self.edges.iterrows()]

        # Create a list of vertices based on the list of edges. This list stores all variables needed for Dijkstra's
        # shortest path algorithm.
        self.vertices = pd.DataFrame(columns=['location', 'known', 'distance', 'path'])
        for index, vertex in self.edges.iterrows():
            location = vertex.start
            if location not in self.vertices.location.values:
                self.vertices = self.vertices.append([{'location': location, 'known': False, 'distance': math.inf, 'path': None}])
'''

        def find_closest(current_waypoint, vertices, only_unknown=False):
            shortest_distance = math.inf
            closest_vertex = None
            for index, point in vertices.iterrows():
                location = point.location
                known = point['known']
                distance = current_waypoint.distance(location)
                if distance < shortest_distance:
                    if (not only_unknown) or (only_unknown and not known):
                        shortest_distance = distance
                        closest_vertex = point
            return closest_vertex

        def update_vertex(vertex, distance=math.inf, path=None, known=None):
            updated = vertices.loc[vertices.location == vertex.location]
            if known is not None:
                updated.at[0, 'known'] = known
            if distance < math.inf:
                updated.at[0, 'distance'] = distance
            if path is not None:
                updated.at[0, 'path'] = path
            vertices.loc[vertices.location == vertex.location] = updated

        def find_next_unprocessed():
            shortest_path = math.inf
            next_vertex = None
            for vertex in vertices.iterrows():
                if vertex[0].distance <= shortest_path:
                    shortest_path = vertex[1].distance
                    next_vertex = vertex[0]
            return next_vertex

        current_waypoint = self.world.get_map().get_waypoint(location)  # Find the waypoint that's currently closest to the car
        current_vertex = find_closest(current_waypoint.transform.location, vertices)  # Find the vextex that is closest to our current waypoint

        # Update vertex table with the start node
        update_vertex(current_vertex, distance=0, path=None, known=True)

        # Find all edges that begin at the start node and update distances to their end vertices
        for index, edge in distances[distances.start == current_vertex.location].iterrows():
            end_vertex = edge.end
            new_distance = edge.distance + current_vertex.distance
            print("{:3.2f} + {:3.2f} = {:3.2f}".format(edge.distance, current_vertex.distance, new_distance))
            next_vertex = find_closest(end_vertex, vertices, only_unknown=True)
            if next_vertex.distance > new_distance:
                update_vertex(next_vertex, distance=new_distance, path=current_vertex)
                self.world.debug.draw_string(next_vertex.location, "X", life_time=20.0)

        next_vertex = next_vertex.location
        for i in range(20):
            current_vertex = find_closest(next_vertex, vertices, only_unknown=True)  # Find the vextex that is closest to our current waypoint
            update_vertex(current_vertex, distance=0, path=None, known=False)
            for index, vertex in distances[distances.start == current_vertex.location].iterrows():
                next_vertex = vertex.end

            self.world.debug.draw_string(next_vertex, "X", life_time=20.0)
            updated.at[0, 'path'] = path
        vertices.loc[vertices.location == vertex.location] = updated
  '''
