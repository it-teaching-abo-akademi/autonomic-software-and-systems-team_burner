#!/usr/bin/env python

import glob
import os
import sys
from collections import deque
import math

from ai_knowledge import Status
try:
    sys.path.append(glob.glob('**/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("Can't find CARLA")
    exit(0)

import carla


# Executor is responsible for moving the vehicle around
# In this implementation it only needs to match the steering and speed so that we arrive at provided waypoints
# BONUS TODO: implement different speed limits so that planner would also provide speed target speed in addition to direction
class Executor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        self.target_pos = knowledge.get_location()

    # Update the executor at some intervals to steer the car in desired direction
    def update(self, time_elapsed):
        status = self.knowledge.get_status()
        # TODO: this needs to be able to handle
        if status == Status.DRIVING:
            dest = self.knowledge.get_current_destination()
            self.update_control(dest, [1], time_elapsed)

    # TODO: Take into account that exiting the crash site could also be done in reverse, so there might need to be additional
    #  data passed between planner and executor, or there needs to be some way to tell this that it is ok to drive in reverse
    #  during HEALING and CRASHED states. An example is additional_vars, that could be a list with parameters that can tell us
    #  which things we can do (for example going in reverse)
    def update_control(self, destination, additional_vars, delta_time):
        speed = self.knowledge.retrieve_data('speed')
        target_speed = self.knowledge.retrieve_data('target_speed')

        # Initialize throttle and heading
        control = self.vehicle.get_control()

        # TODO: Combine the threshold calculator and this calculator in the planner, and refine it. The car still wobbles a lot.
        heading_diff = self.knowledge.retrieve_data('heading_diff')
        control.steer = heading_diff / 2

        # Calculate the difference between the current speed and the target speed, and adjust throttle accordingly.
        # Since different cars responsed differently to the same amount of throttle, this needs to be extended with
        # an adaptive feedback loop.
        # TODO: Adaptive feedback
        speed_diff = target_speed - speed
        if target_speed == 0:
            control.throttle = 0.0
            control.brake = 1.0
        elif speed_diff == 0:
            control.throttle = 0
            control.brake = 0
        elif speed_diff < 0:
            control.throttle = 0
            control.brake = 0.3
        elif speed_diff > 0:
            control.throttle = min((speed_diff / target_speed) + 0.2 , 1.0) - abs(control.steer)
            control.brake = 0.0

        self.vehicle.apply_control(control)


# Planner is responsible for creating a plan for moving around
# In our case it creates a list of waypoints to follow so that vehicle arrives at destination
# Alternatively this can also provide a list of waypoints to try avoid crashing or 'uncrash' itself
class Planner(object):
    def __init__(self, knowledge):
        self.knowledge = knowledge
        self.path = deque([])
        self.world = self.knowledge.retrieve_data('world')
        self.map = self.world.get_map()

    # Create a map of waypoints to follow to the destination and save it
    def make_plan(self, source, destination):
        self.path = self.build_path(source, destination)
        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        at_lights = self.knowledge.retrieve_data('at_lights')

        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

        if at_lights or len(self.path) == 0:
            self.knowledge.update_data('target_speed', 0)
        else:
            self.knowledge.update_data('target_speed', self.knowledge.retrieve_data('speed_limit'))

    # Update internal state to make sure that there are waypoints to follow and that we have not arrived yet
    def update_plan(self):
        if len(self.path) == 0:
            return

        if self.knowledge.arrived_at(self.path[0]):
            self.path.popleft()

        if len(self.path) == 0:
            self.knowledge.update_status(Status.ARRIVED)
        else:
            self.knowledge.update_status(Status.DRIVING)

    # get current destination
    def get_current_destination(self):
        status = self.knowledge.get_status()
        # if we are driving, then the current destination is next waypoint
        if status == Status.DRIVING:
            # TODO: Take into account traffic lights and other cars
            return self.path[0]
        if status == Status.ARRIVED:
            return self.knowledge.get_location()
        if status == Status.HEALING:
            # TODO: Implement crash handling. Probably needs to be done by following waypoint list to exit the crash site.
            # Afterwards needs to remake the path.
            return self.knowledge.get_location()
        if status == Status.CRASHED:
            # TODO: implement function for crash handling, should provide map of wayoints to move towards to for exiting crash state.
            # You should use separate waypoint list for that, to not mess with the original path.
            return self.knowledge.get_location()
        # otherwise destination is same as current position
        return self.knowledge.get_location()

    # Create a path of waypoints from the current location to the current destination
    def build_path(self, source, destination):
        def find_next(current, destination):
            right_distance = math.inf
            left_distance = math.inf
            found = None
            min_distance = math.inf
            nexts = current.next(10)

            for point in nexts:
                next_nexts = point.next(20)
                for next_point in next_nexts:
                    next_dist = destination.distance(next_point.transform.location)
                    if next_dist < min_distance:
                        min_distance = next_dist
                        found = point

                if found.lane_change == carla.LaneChange.Right or point.lane_change == carla.LaneChange.Both:
                    right_point = self.get_waypoint(found.get_right_lane().transform.location)
                    right_distance = right_point.transform.location.distance(destination_wp)
                if found.lane_change == carla.LaneChange.Left or found.lane_change == carla.LaneChange.Both:
                    left_point = self.get_waypoint(found.get_left_lane().transform.location)
                    left_distance = left_point.transform.location.distance(destination_wp)

                if right_distance < min_distance and right_distance < left_distance:
                    found = right_point
                    min_distance = right_distance
                elif left_distance < min_distance and left_distance < right_distance:
                    found = left_point
                    min_distance = left_distance

            return found if found is not None else self.get_waypoint(destination)

        destination_wp = self.get_waypoint(destination).transform.location
        source = self.get_waypoint(source.location).transform

        self.path = deque([])
        next_point = self.get_waypoint(source.location)
        self.path.append(source.location)

        while destination_wp.distance(next_point.transform.location) > 5:
            next_point = find_next(next_point, destination_wp)
            self.path.append(next_point.transform.location)
        self.path.append(destination_wp)
#        self.draw_debug_path(self.path)
        return self.path

    def draw_debug_path(self, path):
        debug_markers_lifetime = 20.0
        self.world.debug.draw_string(path[0], "START", life_time=debug_markers_lifetime)
        for point in path:
            color = carla.Color(r=0,b=255,g=0) if self.get_waypoint(point).is_intersection else carla.Color(r=255,b=0,g=0)
            self.world.debug.draw_point(point + carla.Location(y=0.5), 0.05, life_time=debug_markers_lifetime, color=color)
        self.world.debug.draw_string(path[len(path) - 1], "END", life_time=debug_markers_lifetime)

    def get_waypoint(self, location):
        return self.map.get_waypoint(location)