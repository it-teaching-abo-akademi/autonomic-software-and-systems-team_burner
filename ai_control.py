#!/usr/bin/env python

import glob
import os
import sys
from collections import deque
import math

from ai_knowledge import Status
import pandas as pd
try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
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
        control.steer = heading_diff / (speed / 15 if speed != 0 else 2)

        # Calculate the difference between the current speed and the target speed, and adjust throttle accordingly.
        # Since different cars responsed differently to the same amount of throttle, this needs to be extended with
        # an adaptive feedback loop.
        # TODO: Adaptive feedback
        speed_diff = target_speed - speed
        if target_speed == 0:
            control.throttle = 0.0
            control.brake = 1.0
        elif speed_diff > 0:
            control.throttle = min((speed_diff - 5) / target_speed + 0.7, 1.0) - abs(control.steer / 2)
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
        location = self.knowledge.retrieve_data('location')

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
        self.path = deque([])
        destination_wp = carla.Location(x=destination.x, y=destination.y, z=destination.z)

        def find_next(current, destination, search_distance):
            found = None
            md = math.inf
            nexts = current.next(search_distance)
            for point in nexts:
                dist = destination.distance(point.transform.location)
                if dist < md:
                    md = dist
                    found = point
            return found

        current_wp = self.world.get_map().get_waypoint(source.location)
        while True:
            if 'next_point' in locals():
                previous_point = next_point
            next_point = find_next(current_wp, destination_wp, 5)

            self.path.append(next_point.transform.location)
            if destination_wp.distance(next_point.transform.location) <= 7.5:
                break
            else:
                current_wp = next_point

        self.path.append(destination_wp)

        double_step = deque([])
        double_step.append(self.path[0])
        previous = self.path[0]

        for point in self.path:
            current_point = self.get_waypoint(point)
            if current_point.transform.rotation.yaw != self.get_waypoint(previous).transform.rotation.yaw:
                if current_point.lane_width > 2.0 and not current_point.is_intersection:
                    perpend = math.radians(current_point.transform.rotation.yaw) + (math.pi / 2)
                    x = math.cos(perpend) * 0.5
                    y = math.sin(perpend) * 0.5
                    point += carla.Location(x=x, y=y)
                double_step.append(point)
                previous = point

        final_point = self.path[len(self.path) - 1]
        if double_step[len(double_step) - 1] != final_point:
            double_step.append(final_point)
        self.draw_debug_path(double_step)
        return double_step

    def draw_debug_path(self, path):
        debug_markers_lifetime = 20.0
        self.world.debug.draw_string(path[0], "START", life_time=debug_markers_lifetime)
        for point in path:
            color = carla.Color(r=0,b=255,g=0) if self.get_waypoint(point).is_intersection else carla.Color(r=255,b=0,g=0)
            self.world.debug.draw_string(point, str(self.get_waypoint(point).road_id), life_time=20.0, color=color)
        self.world.debug.draw_string(path[len(path) - 1], "END", life_time=debug_markers_lifetime)

    def get_waypoint(self, location):
        return self.map.get_waypoint(location)