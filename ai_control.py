#!/usr/bin/env python

import glob
import os
import sys
from collections import deque
from ai_knowledge import Status

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


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
        # control.hand_brake = False

        # Get the difference between the car's heading and the path to current destination and turn the wheel
        # accordingly. To stabilize the car a little, the analyzer has a rudimentary threshold under which it always
        # reports the difference as 0.
        # TODO: Combine the threshold calculator and this calculator in the planner, and refine it. The car still wobbles a lot.
        heading_diff = self.knowledge.retrieve_data('heading_diff')
        heading_sign = -1 if heading_diff < 0 else 1
        abs_heading_diff = min(abs(heading_diff), 10)

        control.steer = abs_heading_diff / 20.0
        control.steer = control.steer * heading_sign

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
        self.print_diagnostics()

    # Function to print messages to the console for development and debugging
    def print_diagnostics(self):
        current = self.vehicle.get_control()
        at_lights = self.knowledge.retrieve_data('at_lights')
        target_speed = self.knowledge.retrieve_data('target_speed')
        speed = self.knowledge.retrieve_data('speed')

        # world = self.vehicle.get_world()
        # m = world.get_map()
        # w = m.get_waypoint(self.knowledge.retrieve_data('location'))

        print("At lights: {}\t\tTarget speed: {:3.2f}\t\tSpeed: {:3.2f}\t\tThrottle: {:3.2f}  Brake: {:3.2f}".format(at_lights, target_speed, speed, current.throttle, current.brake))




# Planner is responsible for creating a plan for moving around
# In our case it creates a list of waypoints to follow so that vehicle arrives at destination
# Alternatively this can also provide a list of waypoints to try avoid crashing or 'uncrash' itself
class Planner(object):
    def __init__(self, knowledge):
        self.knowledge = knowledge
        self.path = deque([])

    # Create a map of waypoints to follow to the destination and save it
    def make_plan(self, source, destination):
        self.path = self.build_path(source, destination)
        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        at_lights = self.knowledge.retrieve_data('at_lights')
        distance_to_target = self.knowledge.retrieve_data('distance')

        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

        if at_lights or distance_to_target < 5:
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

    # TODO: Implementation
    def build_path(self, source, destination):
        self.path = deque([])
        self.path.append(destination)
        # TODO: create path of waypoints from source to
        return self.path
