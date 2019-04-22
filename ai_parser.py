#!/usr/bin/env python

import glob
import os
import sys

import math

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import weakref
import carla
import numpy as np
import cv2
import ai_knowledge as data


# Monitor is responsible for reading the data from the sensors and telling it to the knowledge
# TODO: Implement other sensors (lidar and depth sensors mainly)
# TODO: Use carla API to read whether car is at traffic lights and their status, update it into knowledge
class Monitor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        weak_self = weakref.ref(self)

        location = self.vehicle.get_transform().location
        destination = self.knowledge.get_current_destination()
        distance = location.distance(destination)

        self.knowledge.update_data('location', location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)
        self.knowledge.update_data('velocity', self.vehicle.get_velocity())
        self.knowledge.update_data('distance', distance)
        world = self.vehicle.get_world()

        bp = world.get_blueprint_library().find('sensor.other.lane_detector')
        self.lane_detector = world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
        self.lane_detector.listen(lambda event: Monitor._on_invasion(weak_self, event))

        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
        self.sensor.listen(lambda event: self._on_collision(weak_self, event))

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        # Update the position of vehicle into knowledge
        location = self.vehicle.get_transform().location
        destination = self.knowledge.get_current_destination()
        distance = location.distance(destination)

        self.knowledge.update_data('location', location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)
        self.knowledge.update_data('velocity', self.vehicle.get_velocity())
        self.knowledge.update_data('distance', distance)

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        print("################################################################# LANE INVASION")
        self.knowledge.update_data('lane_invasion', event.crossed_lane_markings)

    @staticmethod
    # Copy-&-pasted from manual_control.py
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        print("################################################################# COLLISION")
        self.vehicle.destroy()  # Temporarily for getting crashed cars out of the way when tuning steering

# Analyser is responsible for parsing all the data that the knowledge has received from Monitor and turning it into something usable
# TODO: During the update step parse the data inside knowledge into information that could be used by planner to plan the route
class Analyser(object):
    def __init__(self, knowledge):
        self.knowledge = knowledge

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        distance_x, distance_y = self.calculate_XY_distances()
        self.knowledge.update_data('distance_x', distance_x)
        self.knowledge.update_data('distance_y', distance_y)
        self.knowledge.update_data('heading', math.degrees(math.atan2(distance_y, distance_x)))
        self.knowledge.update_data('heading_diff', self.calculate_heading_diff(self.knowledge.retrieve_data('heading'), self.knowledge.retrieve_data('rotation').yaw))
        self.knowledge.update_data('speed', self.velocity_to_speed(self.knowledge.retrieve_data('velocity')))
        return

    def velocity_to_speed(self, velocity):
        speed = np.hypot(velocity.x, velocity.y)
        return speed * 3.6

    def calculate_XY_distances(self):
        location = self.knowledge.retrieve_data('location')
        destination = self.knowledge.get_current_destination()
        distance_x = destination.x - location.x
        distance_y = destination.y - location.y
        return distance_x, distance_y

    def calculate_heading_diff(self, heading, rotation):
        abs_heading = math.fabs(heading)
        abs_rotation = math.fabs(rotation)
        heading_sign = bool(heading / abs_heading)
        rotation_sign = bool(rotation / abs_rotation)

        if heading_sign == rotation_sign:
            diff = heading - rotation
        else:
            diff = abs_heading + abs_rotation

        if diff > 180:
            diff = 360 - diff

        if abs(self.knowledge.retrieve_data('distance_y')) < 3 or abs(self.knowledge.retrieve_data('distance_x')) < 3:
            diff = 0

        return diff

