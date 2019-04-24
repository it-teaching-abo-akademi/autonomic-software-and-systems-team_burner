#!/usr/bin/env python

import glob
import os
import sys
import math
import weakref

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


# Monitor is responsible for reading the data from the sensors and telling it to the knowledge
# TODO: Implement other sensors (lidar and depth sensors mainly)
class Monitor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        weak_self = weakref.ref(self)

        # Initialize all defined memory variables
        self.update(0)

        world = self.vehicle.get_world()

        # Set up lane detector
        lane_sensor = world.get_blueprint_library().find('sensor.other.lane_detector')
        self.lane_detector = world.spawn_actor(lane_sensor, carla.Transform(), attach_to=self.vehicle)
        self.lane_detector.listen(lambda event: Monitor._on_invasion(weak_self, event))

        # Set up collision sensor
        collision_sensor = world.get_blueprint_library().find('sensor.other.collision')
        self.collision_detector = world.spawn_actor(collision_sensor, carla.Transform(), attach_to=self.vehicle)
        self.collision_detector.listen(lambda event: self._on_collision(weak_self, event))

        # Set up a camera to enable rendering a first person perspective view, and to be able to do object recognition
        # if the need arises.
        camera_rgb_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_rgb_blueprint.set_attribute('image_size_x', '960')
        camera_rgb_blueprint.set_attribute('image_size_y', '540')
        camera_rgb_blueprint.set_attribute('fov', '110')
        camera_rgb_blueprint.set_attribute('sensor_tick', '0.1')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera_rgb = world.spawn_actor(camera_rgb_blueprint, camera_transform, attach_to=self.vehicle)
        self.camera_rgb.listen(lambda image: self.knowledge.update_data('rgb_camera', image))

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        # Update the position of vehicle into knowledge
        location = self.vehicle.get_transform().location
        destination = self.knowledge.get_current_destination()
        distance = location.distance(destination)

        self.knowledge.update_data('location', location)
        self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)
        self.knowledge.update_data('velocity', self.vehicle.get_velocity()) # This is a vector that will be processed in the analyzer
        self.knowledge.update_data('distance', distance)

        self.knowledge.update_data('speed_limit', self.vehicle.get_speed_limit())

        # at_lights is true iff the vehicle is at a traffic light that is red or yellow
        self.knowledge.update_data('at_lights', self.vehicle.is_at_traffic_light() and
                                   str(self.vehicle.get_traffic_light_state()) == 'Red' or
                                   str(self.vehicle.get_traffic_light_state()) == 'Red' or
                                   str(self.vehicle.get_traffic_light_state()) == 'Yellow')

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        self.knowledge.update_data('lane_invasion', True)
        if not self:
            return

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return


# Analyser is responsible for parsing all the data that the knowledge has received from Monitor and turning it into something usable
# TODO: During the update step parse the data inside knowledge into information that could be used by planner to plan the route
class Analyser(object):
    def __init__(self, knowledge):
        self.knowledge = knowledge

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        # Get the 2D-distances to the current destination
        distance_x, distance_y = self.calculate_XY_distances()

        # Calculate the 2D-direction to the current destination, and the difference to our current direction
        heading = math.degrees(math.atan2(distance_y, distance_x))
        heading_diff = self.calculate_heading_diff(heading, self.knowledge.retrieve_data('rotation').yaw)

        self.knowledge.update_data('distance_x', distance_x)
        self.knowledge.update_data('distance_y', distance_y)
        self.knowledge.update_data('heading', heading)
        self.knowledge.update_data('heading_diff', heading_diff)
        self.knowledge.update_data('speed', self.velocity_to_speed(self.knowledge.retrieve_data('velocity')))
        return

    def velocity_to_speed(self, velocity):
        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) * 3.6

    def calculate_XY_distances(self):
        location = self.knowledge.retrieve_data('location')
        destination = self.knowledge.get_current_destination()
        distance_x = destination.x - location.x
        distance_y = destination.y - location.y
        return distance_x, distance_y

    def calculate_heading_diff(self, heading, rotation):
        abs_heading = math.fabs(heading)
        abs_rotation = math.fabs(rotation)
        heading_sign = 0 if heading == 0 else heading / abs_heading
        rotation_sign = 0 if rotation == 0 else rotation / abs_rotation

        if heading_sign == rotation_sign:
            diff = heading - rotation
        else:
            diff = abs_heading + abs_rotation

        if diff > 180:
            diff = 360 - diff

        diff_treshold = 5

        distance_y = self.knowledge.retrieve_data('distance_y')
        distance_x = self.knowledge.retrieve_data('distance_x')
        if min(abs(distance_y), abs(distance_x)) < diff_treshold:
            diff = 0

        return diff
