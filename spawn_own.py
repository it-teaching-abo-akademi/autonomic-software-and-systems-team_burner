#!/usr/bin/env python

# Script for testing the AI. Shows the world in a first person perspective from a camera that's mounted on the car.
# The starting point and destination can be changed on line 78 (or somewhere in the vicinity, depending on the amount
# of code that's added or removed) that says: try_spawn_random_vehicle_at(spawn_points[2], spawn_points[0])
#

import glob
import os
import sys

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import numpy as np
import pygame
import custom_ai as ai

pygame.init()
camera = pygame.display.set_mode((640, 480), pygame.HWSURFACE | pygame.DOUBLEBUF)


def render(image):
    def draw_text(text):
        font = pygame.font.SysFont(None, 24)
        return font.render(text, True, (255, 0, 0), (255, 255, 255))

    if image:
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        camera.blit(surface, (0, 0))
        # text = draw_text('moo')
        # camera.blit(text, (10, 10))
        pygame.display.flip()


def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    try:
        world = client.get_world()
        blueprints = world.get_blueprint_library().filter('vehicle.*')

        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]

        def draw_forward_vector():
            start_transform = world.get_map().get_waypoint(actor_list[0].get_location()).transform
            start_location = start_transform.location
            f_vect = start_transform.get_forward_vector()
            raiser = carla.Location(y=0.5)
            line_end = start_location + carla.Location(x=f_vect.x * 5, y=f_vect.y * 5)
            world.debug.draw_line(start_location + raiser, line_end + raiser, thickness=0.3, life_time=10.0)

        def label_spawn_points():
            wps = world.get_map().get_spawn_points()
            for i in range(len(wps)):
                world.debug.draw_string(wps[i].location, str(i))

        # This is definition of a callback function that will be called when the autopilot arrives at destination
        def route_finished(autopilot):
            print("Vehicle arrived at destination")

        # Function to spawn new vehicles
        def try_spawn_random_vehicle_at(transform, destination):
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'autopilot')
            vehicle = world.try_spawn_actor(blueprint, transform)
            if vehicle is not None:
                actor_list.append(vehicle)
                # Instead of setting default autopilot, we create our own and append it to the list of autopilots
                autopilot = ai.Autopilot(vehicle)
                autopilot.set_destination(destination.location)
                # We also register callback to know when the vehicle has arrived at it's destination
                autopilot.set_route_finished_callback(route_finished)
                print('spawned %r at %s' % (vehicle.type_id, transform.location))
                return autopilot
            return False

        spawn_points = list(world.get_map().get_spawn_points())
        print('found %d spawn points.' % len(spawn_points))

        start = spawn_points[124]
        end = spawn_points[36]
        controller = try_spawn_random_vehicle_at(start, end)

        # Infinite loop to update car status
        while True:
            status = controller.update()
            render(controller.knowledge.retrieve_data('rgb_camera'))

            from pygame.locals import K_ESCAPE
            from pygame.locals import K_l
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    if event.key == K_l:
                        draw_forward_vector()
                    if event.key == K_ESCAPE:
                        return True
    finally:
        print('\ndestroying %d actors' % len(actor_list))
        client.apply_batch([carla.command.DestroyActor(x.id) for x in actor_list])
        pygame.quit()


main()
print('\ndone.')
sys.exit(0)
