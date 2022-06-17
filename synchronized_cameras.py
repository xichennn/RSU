#!/usr/bin/env python

import os
import sys
import carla
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import math

from queue import Queue
from queue import Empty
from PIL import Image
import csv

IM_WIDTH = 800
IM_HEIGHT = 600
FOV=110
ROS_VERSION=1


# Sensor callback.
# This is where you receive the sensor data and
# process it as you liked and the important part is that,
# at the end, it should include an element into the sensor queue.
def sensor_callback(sensor_data, sensor_queue, sensor_name):
    # Do stuff with the sensor_data data like save it to disk
    # Then you just need to add to the queue
    sensor_queue.put((sensor_data.frame, sensor_name,sensor_data.raw_data))
def save_vehicle_transform(world, vehicle):
    snapshot = world.get_snapshot()
    print("%d  %+8.03f %+8.03f %+8.03f" %
            # "%d %06.03f %+8.03f %+8.03f %+8.03f %+8.03f %+8.03f %+8.03f %+8.03f %+8.03f %+8.03f" 
            (snapshot.frame, \
            # snapshot.timestamp.elapsed_seconds, \
            # vehicle.get_acceleration().x, vehicle.get_acceleration().y, vehicle.get_acceleration().z, \
            # vehicle.get_velocity().x, vehicle.get_velocity().y, vehicle.get_velocity().z, \
            vehicle.get_location().x, vehicle.get_location().y, vehicle.get_location().z))

def main():

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    #get vehicle transformation
    actor_list=world.get_actors()
    vehicle_list=actor_list.filter("vehicle.*")
    for vehicle in vehicle_list:
        if vehicle.attributes['role_name'] == "hero1":
            player=vehicle
   

    try:
        # We need to save the settings to be able to recover them at the end
        # of the script to leave the server in the same state that we found it.
        original_settings = world.get_settings()
        settings = world.get_settings()
        # We set CARLA syncronous mode
        settings.fixed_delta_seconds = 0.2
        settings.synchronous_mode = True
        world.apply_settings(settings)
        # We create the sensor queue in which we keep track of the information
        # already received. This structure is thread safe and can be
        # accessed by all the sensors callback concurrently without problem.
        sensor_queue = Queue()
        # We create all the sensors and keep them in a list for convenience.
        actor_list = []

        #bluprints for sensors
        blueprint_library = world.get_blueprint_library()
    
        # https://carla.readthedocs.io/en/latest/cameras_and_sensors
        # get the blueprint for this sensor
        blueprint = blueprint_library.find('sensor.camera.rgb')
        # configure image dimensions
        blueprint.set_attribute('image_size_x', '800')
        blueprint.set_attribute('image_size_y', '600')
        blueprint.set_attribute('fov', '110')

        # Adjust sensor location
        spawn_point_south = carla.Transform(carla.Location(x=5, y=121.7, z=6.5), carla.Rotation(pitch=0,yaw=90,roll=0))
        spawn_point_north = carla.Transform(carla.Location(x=-8, y=143, z=6.5), carla.Rotation(pitch=0,yaw=270,roll=0))
        spawn_point_west = carla.Transform(carla.Location(x=14, y=135, z=6.5), carla.Rotation(pitch=0,yaw=180,roll=0))
        spawn_point_east = carla.Transform(carla.Location(x=-17, y=130, z=6.5), carla.Rotation(pitch=0,yaw=0,roll=0))

        # spawn cameras
        camera_south = world.spawn_actor(blueprint, spawn_point_south)
        camera_north = world.spawn_actor(blueprint, spawn_point_north)
        camera_west = world.spawn_actor(blueprint, spawn_point_west)
        camera_east = world.spawn_actor(blueprint, spawn_point_east)

        # receive image data
        camera_south.listen(lambda data: sensor_callback(data, sensor_queue, "camera_south"))
        camera_north.listen(lambda data: sensor_callback(data, sensor_queue, "camera_north"))
        camera_west.listen(lambda data: sensor_callback(data, sensor_queue, "camera_west"))
        camera_east.listen(lambda data: sensor_callback(data, sensor_queue, "camera_east"))
        # add camera to list of actors
        actor_list.append(camera_south)
        actor_list.append(camera_north)
        actor_list.append(camera_west)
        actor_list.append(camera_east)

        #main loop
    
        while True:
            #tick the server
            world.tick()
            w_frame=world.get_snapshot().frame
            print("\nWorld's frame: %d" % w_frame)
            print("     Frame: %d  x: %+8.03f y: %+8.03f z: %+8.03f" %
                (w_frame,player.get_location().x, player.get_location().y, player.get_location().z)) 
            with open("out/transforms.csv","a",encoding="UTF8",newline="") as f:
                writer=csv.writer(f)
                writer.writerow([w_frame,player.get_location().x, player.get_location().y, player.get_location().z])
          

            # Now, we wait to the sensors data to be received.
            # As the queue is blocking, we will wait in the queue.get() methods
            # until all the information is processed and we continue with the next frame.
            # We include a timeout of 1.0 s (in the get method) and if some information is
            # not received in this time we continue.
            try:
                for _ in range(len(actor_list)):
                    s_frame = sensor_queue.get(True, 1.0)
                    print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))
                    im_array = np.copy(np.frombuffer(s_frame[2], dtype=np.dtype("uint8")))
                    im_array = np.reshape(im_array, (600, 800, 4))
                    im_array = im_array[:, :, :3][:, :, ::-1]
                    image = Image.fromarray(im_array)
                    image.save("out/%05d_%s.png" % (s_frame[0],s_frame[1]))
            except Empty:
                print("    Some of the sensor information is missed")


    finally:
        world.apply_settings(original_settings)
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
