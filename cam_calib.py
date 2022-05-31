#!/usr/bin/env python

# Copyright (c) 2022 omitted

"""
RGB camera calibration example
"""

# import glob
# import os
# import sys
import carla

# import rospy
# from carla_msgs.msg import CarlaWorldInfo
import math
import numpy as np

class spawn_intersection_vehicles():
    """
    calibration of carla camera
    """
    def __init__(self):
        # self.host = "localhost"
        # self.port = 2000
        self.timeout = 10
        #self.client = None
        self.client = carla.Client("localhost",2000)
        self.world = self.client.get_world()

        self.actor_list=[]

    def spawn_static_vehicles(self):
        """
        6 spawn points found at the intersection, set z=0.5 to avoid collision
        [x,y,z,roll,pitch,yaw]
        [-6, 126, 0.5,0,0,90] [-9, 135, 0.5,0,0,180] [1, 130, 0.5,0,0,180]
        [-1,135,0.5,0,0,45] [5,140,0.5,0,0,270] [8,127, 0.5, 0,0,180]
      
        """
        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.find("vehicle.tesla.model3")
        vehicle1=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=-6, y=126, z=0.5),carla.Rotation(yaw=90)))
        self.actor_list.append(vehicle1)
        vehicle2=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=-9, y=135, z=0.5)))
        self.actor_list.append(vehicle2)
        vehicle3=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=1, y=130, z=0.5),carla.Rotation(yaw=180)))
        self.actor_list.append(vehicle3)
        vehicle4=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=-1, y=135, z=0.5),carla.Rotation(yaw=45)))    
        self.actor_list.append(vehicle4)    
        vehicle5=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=5, y=140, z=0.5),carla.Rotation(yaw=270)))
        self.actor_list.append(vehicle5)
        vehicle6=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=8, y=127, z=0.5),carla.Rotation(yaw=180)))
        self.actor_list.append(vehicle6)
        vehicle7=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=4, y=134, z=0.5),carla.Rotation(yaw=180)))  
        self.actor_list.append(vehicle7)
        vehicle8=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=-6, y=138, z=0.5),carla.Rotation(yaw=90))) 
        self.actor_list.append(vehicle8)
        vehicle9=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=-8, y=130, z=0.5),carla.Rotation(yaw=180)))
        self.actor_list.append(vehicle9)   
        vehicle10=self.world.spawn_actor(
            blueprint=vehicle_bp,
            transform=carla.Transform(carla.Location(x=8, y=136, z=0.5)))
        self.actor_list.append(vehicle10) 

    def destroy_static_vehicles(self):
        if self.actor_list is not None:
            for actor in self.actor_list:
                actor.destroy()
        # self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])

class carla_camera_calibration():
    """
    calibration of carla camera
    """
    def _init_(self,image_w=800,image_h=600,image_fov=110):
        self.image_w = image_w
        self.image_h=image_h
        self.image_fov=image_fov

    def projection_matrix(self,image_w,image_h,image_fov):
        #K intrinsic
        K = np.identity(3) 
        f=image_w / (2.0 * math.tan(float(image_fov) * math.pi / 360.0))
        K[0, 0] = K[1, 1] = f
        K[0, 2] = image_w/2.0
        K[1, 2] = image_h/2.0
        self.K=K

    def points_3D_world(self):
        #assume z is 0
        #[3, number_of_points] input points
        points_3d=np.array([[-6, 126,0],[-9,135,0],[1,130,0],[-1,135,0],[5,140,0],[8,127,0],[4,134,0],[-6,138,0],[-8,130,0],[8,136,0]]).T
        # Add an extra 1.0 at the end of each 3d point so it becomes of shape (4, number_of_points) and it can be multiplied by a (4, 4) matrix.
        points_3d = np.r_[
                points_3d, [np.ones(points_3d.shape[1])]]
        self.world_points=points_3d

    def camera_coordinates(self,whichbound):
        if whichbound=="south":
            camera = carla.Transform(carla.Location(x=5, y=121.7, z=6.5), carla.Rotation(pitch=0,yaw=90,roll=0))
        if whichbound=="north":
            camera = carla.Transform(carla.Location(x=-8, y=143, z=6.5), carla.Rotation(pitch=0,yaw=270,roll=0))
        if whichbound=="west":
            camera = carla.Transform(carla.Location(x=14, y=135, z=6.5), carla.Rotation(pitch=0,yaw=180,roll=0))
        if whichbound=="east":
            camera = carla.Transform(carla.Location(x=-17, y=130, z=6.5), carla.Rotation(pitch=0,yaw=0,roll=0))
        return camera

    def world_to_camera_matrix(self,whichbound):
        #whichbound specifies "south","north","west","east"
        #get cameras coordinates
        camera=self.camera_coordinates(whichbound)

        # This (4, 4) matrix transforms the points from world to camera coordinates
        world_2_camera = np.array(camera.get_inverse_matrix())
        return world_2_camera
       

    def points_world_to_camera(self,whichbound,image_w,image_h,image_fov):
        # Transform the points from world space to camera space.
        self.points_3D_world()
        world_2_camera=self.world_to_camera_matrix(whichbound)
        camera_points= np.dot(world_2_camera, self.world_points)

        # New we must change from UE4's coordinate system to an "standard"
        # camera coordinate system (the same used by OpenCV):

        # ^ z                       . z
        # |                        /
        # |              to:      +-------> x
        # | . x                   |
        # |/                      |
        # +-------> y             v y

        # This can be achieved by multiplying by the following matrix:
        # [[ 0,  1,  0 ],
        #  [ 0,  0, -1 ],
        #  [ 1,  0,  0 ]]

        # Or, in this case, is the same as swapping:
        # (x, y ,z) -> (y, -z, x)
        point_in_camera_coords = np.array([
            camera_points[1],
            camera_points[2] * -1,
            camera_points[0]])

        # Finally we can use our K matrix to do the actual 3D -> 2D.
        self.projection_matrix(image_w,image_h,image_fov)
        points_2d = np.dot(self.K, point_in_camera_coords)

        # Remember to normalize the x, y values by the 3rd value.
        points_2d = np.array([
            points_2d[0, :] / points_2d[2, :],
            points_2d[1, :] / points_2d[2, :],
            points_2d[2, :]])

        return points_2d


# if __name__ == '__main__':

#     main()
# calib=cam_calib.carla_camera_calibration()
# north_2d=calib.points_world_to_camera("north",image_w=800,image_h=600,image_fov=110)

