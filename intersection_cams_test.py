#!/usr/bin/env python

import os
import sys
import carla
from geometry_msgs.msg import PoseStamped
import numpy 
import time
import math

import rospy
import logging
logging.basicConfig()
from tf.transformations import euler_from_quaternion
import tf
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
#from carla_ros_bridge.sensor import Sensor

IM_WIDTH = 800
IM_HEIGHT = 600
FOV=110
ROS_VERSION=1

def process_img(image):
    """
    convert the carla image to a numpy data array
    as input for the cv_bridge.cv2_to_imgmsg() function
    The RGB camera provides a 4-channel int8 color format (bgra).
    
    :param image: carla image object
    :type image: carla.Image
    :return tuple (numpy data array containing the image information, encoding)
    :rtype tuple(numpy.ndarray, string)
    """
    carla_image_data_array = numpy.ndarray(
            shape=(IM_HEIGHT, IM_WIDTH, 4),
            dtype=numpy.uint8, buffer=image.raw_data)
    # i = numpy.array(image.raw_data)
    # print(image.timestamp)
    # i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    # i3 = i2[:, :, :3]
    # cv2.imshow("", i3)
    # cv2.waitKey(1)
    #return i3/255.0
    return carla_image_data_array, 'bgra8'
def publish_img_south(image):
    """
    Function to transform the received carla camera data into a ROS image message
    """
    image_data_array, encoding=process_img(image)

    img_msg=CvBridge().cv2_to_imgmsg(image_data_array, encoding=encoding)
    img_msg.header.stamp=ros_timestamp(sec=image.timestamp, from_sec=True)
    image_south_pub.publish(img_msg)
    intersection_camera_info_pub.publish(build_camera_info())

def publish_img_north(image):
    image_data_array, encoding=process_img(image)
    # cv2.imshow("",image_data_array)
    # cv2.waitKey(1)
    img_msg=CvBridge().cv2_to_imgmsg(image_data_array, encoding=encoding)
    img_msg.header.stamp=ros_timestamp(sec=image.timestamp, from_sec=True)
    image_north_pub.publish(img_msg)
def publish_img_west(image):
    image_data_array, encoding=process_img(image)

    img_msg=CvBridge().cv2_to_imgmsg(image_data_array, encoding=encoding)
    img_msg.header.stamp=ros_timestamp(sec=image.timestamp, from_sec=True)
    image_west_pub.publish(img_msg)
def publish_img_east(image):
    image_data_array, encoding=process_img(image)

    img_msg=CvBridge().cv2_to_imgmsg(image_data_array, encoding=encoding)
    img_msg.header.stamp=ros_timestamp(sec=image.timestamp, from_sec=True)
    image_east_pub.publish(img_msg)
def ros_timestamp(sec=0, nsec=0, from_sec=False):
    if from_sec:
        return rospy.Time.from_sec(sec)
    return rospy.Time(int(sec), int(nsec))    
    
def build_camera_info():
    """
    Private function to compute camera info
    camera info doesn't change over time
    """
    camera_info = CameraInfo()
    # store info without header
    #camera_info.header = rospy.Time.now()
    camera_info.width = int(IM_WIDTH)
    camera_info.height = int(IM_HEIGHT)
    camera_info.distortion_model = 'plumb_bob'
    cx = camera_info.width / 2.0
    cy = camera_info.height / 2.0
    fx = camera_info.width / (
        2.0 * math.tan(float(FOV) * math.pi / 360.0))
    fy = fx
    if ROS_VERSION == 1:
        camera_info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    elif ROS_VERSION == 2:
        # pylint: disable=assigning-non-slot
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return camera_info

actor_list = []
try:
    rospy.init_node('intersection_cameras')
    rospy.loginfo('intersection camera node started')
    image_south_pub = rospy.Publisher('/image_south',Image, queue_size=10)
    image_north_pub = rospy.Publisher('/image_north',Image, queue_size=10)
    image_west_pub = rospy.Publisher('/image_west',Image, queue_size=10)
    image_east_pub = rospy.Publisher('/image_east',Image, queue_size=10)
    intersection_camera_info_pub = rospy.Publisher('/intersection_camera_info',CameraInfo, queue_size=10)

    rate = rospy.Rate(10) # 10hz
    
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    
    # https://carla.readthedocs.io/en/latest/cameras_and_sensors
    # get the blueprint for this sensor
    blueprint = blueprint_library.find('sensor.camera.rgb')
    # change the dimensions of the image
    blueprint.set_attribute('image_size_x', '800')
    blueprint.set_attribute('image_size_y', '600')
    blueprint.set_attribute('fov', '110')

    # Adjust sensor relative to vehicle
    spawn_point_south = carla.Transform(carla.Location(x=5, y=121.7, z=6.5), carla.Rotation(pitch=0,yaw=90,roll=0))
    spawn_point_north = carla.Transform(carla.Location(x=-8, y=143, z=6.5), carla.Rotation(pitch=0,yaw=270,roll=0))
    spawn_point_west = carla.Transform(carla.Location(x=14, y=135, z=6.5), carla.Rotation(pitch=0,yaw=180,roll=0))
    spawn_point_east = carla.Transform(carla.Location(x=-17, y=130, z=6.5), carla.Rotation(pitch=0,yaw=0,roll=0))

    # spawn cameras
    camera_south = world.spawn_actor(blueprint, spawn_point_south)
    camera_north = world.spawn_actor(blueprint, spawn_point_north)
    camera_west = world.spawn_actor(blueprint, spawn_point_west)
    camera_east = world.spawn_actor(blueprint, spawn_point_east)


    # add camera to list of actors
    actor_list.append(camera_south)
    actor_list.append(camera_north)
    actor_list.append(camera_west)
    actor_list.append(camera_east)

    # publish image data
    camera_south.listen(lambda data: publish_img_south(data))
    camera_north.listen(lambda data: publish_img_north(data))
    camera_west.listen(lambda data: publish_img_west(data))
    camera_east.listen(lambda data: publish_img_east(data))
    
    while not rospy.is_shutdown():
       # rospy.loginfo("publishing image with error %s" % rospy.get_time())
       # if img_south is not None:
        #    image_south_pub.publish(img_south)
            #im.image_pub.publish(MSG)
            #rospy.loginfo("publishing image with error")
        rate.sleep()
    #time.sleep(20)

finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
