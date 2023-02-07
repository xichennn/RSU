#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
#from derived_object_msgs.msg import ObjectArray
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge
import os
from calibration import Camera2DGroundModel
from calibration import MapModel
from detection import DetectorTorchvisionMaskRCNN
from tracking import Tracker
from background import KLTOpticalFlowTracker
from background import CameraShakeRectifier

from visualization import FrameVis


class Tracker_node(object):
    def __init__(self):

        '''Initialize tracker'''
        os.environ['KMP_DUPLICATE_LIB_OK']='True'
        np.seterr(divide='ignore', invalid='ignore')
        self.init_flag = True
        self.frame_id = 1

        cwd = os.getcwd()
        folder = f"{cwd}//data"
        prefix = f"northbound"
        postfix = f""
        detector = DetectorTorchvisionMaskRCNN(folder, prefix, postfix, pretrained=True)
        
        map_model = None

        mask_bg = cv2.imread(f"{folder}/north_mask.png")
        frame_bg = cv2.imread(f"{folder}/north_carla.png")
        frame_width = frame_bg.shape[1]
        frame_height = frame_bg.shape[0]
        print([frame_width,frame_height])
        klt_tracker = KLTOpticalFlowTracker(mask_bg)
        csr = CameraShakeRectifier(frame_bg, mask_bg)
        self.out_raw = cv2.VideoWriter('out_raw.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 20, (frame_width,frame_height))
        self.out_tracker = cv2.VideoWriter('out_tracker.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 20, (frame_width,frame_height))

        camera_model = Camera2DGroundModel()
        calibration_folder = f"{cwd}/data/calibration_2d/{prefix}"
        try:
            camera_model.load_calib_para(calibration_folder, prefix)
        except:
            try:
                points_xyz = np.genfromtxt('my_file.csv', delimiter=',')
                points_uv = np.genfromtxt('my_file.csv', delimiter=',') 
                camera_model.calib_ground(points_xyz, points_uv)
                camera_model.calib_camera(points_xyz, points_uv, frame_width, frame_width, f_init=2000)
                
            except:
                print("Calibration files not found")

        h_margin=15
        v_margin=10
        fps=30

        self.tracker = Tracker(map_model, klt_tracker, detector, camera_model,
                        frame_width, frame_height, h_margin, v_margin, fps)
        mask_bg_gray = cv2.cvtColor(mask_bg, cv2.COLOR_BGR2GRAY)
        mask_bg_th = mask_bg_gray > 0
        self.tracker.mask_bg = mask_bg_th
            
        # ROS stuff
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        # TODO topic name
        self.publisher = rospy.Publisher('vehicle_coordinates',Image, queue_size=10)

        # subscribed Topic
        # TODO topic name
        self.subscriber = rospy.Subscriber('image_north', Image, self.process_image_callback) 
        self.coord_msg = None
        #image_msg and image_array conversion
        self.bridge=CvBridge()


    def track(self,frame):
        if self.init_flag:       
            self.tracker.setup_first_frame(self.frame_id, frame)
            self.tracker.track_iteration_finish()
            self.init_flag = False
        else:
            # Remove the camera shake.
            # frame = csr.rectify(frame)

            self.tracker.track_iteration_online(self.frame_id, frame)
            self.tracker.track_iteration_finish()
        self.frame_id += 1

        string = ""
        coord_msg = []
        for vid, vehicle in self.tracker.current_vehicles.items():
            string = f"{string}{vid}: {np.around(vehicle.ss[0:2].T.ravel(), decimals=2)}/n"
            coord_msg.append(np.around(vehicle.ss[0:2].T.ravel()))
        print(coord_msg)
        return coord_msg

    def process_image_callback(self,frame):
        #convert sensor.image to ndarray
        img_ndarray = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        cv2.imshow("image_raw",img_ndarray)
        # cv2.imwrite(f"north_{self.frame_id}.png", img_ndarray)
        cv2.waitKey(1)
        self.coord_msg = self.track(img_ndarray)
        #self.out_raw.write(img_ndarray) 
        fvis = FrameVis()
        fvis.draw_bb3ds_on_frame(img_ndarray, self.tracker.instances)
        cv2.imshow("image_tracker",img_ndarray)
        self.out_tracker.write(img_ndarray)
        # cv2.imshow("window3",im_33)



def main():
    rospy.init_node('tracker')
    rospy.loginfo('tracker node started')
    tracker = Tracker_node()

    rate = rospy.Rate(10) # 20hz
    while not rospy.is_shutdown():
        if tracker.coord_msg is not None:
            pass
            #print(tracker.coord_msg)
            #tracker.publisher.publish(tracker.coord_msg)
            
        rate.sleep()

if __name__ == '__main__':
    main()
