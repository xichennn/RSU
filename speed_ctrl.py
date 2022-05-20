#!/usr/bin/env python
# license removed for brevity
import rospy
import sys

from autoware_msgs.msg import VehicleCmd
from autoware_msgs.msg import LaneArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import Lane
from v2x_node_class.msg import trajectory

import numpy as np
import matplotlib.pyplot as plt

import os
import pandas as pd

class speed_ctrl(object):

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.traj_pub = rospy.Publisher('/final_waypoints_ctrl', Lane, queue_size=0)

        # subscribed Topic
        self.subscriber = rospy.Subscriber('/v2x_node/v2x/v2x_trajectory_1', trajectory, self.traj_change_callback)
        self.subscriber = rospy.Subscriber('/final_waypoints', Lane, self.trajectory_callback)
    
     
        self.flag=0
        # self.distcp_pub = rospy.Publisher('/dist_cp', TwistStamped, queue_size=20)
        # self.distcpMsg=TwistStamped()
        # self.dist_cp=[]
        self.trajMsg=None#Lane()
        self.trajRSU=None#Lane()

    def trajectory_callback(self,msg):
        if self.flag==0:
            self.trajMsg = msg
            rospy.loginfo('using final waypoints')
            rospy.loginfo(self.trajMsg.waypoints[0].twist.twist.linear.x)
        # # if self.flag==1:
        #     shift=-3
        #     ratio=0.5
        #     for i in range(len(msg.waypoints)):
        #         # self.trajMsg.waypoints[i].pose.pose.position.x=shift+self.trajMsg.waypoints[i].pose.pose.position.x
        #         self.trajMsg.waypoints[i].twist.twist.linear.x=ratio*self.trajMsg.waypoints[i].twist.twist.linear.x
    def traj_change_callback(self, msg):   
        if msg.trajectory.waypoints[0]!=["[]"] and len(msg.trajectory.waypoints)!=0 :
            self.flag=1
            rsu_traj=msg.trajectory
            self.trajRSU =rsu_traj
            rospy.loginfo('RSU traj received')
            rospy.loginfo(self.trajRSU.waypoints[0].twist.twist.linear.x)   
        else:
            self.flag=0




    # def conflict_callback(self,msg):
    #     pose_x=msg.lanes[0].waypoints[0].pose.pose.position.x
    #     pose_y=msg.lanes[0].waypoints[0].pose.pose.position.y
    #     print pose_x
    #     ego_pos = np.array((pose_x,pose_y))
    #     conflict_point = np.array((5.47,-133.34))
    #     if pose_y > -133.34:
    #         dist = np.linalg.norm(ego_pos-conflict_point)
    #     else:
    #         dist = -np.linalg.norm(ego_pos-conflict_point)
    #     print dist
    #     if dist <30 and dist >0:
    #         self.flag=1
    #     else:
    #         self.flag=0
    #     self.dist_cp.append(dist)
    #     self.distcpMsg.twist.linear.x=dist
    #     self.distcpMsg.header.stamp=rospy.Time.now()
    #     #self.dist_cp_plot(msg.lanes[0])

    # def speed_ctrl_callback(self,msg):
        
    #     self.ctrlMsg = msg
    #     if self.flag==1:
    #         ratio=0.5
    #         self.ctrlMsg.twist_cmd.twist.linear.x=ratio*self.ctrlMsg.twist_cmd.twist.linear.x
    #         self.ctrlMsg.twist_cmd.twist.angular.z=0.5*self.ctrlMsg.twist_cmd.twist.angular.z
        
    #     #except Exception as err:
    #      #   print err


def main():
    rospy.init_node('speed_control')
    rospy.loginfo('speed control node started')
    speed = speed_ctrl()
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
       # rospy.loginfo("publishing image with error %s" % rospy.get_time())
        rospy.loginfo("publish:")
        if speed.trajRSU is not None:
            speed.traj_pub.publish(speed.trajRSU)
            rospy.loginfo(speed.trajRSU.waypoints[0].twist.twist.linear.x)
        elif speed.trajMsg is not None:
            speed.traj_pub.publish(speed.trajMsg)
            rospy.loginfo(speed.trajMsg.waypoints[0].twist.twist.linear.x)
        # rospy.loginfo("publish:")
        # if len(speed.trajMsg.waypoints) !=0:
        #     rospy.loginfo(speed.trajMsg.waypoints[0].twist.twist.linear.x)
        # if speed.distcpMsg is not None:
        #     speed.distcp_pub.publish(speed.distcpMsg)
        rate.sleep()


if __name__ == '__main__':
    main()


