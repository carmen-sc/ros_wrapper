#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import CameraInfo, CompressedImage


# a wrapper node which reads ROS topics from the Duckiebot stack and parses them to the simulator

class WrapperNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WrapperNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # define overarching things
        self.vehicle = os.getenv("VEHICLE_NAME")

        # define handling of wheel stuff
        self.wheel_action_sub = rospy.Subscriber(f'/{self.vehicle}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.gen_wheel_action)
        
        # placeholder for the action
        self.action = np.array([0.0, 0.0])
        self.updated = True

        # define handling of camera stuff
        self.cam_pub = rospy.Publisher(f'/{self.vehicle}/camera_node/image/compressed', CompressedImage, queue_size=10)
        self.cam_info_pub = rospy.Publisher(f'/{self.vehicle}/camera_node/camera_info', CameraInfo, queue_size=1)
             

    def gen_wheel_action(self, msg):
        vel_left = msg.vel_left
        vel_right = msg.vel_right
        self.action = np.array([vel_left, vel_right])

    def pub_img(self, observ):
        img_msg = CompressedImage()

        time = rospy.get_rostime()
        img_msg.header.stamp.secs = time.secs
        img_msg.header.stamp.nsecs = time.nsecs

        img_msg.format = "jpeg"
        contig = cv2.cvtColor(np.ascontiguousarray(observ), cv2.COLOR_BGR2RGB)
        img_msg.data = np.array(cv2.imencode(".jpg", contig)[1]).tostring()

        self.cam_pub.publish(img_msg)

    def pub_cam_info(self):
        self.cam_info_pub.publish(CameraInfo())


if __name__ == '__main__':
    # create the node
    node = WrapperNode(node_name='wrapper_node')
    # keep spinning
    rospy.spin()