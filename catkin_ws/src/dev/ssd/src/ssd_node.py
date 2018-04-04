#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from sensor_msgs.msg import CompressedImage
from duckietown_utils.bag_logs import numpy_from_ros_compressed
import numpy as np
from SSD import ssdDetector
import cv2
import time
from duckietown_msgs.msg import BoolStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SsdDetector_node(object):
    def __init__(self):
        self.active = False
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        
        self.detector = self.detector = ssdDetector()
        self.status = "init"
        # Setup publishers
        self.pub_detect = rospy.Publisher("~detect_image",Image, queue_size=1)
        # Setup subscriber
        self.sub_cam = rospy.Subscriber("/duckiebot0/camera_node/image/compressed", CompressedImage, self.camera_callback)
        self.sub_switch = rospy.Subscriber('/duckiebot0/new_joy_mapper_node/detect', BoolStamped, self.switch_callback)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.2)
        # Create a timer that calls the infer_callback function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.infer_callback)
        
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    
    def switch_callback(self, msg):
        self.active = msg.data

    def camera_callback(self,msg):
        if not self.active:
            return

        #float_time = msg.header.stamp.to_sec()
        
        rgb = numpy_from_ros_compressed(msg)
        rgb[:, :, [0, 2]] = rgb[:, :, [2, 0]]
        self.status = "default"
        self.rgb = rgb

        #ospy.loginfo("[%s] get image data" %(self.node_name))

    def infer_callback(self,event):
        if not self.active:
            return
        # infering
        if self.status == "default":
            detect_img = self.detector.detect(self.rgb)
            # image_msg = self.bridge.cv2_to_imgmsg(detect_img)
            #image_msg = Image()
            #image_msg.data = detect_img
            #image_msg.header.stamp = rospy.Time.now()
            #self.pub_detect.publish(image_msg)

    def on_shutdown(self):
        self.detector.onShutdown()
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('ssd_node', anonymous=False)

    # Create the NodeName object
    node = SsdDetector_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()