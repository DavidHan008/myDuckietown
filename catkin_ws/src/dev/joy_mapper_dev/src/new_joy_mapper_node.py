#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int8, String
import time
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState

# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7, logitek = 8, left joy = 9, right joy = 10
class NewJoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.state = 'init'
        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()
        
        self.go_status = False
        self.pub_go= rospy.Publisher("~letsgo", BoolStamped, queue_size=1)
        self.pub_detect = rospy.Publisher("~detect", BoolStamped, queue_size=1)
        self.detect_status = False
        
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_mode = rospy.Subscriber('/duckiebot0/fsm_node/mode', FSMState, self.cbMode)
        self.forward = rospy.ServiceProxy('/duckiebot0/open_loop_intersection_control_node/turn_forward',Empty)
        self.button2command = {
             # 'a' is pressed
            # 0: 'CAR_SIGNAL_A',
            # 'b' is pressed
            1: "COORDINATION",
            # 'Y' is pressed
            # 3: 'DETECT',
            # 'X' is pressed
            2: 'Detect',
            # lb is pressed
            # 4: 'traffic_light_go',
            # rb is pressed
            # 5: 'traffic_light_stop',
            # logitek button is pressed
            # 8: 'test_all_1',
        }
        rospy.Timer(rospy.Duration.from_sec(0.1), self.cbForward)
        self.go_mode = "init"

    def cbForward(self, event):
        if self.go_mode == "default":
            go_msg = BoolStamped()
            go_msg.header.stamp = self.joy.header.stamp
            if self.go_status == True:
                go_msg.data = True
            else:
                go_msg.data = False
            self.pub_go.publish(go_msg)

    def cbMode(self, msg):
        self.state = msg.state

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.go_mode = "default"

    def publishControl(self):

        #for b, state in self.button2command.items():
        if self.joy.buttons[1] == 1:
                #if self.state == state:
                    #self.forward()
                    #go_msg = BoolStamped()
                    #go_msg.header.stamp = self.joy.header.stamp
                    #if self.go_status == False:
                    #    self.go_status = True
                    #if self.go_status == True:
                    #    self.go_status = False
                    #self.pub_go.publish(go_msg)
            self.go_status = not self.go_status
            rospy.loginfo("Publishing forward command")
        
        if self.joy.buttons[2] == 1:
            detect_msg = BoolStamped()
            detect_msg.header.stamp = self.joy.header.stamp
            if self.detect_status == False:
                detect_msg.data = False
            else:
                detect_msg.data = True
            self.detect_status = ~self.detect_status
            self.pub_detect.publish(detect_msg)
            rospy.loginfo("Publishing detect command")

if __name__ == "__main__":
    rospy.init_node("new_joy_mapper",anonymous=False)
    led_joy_mapper = NewJoyMapper()
    rospy.spin()##  
# 