#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Twist

# Imports message type
from std_msgs.msg import String 

# Define callback function
def callback(msg):
    v = msg.v
    omega = msg.omega
    car_cmd = Twist()
    car_cmd.linear.x = 0.4*v 
    car_cmd.angular.z = omega*0.2
    publisher.publish(car_cmd)

# Initialize the node with rospy
rospy.init_node('cmd_inter_node', anonymous=False)

# Create subscriber
subscriber = rospy.Subscriber("~cmd", Twist2DStamped, callback)

# Create publisher
publisher = rospy.Publisher("~cmd_vel", Twist,queue_size=1)

# Runs continuously until interrupted
rospy.spin() 
