#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String, Float64, Float32MultiArray
from duckietown_msgs.msg import Twist2DStamped, LanePose
from geometry_msgs.msg import Pose2D
import skfuzzy as fuzzy
import skfuzzy.control as ctrl
import numpy as np
from fuzzy import FuzzyControl, FuzzyControlSlow

class lane_controller(object):
    """fuzzy control version of lane controller"""
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None

        self.pub_counter = 0

        # Initialize buffers
        self.last_timestamp = 0

        self.omega_bar = 0.2

        self.obj_type = None
        self.obj_x = 0
        self.obj_y = 0

        self.delay_step = 50
        self.delay_buffer = 0
        self.delay_trigger = False

        ctl_sys = FuzzyControl()
        self.sim = ctl_sys.get_ctl()
        self.sim_slow = ctl_sys.get_ctl_slow()

        # Setup parameters
        self.setGains()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_debug = rospy.Publisher("~debug", Pose2D, queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        self.sub_obj = rospy.Subscriber("~objects", Float32MultiArray, self.cbUpdateObj, queue_size=1)
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbUpdateObj(self, msg):
        obj_state = msg.data
        if len(obj_state) != 0:
            self.obj_type = obj_state[0]
            self.obj_x = obj_state[9]
            self.obj_y = obj_state[10]
        else:
            self.obj_type = None

        print self.obj_type, self.obj_x, self.obj_y

    def setGains(self):
        v_bar = 0.5 # nominal speed, 0.5m/s
        k_theta = -2.0
        k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        d_offset = 0.0

        self.v_bar = self.setupParameter("~v_bar",v_bar) # Linear velocity
        # FIXME: AC aug'17: are these inverted?
        self.k_d = self.setupParameter("~k_d",k_theta) # P gain for theta
        self.k_theta = self.setupParameter("~k_theta",k_d) # P gain for d
        self.d_thres = self.setupParameter("~d_thres",theta_thres) # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres",d_thres) # Maximum desire theta
        self.d_offset = self.setupParameter("~d_offset",d_offset) # a configurable offset from the lane position

    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")

        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres, self.d_offset)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres, d_offset)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            rospy.loginfo("old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_old))
            rospy.loginfo("new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_new))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.theta_thres = theta_thres
            self.d_offset = d_offset


    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self, car_cmd_msg):

        #wheels_cmd_msg = WheelsCmdStamped()
        #wheels_cmd_msg.header.stamp = stamp
        #speed_gain = 1.0
        #steer_gain = 0.5
        #vel_left = (speed_gain*speed - steer_gain*steering)
        #vel_right = (speed_gain*speed + steer_gain*steering)
        #wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        #wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)

        self.pub_car_cmd.publish(car_cmd_msg)
        #self.pub_wheels_cmd.publish(wheels_cmd_msg)

    def cbPose(self, lane_pose_msg):

        self.lane_reading = lane_pose_msg

        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - self.lane_reading.header.stamp

        # delay from taking the image until now in seconds
        image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs/1e9

        # Calculating time delay since last frame of image
        if self.last_timestamp == 0:
            delay_time = 100
        else:
            delay_time = (timestamp_now - self.last_timestamp)
            delay_time = delay_time.secs

        cross_track_err = lane_pose_msg.d - self.d_offset
        heading_err = lane_pose_msg.phi

        if math.fabs(cross_track_err) > self.d_thres:
            cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres
        #car_control_msg.omega =  self.k_d * cross_track_err + self.k_theta * heading_err #*self.steer_gain #Right stick H-axis. Right is negative

        if self.obj_type is None and self.delay_trigger==False:
            # No object detected
            self.sim.input['lane_error'] = cross_track_err
            self.sim.input['head_error'] = heading_err
            self.sim.compute()
            ctl_omega = self.sim.output['out_omega']
            ctl_v = self.sim.output['out_v']
            #print ctl_v, ctl_omega
        else:
            if self.obj_type is not None:
                self.delay_trigger = True
                self.delay_buffer = self.delay_step
            else:
                self.delay_buffer -= 1
                if self.delay_buffer == 0:
                    self.delay_trigger = False
            # Detect other cars
            self.sim_slow.input['lane_error'] = cross_track_err
            self.sim_slow.input['head_error'] = heading_err
            self.sim_slow.compute()
            ctl_omega = self.sim_slow.output['out_omega']
            ctl_v = self.sim_slow.output['out_v']
            ctl_v = 0.0
            ctl_omega = 0.0

        # publish car command
        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = ctl_v
        car_control_msg.omega = ctl_omega
        self.publishCmd(car_control_msg)

        # Update timestamp buffer
        self.last_timestamp = rospy.Time.now()
        
        # debuging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "lane_controller publish"
        #     print car_control_msg

if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
