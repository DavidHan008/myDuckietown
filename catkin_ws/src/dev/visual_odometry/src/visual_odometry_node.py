#!/usr/bin/python
from visual_odometry.Frame import *
from visual_odometry.Camera import *
from visual_odometry.util import *
from visual_odometry.Map import *
import visual_odometry.OptFlow as OF
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from duckietown_utils.bag_logs import numpy_from_ros_compressed
import cv2

class VisualOdometsy():
    def __init__(self):
        # Init utils
        self.node_name = "visual_odometry"
        self.bridge = CvBridge()
        self.framerate = 30
        self.status = "init"
        self.ok = False
        self.map = Map()
        self.lossframe = 0

        # Optic-Flow debug switch
        self.optflow = True

        # Publishers
        #self.pub_rot = rospy.Publisher("~rotation", viz_odom_rot)
        #self.pub_tran = rospy.Publisher("~tranlation", viz_odom_tran, queue_size=1)
        self.pub_debug_rot = rospy.Publisher("~debug_rot", Float32MultiArray, queue_size=1)
        self.pub_debug_tran = rospy.Publisher("~debug_tran", Float32MultiArray, queue_size=1)
        # Subscribers
        self.sub_image = rospy.Subscriber("~image_raw", CompressedImage, self.cbImage, queue_size=1)
        # Timers
        # rospy.Timer(rospy.Duration.from_sec(1.0/self.framerate), self.mainLoop)

    def cbImage(self, image_msg):
        # cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        rgb = numpy_from_ros_compressed(image_msg)
        rgb[:, :, [0, 2]] = rgb[:, :, [2, 0]]
        self.cur_frame = Frame(rgb)
        #self.cur_frame.plotFeature()
        self.ok = True
        self.mainLoop()
    
    def mainLoop(self):
        if self.ok == False:
            return
        else:
            if self.status == "init":
                self.map.pushFrame(self.cur_frame)
                self.status = "default"
                return
            else:
                #print len(self.cur_frame.keypoints)    
                rospy.loginfo("[%s]: current feature number: [%s]" %(self.node_name, len(self.cur_frame.keypoints)))
                ref_frame = self.map.popFrame()
                try:
                    # Read current frame
                    cur_frame = self.cur_frame
                    #ref_frame = self.map.popFrame()

                    # Calculate matched frame
                    matches = matchBF(cur_frame.descriptors, ref_frame.descriptors)
                    cur_kp, ref_kp = convertKeypoint(cur_frame.keypoints, ref_frame.keypoints, matches)
                    K = np.array(cur_frame.getK()).reshape([3, 3])
                    #print len(cur_kp), len(ref_kp)
                    rospy.loginfo("[%s]: current matched number: [%s]" %(self.node_name, len(ref_kp)))
                    focal, pp = cur_frame.getFocalPp()
                    #print focal, pp
                    
                    # Find Essential Matrix
                    #E, _ = cv2.findEssentialMat(cur_kp, ref_kp, K, cv2.RANSAC, threshould=1.0)
                    #E, _ = cv2.findEssentialMat(cur_kp, ref_kp, focal[0], pp, cv2.RANSAC, threshold=2.0)
                    E, _ = cv2.findEssentialMat(ref_kp, cur_kp, focal[0], pp, cv2.RANSAC, threshold=0.4, prob=0.999)

                    # Recover [R, t] from Essential matrix
                    #_, R, t, mask = cv2.recoverPose(E, cur_kp, ref_kp, K, 2.0)
                    _, R, t, mask = cv2.recoverPose(E, ref_kp, cur_kp, K, 0.4)

                    dR = R - np.eye(3, dtype=float)
                    d = np.sum(dR * dR)
                    print "dR: ", d
                    if d > 5:
                        #self.map.pushFrame(cur_frame)
                        raise Exception()
                    dt = t - np.array([0,0,0])
                    d = np.sum(dt * dt)
                    print "dt: ", d
                    if d > 5:
                        raise Exception()

                    pre_R, pre_t = ref_frame.getRt()
                    cur_R = np.dot(pre_R, R)
                    cur_t = pre_t + t
                    #print cur_R[0, :]
                    #print cur_t

                    # Publish Rotation results
                    #rot_msg = viz_odom_rot()
                    debug_rot = Float32MultiArray()
                    #rot_msg.header.stamp = rospy.Time.now()
                    #rot_msg.rotation = cur_R[0, :]
                    debug_rot.data = cur_R[1, :]
                    #self.pub_rot.publish(rot_msg)
                    self.pub_debug_rot.publish(debug_rot)

                    # Publish Translation results
                    debug_tran = Float32MultiArray()
                    debug_tran.data = cur_t
                    self.pub_debug_tran.publish(debug_tran)
                    #tran_msg = viz_odom_tran()
                    #tran_msg.header.stamp = rospy.Time.now()
                    #tran_msg.translation = cur_t
                    #self.pub_tran.publish(tran_msg)

                    cur_frame.setRt(cur_R, cur_t)
                    self.lossframe = 0
                    self.map.pushFrame(cur_frame)
                except:
                    self.map.pushFrame(ref_frame)
                    self.lossframe = self.lossframe + 1
                    print "loss frame ", self.lossframe

    def plotFeature(self, _event):
        if self.status == "default":
            #print self.cur_frame.keypoints
            self.map.pushFrame(self.cur_frame)
            print self.map.popFrame().keypoints

if __name__ == '__main__':
    rospy.init_node('visual_odometry',anonymous=False)
    vo = VisualOdometsy()
    rospy.spin()