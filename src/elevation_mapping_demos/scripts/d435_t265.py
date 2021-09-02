#!/usr/bin/env python

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros
import rospy
import lcm
from robot_location_t import robot_location_t

class D435_t265:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.pose_sub = rospy.Subscriber("/t265/odom/sample", Odometry, self.pose_callback)
        self.pose_pub = rospy.Publisher('/t265_pose', PoseWithCovarianceStamped, queue_size=100)
        # self.lcm_timer = rospy.Timer(rospy.Duration(0.01), self.lcm_timer_callback)
        self.pcs = PoseWithCovarianceStamped()
        self.lc = lcm.LCM()
        self.rl = robot_location_t()

    def pose_callback(self, pose):
        self.pcs.header = pose.header
        self.pcs.header.frame_id = "t265_pose"
        self.pcs.pose = pose.pose
        self.pose_pub.publish(self.pcs)

        self.rl.pos_vo[0] = pose.pose.pose.position.x
        self.rl.pos_vo[1] = pose.pose.pose.position.y
        self.rl.pos_vo[2] = pose.pose.pose.position.z
        self.rl.quat[0] = pose.pose.pose.orientation.w
        self.rl.quat[1] = pose.pose.pose.orientation.x
        self.rl.quat[2] = pose.pose.pose.orientation.y
        self.rl.quat[3] = pose.pose.pose.orientation.z
        self.lc.publish("robot_location", self.rl.encode())


    # def lcm_timer_callback(self, event=None):
    #     pass


if __name__ == '__main__':
    rospy.init_node("d435_t265")
    d435_t265 = D435_t265()
    rospy.spin()
