#! /usr/bin/env python3

# this node is used to set the virtual leader's pose relativ to a given robot

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class SetLeaderPose():

    def __init__(self):
        self.config()
        rospy.Subscriber(self.robot_pose_topic, PoseStamped, self.robot_pose_callback)
        self.leader_pose_pub = rospy.Publisher(self.leader_pose_topic, PoseWithCovarianceStamped, queue_size=1)

    def config(self):
        self.robot_pose_topic = rospy.get_param('~robot_pose_topic', '/robot_pose')
        self.leader_pose_topic = rospy.get_param('~leader_set_pose_topic', '/set_pose')
        self.relative_pose = rospy.get_param('~relative_pose', [0.0, 0.0, 0.0])

    def run(self):
        


if __name__ == '__main__':
    rospy.init_node('set_leader_pose')
    SetLeaderPose().run()