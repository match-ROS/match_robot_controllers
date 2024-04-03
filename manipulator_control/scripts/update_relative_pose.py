#!/usr/bin/env python3

# this node is used to update the relative pose of the manipulator to the virtual object
# use this node only in standstill

import rospy
from geometry_msgs.msg import PoseStamped
from tf import transformations

class UpdateRelativePose:
    def config(self):
        self.virtual_object_topic = rospy.get_param('~virtual_object_topic', '/virtual_object/object_pose')
        self.TCP_pose_topic = rospy.get_param('~TCP_pose_topic', '/mur620a/UR10_l/global_tcp_pose')
        self.relative_pose_topic = rospy.get_param('~relative_pose_topic', '/mur620a/UR10_l/relative_pose')

        # print parameters to log
        rospy.loginfo('virtual_object_topic: ' + self.virtual_object_topic)
        rospy.loginfo('TCP_pose_topic: ' + self.TCP_pose_topic)
        rospy.loginfo('relative_pose_topic: ' + self.relative_pose_topic)

    def __init__(self):
        self.relative_pose = PoseStamped()
        self.relative_pose.pose.orientation.w = 1.0
        self.config()

        self.relative_pose_pub = rospy.Publisher(self.relative_pose_topic, PoseStamped, queue_size=1)

        self.update_relative_pose()

    def update_relative_pose(self):
        object_pose = rospy.wait_for_message(self.virtual_object_topic, PoseStamped)
        TCP_pose = rospy.wait_for_message(self.TCP_pose_topic, PoseStamped)
        self.relative_pose.pose.position.x = TCP_pose.pose.position.x - object_pose.pose.position.x
        self.relative_pose.pose.position.y = TCP_pose.pose.position.y - object_pose.pose.position.y
        self.relative_pose.pose.position.z = TCP_pose.pose.position.z - object_pose.pose.position.z

        q_diff = transformations.quaternion_multiply(
            [TCP_pose.pose.orientation.x, TCP_pose.pose.orientation.y, TCP_pose.pose.orientation.z, TCP_pose.pose.orientation.w],
            transformations.quaternion_inverse([object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w]))
        self.relative_pose.pose.orientation.x = q_diff[0]
        self.relative_pose.pose.orientation.y = q_diff[1]
        self.relative_pose.pose.orientation.z = q_diff[2]
        self.relative_pose.pose.orientation.w = q_diff[3]

        self.relative_pose_pub.publish(self.relative_pose)
        rospy.sleep(1)
         
        # log relative pose
        rospy.loginfo('Relative pose updated')
        rospy.loginfo('relative_pose: ' + str(self.relative_pose.pose.position.x) + ', ' + str(self.relative_pose.pose.position.y) + ', ' + str(self.relative_pose.pose.position.z) + ', ' + str(self.relative_pose.pose.orientation.x) + ', ' + str(self.relative_pose.pose.orientation.y) + ', ' + str(self.relative_pose.pose.orientation.z) + ', ' + str(self.relative_pose.pose.orientation.w))

        # shutdown the node
        rospy.signal_shutdown('Relative pose updated')


if __name__ == '__main__':
    rospy.init_node('update_relative_pose')
    UpdateRelativePose()
    rospy.spin()