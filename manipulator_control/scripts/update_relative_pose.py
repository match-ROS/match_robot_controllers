#!/usr/bin/env python3

# this node is used to update the relative pose of the manipulator to the virtual object
# use this node only in standstill

import rospy
from geometry_msgs.msg import PoseStamped
from tf import transformations
from numpy import transpose

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
        rospy.sleep(1) # wait for publisher to be registered

        self.update_relative_pose()

    def update_relative_pose(self):
        object_pose = rospy.wait_for_message(self.virtual_object_topic, PoseStamped)
        TCP_pose = rospy.wait_for_message(self.TCP_pose_topic, PoseStamped)
        self.relative_pose.pose.position.x = TCP_pose.pose.position.x - object_pose.pose.position.x
        self.relative_pose.pose.position.y = TCP_pose.pose.position.y - object_pose.pose.position.y
        self.relative_pose.pose.position.z = TCP_pose.pose.position.z - object_pose.pose.position.z

        # convert relative pose to object frame
        relative_pose_transformed = PoseStamped()
        relative_pose_transformed.header.frame_id = object_pose.header.frame_id
        R = transformations.quaternion_matrix([object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w])
        R = transpose(R)
        relative_pose_transformed.pose.position.x = self.relative_pose.pose.position.x * R[0,0] + self.relative_pose.pose.position.y * R[0,1] + self.relative_pose.pose.position.z * R[0,2]
        relative_pose_transformed.pose.position.y = self.relative_pose.pose.position.x * R[1,0] + self.relative_pose.pose.position.y * R[1,1] + self.relative_pose.pose.position.z * R[1,2]
        relative_pose_transformed.pose.position.z = self.relative_pose.pose.position.x * R[2,0] + self.relative_pose.pose.position.y * R[2,1] + self.relative_pose.pose.position.z * R[2,2]

        # compute the angle between the virtual object and the TCP
        q_diff = transformations.quaternion_multiply([object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w], transformations.quaternion_inverse([TCP_pose.pose.orientation.x, TCP_pose.pose.orientation.y, TCP_pose.pose.orientation.z, TCP_pose.pose.orientation.w]))
        q_diff = transformations.quaternion_inverse(q_diff)
        # q_diff = transformations.quaternion_multiply(transformations.quaternion_inverse([object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w]), [TCP_pose.pose.orientation.x, TCP_pose.pose.orientation.y, TCP_pose.pose.orientation.z, TCP_pose.pose.orientation.w])
        # q_diff = transformations.quaternion_inverse(q_diff)
        # invert the quaternion
        q = transformations.quaternion_multiply(q_diff, [object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w])
        #q = transformations.quaternion_multiply(q_diff, transformations.quaternion_inverse([object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w]))
        q_inv = q
        relative_pose_transformed.pose.orientation.x = q_inv[0]
        relative_pose_transformed.pose.orientation.y = q_inv[1]
        relative_pose_transformed.pose.orientation.z = q_inv[2]
        relative_pose_transformed.pose.orientation.w = q_inv[3]



        self.relative_pose_pub.publish(relative_pose_transformed)
        rospy.sleep(0.1)
         
        # log relative pose
        rospy.loginfo('Relative pose updated')
        rospy.loginfo('relative_pose: ' + str(self.relative_pose.pose.position.x) + ', ' + str(self.relative_pose.pose.position.y) + ', ' + str(self.relative_pose.pose.position.z) + ', ' + str(self.relative_pose.pose.orientation.x) + ', ' + str(self.relative_pose.pose.orientation.y) + ', ' + str(self.relative_pose.pose.orientation.z) + ', ' + str(self.relative_pose.pose.orientation.w))

        # shutdown the node
        rospy.signal_shutdown('Relative pose updated')


if __name__ == '__main__':
    rospy.init_node('update_relative_pose')
    UpdateRelativePose()
    #rospy.spin()