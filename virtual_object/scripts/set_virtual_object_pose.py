#! /usr/bin/env python3

# this node is used to set the virtual object's pose relativ to a given manipulator

import rospy

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import transformations
from math import cos, sin, pi



class SetVirtualObjectPose():

    def __init__(self):
        self.config()
        self.got_object_pose = False
        rospy.Subscriber(self.TCP_pose_topic, PoseStamped, self.TCP_pose_callback)
        self.set_object_pose_pub = rospy.Publisher(self.set_object_pose_topic, PoseStamped, queue_size=1)
        rospy.sleep(1.0)
        if not self.got_object_pose:
            rospy.logerr('Could not get robot pose. Shutting down node.')
            rospy.signal_shutdown('Could not get robot pose. Shutting down node.')
        else:
            rospy.loginfo('Got robot pose. Shutting down node.')
            rospy.signal_shutdown('Got robot pose. Shutting down node.')

    def config(self):
        self.TCP_pose_topic = rospy.get_param('~TCP_pose_topic', '/mur620a/UR10_l/global_tcp_pose')
        self.set_object_pose_topic = rospy.get_param('~set_object_pose_topic', '/virtual_object/set_pose')
        self.relative_pose = rospy.get_param('~relative_pose', [0.0, -0.2, 0.0, 0.0, 0.0, 0.0])
        


    def TCP_pose_callback(self, data):
        TCP_pose = data.pose
        self.got_object_pose = True
        object_pose = PoseStamped()
        object_pose.header.stamp = rospy.Time.now()
        object_pose.header.frame_id = data.header.frame_id

        R = transformations.quaternion_matrix([TCP_pose.orientation.x, TCP_pose.orientation.y, TCP_pose.orientation.z, TCP_pose.orientation.w])
        p = [TCP_pose.position.x, TCP_pose.position.y, TCP_pose.position.z]
        p = transformations.translation_matrix(p)
        T = transformations.concatenate_matrices(p,R)
        T = transformations.concatenate_matrices(T,transformations.translation_matrix(self.relative_pose))
        object_pose.pose.position.x = T[0,3]
        object_pose.pose.position.y = T[1,3]
        object_pose.pose.position.z = T[2,3]
        relative_pose_q = transformations.quaternion_from_euler(self.relative_pose[3], self.relative_pose[4], self.relative_pose[5])
        q = transformations.quaternion_multiply([TCP_pose.orientation.x, TCP_pose.orientation.y, TCP_pose.orientation.z, TCP_pose.orientation.w], relative_pose_q)
        object_pose.pose.orientation.x = q[0]
        object_pose.pose.orientation.y = q[1]
        object_pose.pose.orientation.z = q[2]
        object_pose.pose.orientation.w = q[3]
        object_pose.header.stamp = rospy.Time.now()

        
        # publish leader pose
        self.set_object_pose_pub.publish(object_pose)
        rospy.sleep(0.1) # wait till pose is published and kill node


if __name__ == '__main__':
    rospy.init_node('set_object_pose')
    SetVirtualObjectPose()