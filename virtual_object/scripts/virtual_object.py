#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from tf import transformations
import tf2_ros
from copy import deepcopy
from helper_nodes.virtual_object_helper import VirtualObjectHelper
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

class VirtualObject():

    def __init__(self):
        rospy.init_node("virtual_object_node")
        rospy.loginfo("virtual_object_node running")
        VirtualObjectHelper(self)
        self.run()
        rospy.spin()

    # The virutal leader node simulates the leader of the formation. It publishes the leader pose and velocity. The target leader pose is computed 
    # from the target leader pose and the target leader velocity by integration. 
    def run(self):
        Rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            if (rospy.get_time() - self.last_cmd_time.to_sec()) > self.cmd_vel_timeout:
                self.master_vel = Twist()


            _object_vel = deepcopy(self.master_vel) # copy the master_vel to avoid overwriting the original

            time_current = rospy.get_time()
            dt = time_current - self.time_old
            self.time_old = time_current
            
            # calculate the distance the object has moved since the last time
            self.d_pose[0] = _object_vel.linear.x * dt
            self.d_pose[1] = _object_vel.linear.y * dt
            self.d_pose[2] = _object_vel.linear.z * dt
            self.d_pose[3] = _object_vel.angular.x * dt
            self.d_pose[4] = _object_vel.angular.y * dt
            self.d_pose[5] = _object_vel.angular.z * dt
            
            # calculate the new pose of the object
            self.object_pose.pose.position.x = self.object_pose.pose.position.x + self.d_pose[0]
            self.object_pose.pose.position.y = self.object_pose.pose.position.y + self.d_pose[1]
            self.object_pose.pose.position.z = self.object_pose.pose.position.z + self.d_pose[2]
            dq = transformations.quaternion_from_euler(self.d_pose[3],self.d_pose[4],self.d_pose[5])
            q = transformations.quaternion_multiply([self.object_pose.pose.orientation.x,self.object_pose.pose.orientation.y,self.object_pose.pose.orientation.z,self.object_pose.pose.orientation.w],dq)

            self.object_pose.pose.orientation.x = q[0]
            self.object_pose.pose.orientation.y = q[1]
            self.object_pose.pose.orientation.z = q[2]
            self.object_pose.pose.orientation.w = q[3]
            
            self.object_pose.header.stamp = rospy.Time.now()
            self.object_pose.header.frame_id = 'map'

            self.pub.publish(self.object_pose)

            br = tf2_ros.TransformBroadcaster()
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "virtual_object/base_link"
            t.transform.translation = self.object_pose.pose.position
            t.transform.rotation = self.object_pose.pose.orientation
            br.sendTransform(t)

            self.pub_vel.publish(_object_vel)
                
            Rate.sleep()


if __name__=="__main__":
    VirtualObject().run()
