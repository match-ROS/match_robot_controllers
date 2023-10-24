#!/usr/bin/env python3

# given a target position in the world frame and local target velocity, this controller calculates the velocity commands for the robot to reach the target position

import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf import transformations
import math

class DecentralizedLeaderFollowerController:
    
    def __init__(self) -> None:
        self.config()
        rospy.Subscriber(self.target_pose_topic, PoseStamped, self.target_pose_callback)
        rospy.Subscriber(self.actual_pose_topic, PoseStamped, self.actual_pose_callback)
        rospy.Subscriber(self.target_velocity_topic, Twist, self.target_velocity_callback)


    def config(self):
        self.target_pose_topic = rospy.get_param("~target_pose_topic", "/target_pose")
        self.actual_pose_topic = rospy.get_param("~actual_pose_topic", "/actual_pose")
        self.target_velocity_topic = rospy.get_param("~target_velocity_topic", "/target_velocity")
        self.control_rate = rospy.get_param("~control_rate", 100)
        pass

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            if self.target_pose is not None and self.actual_pose is not None and self.target_velocity is not None:
                self.cartesian_controller(self.actual_pose, self.target_pose, self.target_velocity)
            else:
                rospy.loginfo("Waiting for target pose, actual pose and target velocity")
            rate.sleep()
            








    def cartesian_controller(self,actual_pose = Pose(),target_pose = Pose(),target_velocity = Twist(),i = 0):
            phi_act = transformations.euler_from_quaternion([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])
            phi_target = transformations.euler_from_quaternion([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w])
            R = transformations.quaternion_matrix([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])

            e_x = (target_pose.position.x- actual_pose.position.x)
            e_y = (target_pose.position.y - actual_pose.position.y)
            e_local_x = R[0,0]*e_x + R[1,0]*e_y
            e_local_y = R[0,1]*e_x + R[1,1]*e_y

            # broadcast actual pose
            self.target_pose_broadcaster.sendTransform((actual_pose.position.x, actual_pose.position.y, 0.0),
                                                (actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w),
                                                rospy.Time.now(),
                                                "actual_pose_" + str(i),
                                                "map")

            # broadcast target pose
            self.target_pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, 0.0),
                                                (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w),
                                                rospy.Time.now(),
                                                "target_pose_controll" + str(i),
                                                "map")

            u_w = target_velocity.angular.z + target_velocity.linear.x * ( self.Ky * e_local_y + self.Kphi * math.sin(phi_target[2]-phi_act[2]))
            u_v = target_velocity.linear.x * math.cos(phi_target[2]-phi_act[2]) + self.Kx*e_local_x

            # publish metadata
            self.metadata_publisher.publish_controller_metadata(target_pose = target_pose, actual_pose = actual_pose, target_velocity = target_velocity, publish = True,
            error = [e_local_x,e_local_y,phi_target[2]-phi_act[2]], robot_id = i) 

            return u_v, u_w
        
    def target_pose_callback(self, msg):
        self.target_pose = msg.pose    
    
    def actual_pose_callback(self, msg):
        self.actual_pose = msg.pose  
    
    def target_velocity_callback(self, msg):
        self.target_velocity = msg.twist
        
        
if __name__ == '__main__':
    try:
        rospy.init_node('decentralized_leader_follower_controller', anonymous=True)
        controller = DecentralizedLeaderFollowerController().run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    