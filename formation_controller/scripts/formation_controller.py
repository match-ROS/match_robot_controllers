#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from tf import transformations
import math


class Formation_controller():

    def __init__(self):
        self.path_array = rospy.get_param("~path_array", [])
        self.current_vel = 0.0
        self.current_omega = 0.0
        self.KP_vel = 0.5
        self.KP_omega = 0.5
        self.acceleration_limit_lin = 1.0
        self.acceleration_limit_ang = 1.0

        rospy.Subscriber('mir_pose_simple', Pose, self.mir_pose_callback)   
        pass


    def run(self):
        path_index = 0

        while path_index < len(self.path_array)-1 and not rospy.is_shutdown() :
            # compute distance to next point
            distance = math.sqrt((self.path_array[path_index][0] - self.mir_pose.position.x)**2 + (self.path_array[path_index][1] - self.mir_pose.position.y)**2)

            # compute target velocity
            target_vel = self.KP_vel * distance

            # limit target aceeleration
            if abs(target_vel - self.current_vel) > self.acceleration_limit_lin:
                if target_vel > self.current_vel:
                    target_vel = self.current_vel + self.acceleration_limit_lin
                else:
                    target_vel = self.current_vel - self.acceleration_limit_lin 

            # check if next point is reached
            if distance < self.target_vel:
                path_index += 1
                print("Next point")
                break

            # compute next target point
            target_point = [0.0, 0.0]
            target_point[0] = self.path_array[path_index][0]*0.5 + self.path_array[path_index+1][0]*0.5
            target_point[1] = self.path_array[path_index][1]*0.5 + self.path_array[path_index+1][1]*0.5
            
            # compute angle to target point
            angle = math.atan2(target_point[1] - self.mir_pose.position.y, target_point[0] - self.mir_pose.position.x)

            # compute angle error
            angle_error = angle - self.current_theta

            # compute omega

            
            
            print(self.path_array[i][0])



    def mir_pose_callback(self, msg):
        self.mir_pose = msg
        # compute current theta
        self.current_theta = transformations.euler_from_quaternion([self.mir_pose.orientation.x, self.mir_pose.orientation.y, self.mir_pose.orientation.z, self.mir_pose.orientation.w])[2]



if __name__ == "__main__":
    try:
        rospy.init_node('move_to_start_pose')
        rospy.loginfo('move_to_start_pose node started')
        exe = Formation_controller()
        exe.run()
    except rospy.ROSInterruptException:
        pass