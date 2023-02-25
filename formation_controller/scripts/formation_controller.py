#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from tf import transformations
import math


class Formation_controller():

    def __init__(self):
        self.path_array = rospy.get_param("~path_array", [])
        self.relative_positions_x = rospy.get_param("~relative_positions_x", [0, 0, 0])
        self.relative_positions_y = rospy.get_param("~relative_positions_y", [0, 1, -1])
        self.robot_names = rospy.get_param("~robot_names", ["mir1", "mir2", "mir3"])
        self.current_vel = 0.0
        self.current_omega = 0.0
        self.KP_vel = 0.5
        self.KP_omega = 0.5
        self.acceleration_limit_lin = 1.0
        self.acceleration_limit_ang = 1.0


        rospy.Subscriber('mir_pose_simple', Pose, self.mir_pose_callback)   
        pass


    def run(self):
        # init variables
        path_index = 0
        distances = [0.0 for i in range(len(self.robot_names))]
        target_vels = [0.0 for i in range(len(self.robot_names))]

        while path_index < len(self.path_array)-1 and not rospy.is_shutdown() :
            # compute distance to next point
            for i in range(len(self.robot_names)):
                distances[i] = math.sqrt((self.path_array[path_index][0] - self.mir_pose.position.x)**2 + (self.path_array[path_index][1] - self.mir_pose.position.y)**2)

            # compute target velocity
            for i in range(len(self.robot_names)):
                target_vels[i] = self.KP_vel * distances[i]

            # limit target velocities
            vel_scaling_factor = 1.0
            for i in range(len(self.robot_names)):
                if abs(target_vels[i] - self.current_vel) > self.acceleration_limit_lin:
                    if (self.acceleration_limit_lin / abs(target_vels[i] - self.current_vel)) < vel_scaling_factor:
                        vel_scaling_factor = self.acceleration_limit_lin / abs(target_vels[i] - self.current_vel)

            # check if next point is reached
            for i in range(len(self.robot_names)):
                if distances[i] < self.target_vel:
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
            target_omega = self.KP_omega * angle_error

            # limit target aceeleration
            if abs(target_omega - self.current_omega) > self.acceleration_limit_ang:
                if target_omega > self.current_omega:
                    target_omega = self.current_omega + self.acceleration_limit_ang
                else:
                    target_omega = self.current_omega - self.acceleration_limit_ang
            
            

    def derive_robot_paths(self):
        # derive robot paths from path_array
        self.robot_paths_x = []
        self.robot_paths_y = []
        self.robot_paths_theta = []
        for idx in range(0,len(self.robot_names)):
            for i in range(len(self.path_array)):
                self.robot_paths_x[idx].append(self.path_array[i][0] + self.relative_positions_x[idx] * math.cos(self.path_array[i][2]) - self.relative_positions_y[idx] * math.sin(self.path_array[i][2]))
                self.robot_paths_y[idx].append(self.path_array[i][1] + self.relative_positions_x[idx] * math.sin(self.path_array[i][2]) + self.relative_positions_y[idx] * math.cos(self.path_array[i][2]))

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