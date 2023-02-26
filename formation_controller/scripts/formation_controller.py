#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from tf import transformations
import math


class Formation_controller():

    def __init__(self):
        self.path_array = rospy.get_param("~path_array", [])
        self.relative_positions_x = rospy.get_param("~relative_positions_x", [0, 0, 0])
        self.relative_positions_y = rospy.get_param("~relative_positions_y", [0, 1, -1])
        self.robot_names = rospy.get_param("~robot_names", ["mir1", "mir2", "mir3"])
        self.mir_poses = [Pose() for i in range(len(self.robot_names))]
        self.current_vel = 0.0
        self.current_omega = 0.0
        self.KP_vel = 0.5
        self.KP_omega = 0.5
        self.acceleration_limit_lin = 1.0
        self.acceleration_limit_ang = 1.0
        self.robot_path_publishers = []
        
        for i in range(len(self.robot_names)):
            self.robot_path_publishers.append(rospy.Publisher(self.robot_names[i] + '/robot_path', Path, queue_size=1))

        for i in range(len(self.robot_names)):
            rospy.Subscriber(self.robot_names[i] + '/mir_pose_simple', Pose, self.mir_pose_callback, i)   
        
        rospy.sleep(1.0)


    def run(self):
        self.derive_robot_paths()
        # init variables
        path_index = 0
        distances = [0.0 for i in range(len(self.robot_names))]
        target_vels = [0.0 for i in range(len(self.robot_names))]
        target_points = [[0.0, 0.0] for i in range(len(self.robot_names))]
        target_angles = [0.0 for i in range(len(self.robot_names))]
        target_omegas = [0.0 for i in range(len(self.robot_names))]

        while path_index < len(self.path_array)-1 and not rospy.is_shutdown() :
            # compute distance to next point
            for i in range(len(self.robot_names)):
                distances[i] = math.sqrt((self.path_array[path_index][0] - self.mir_poses[i].position.x)**2 + (self.path_array[path_index][1] - self.mir_poses[i].position.y)**2)

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
                if distances[i] < target_vels[i]:
                    path_index += 1
                    print("Next point")
                    break

            # compute next target point
            for i in range(len(self.robot_names)):
                target_points[i][0] = self.robot_paths_x[i][path_index]*0.5 + self.robot_paths_x[i][path_index+1]*0.5
                target_points[i][1] = self.robot_paths_y[i][path_index]*0.5 + self.robot_paths_y[i][path_index+1]*0.5

            
            # compute angle to target point
            for i in range(len(self.robot_names)):
                target_angles[i] = math.atan2(target_points[i][1] - self.mir_pose[i].position.y, target_points[i][0] - self.mir_pose[i].position.x)

            # compute angle error
            for i in range(len(self.robot_names)):
                current_theta = transformations.euler_from_quaternion([self.mir_poses[i].orientation.x, self.mir_poses[i].orientation.y, self.mir_poses[i].orientation.z, self.mir_poses[i].orientation.w])[2]
                angle_error = target_angles[i] - current_theta
                target_omegas[i] = self.KP_omega * angle_error

            # update velocity scaling factor
            for i in range(len(self.robot_names)):
                if abs(target_omegas[i] - self.current_omega) > self.acceleration_limit_ang:
                    if (self.acceleration_limit_ang / abs(target_omegas[i] - self.current_omega)) < vel_scaling_factor:
                        vel_scaling_factor = self.acceleration_limit_ang / abs(target_omegas[i] - self.current_omega)

            # apple velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] *= vel_scaling_factor
                target_omegas[i] *= vel_scaling_factor

            # publish target velocities
            for i in range(len(self.robot_names)):
                pass
            
            

    def derive_robot_paths(self):
        # derive robot paths from path_array
        self.robot_paths_x = []
        self.robot_paths_y = []
        self.robot_paths_theta = []
        for idx in range(0,len(self.robot_names)):
            for i in range(len(self.path_array)):
                self.robot_paths_x[idx].append(self.path_array[i][0] + self.relative_positions_x[idx] * math.cos(self.path_array[i][2]) - self.relative_positions_y[idx] * math.sin(self.path_array[i][2]))
                self.robot_paths_y[idx].append(self.path_array[i][1] + self.relative_positions_x[idx] * math.sin(self.path_array[i][2]) + self.relative_positions_y[idx] * math.cos(self.path_array[i][2]))
                self.robot_paths_theta[idx].append(self.path_array[i][2])

        self.publish_robot_paths()

    def publish_robot_paths(self):
        # publish robot paths
        robot_path = Path()
        robot_path.header.frame_id = "map"
        robot_path.header.stamp = rospy.Time.now()
        for i in range(len(self.robot_names)):
            for j in range(len(self.robot_paths_x[i])):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = self.robot_paths_x[i][j]
                pose.pose.position.y = self.robot_paths_y[i][j]
                pose.pose.position.z = 0.0
                q = transformations.quaternion_from_euler(0.0, 0.0, self.robot_paths_theta[i][j])
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                robot_path.poses.append(pose)
            self.robot_path_publishers[i].publish(robot_path)

    def mir_pose_callback(self, msg, i):
        self.mir_poses[i] = msg
        # compute current theta
        


if __name__ == "__main__":
    try:
        rospy.init_node('move_to_start_pose')
        rospy.loginfo('move_to_start_pose node started')
        exe = Formation_controller()
        exe.run()
    except rospy.ROSInterruptException:
        pass