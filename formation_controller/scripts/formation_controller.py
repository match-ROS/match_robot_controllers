#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path 
from geometry_msgs.msg import Pose, PoseStamped, Twist
from tf import transformations, broadcaster
import math


class Formation_controller():

    def __init__(self):
        self.path_array = rospy.get_param("~path_array", [])
        self.relative_positions_x = rospy.get_param("~relative_positions_x", [0, 0, 0])
        self.relative_positions_y = rospy.get_param("~relative_positions_y", [0, 1, -1])
        self.robot_names = rospy.get_param("~robot_names", ["mir1", "mir2", "mir3"])
        self.mir_poses = [Pose() for i in range(len(self.robot_names))]
        self.target_pose_broadcaster = broadcaster.TransformBroadcaster()
        self.current_vel = 0.0
        self.current_omega = 0.0
        self.KP_vel = 1.0
        self.KP_omega = 1.0
        self.control_rate = rospy.get_param("~control_rate", 100.0)
        self.velocity_limit_lin = 0.2
        self.velocity_limit_ang = 0.4
        self.acceleration_limit_lin = 1.0
        self.acceleration_limit_ang = 2.0
        self.robot_path_publishers = []
        self.robot_twist_publishers = []
        
        for i in range(len(self.robot_names)):
            self.robot_path_publishers.append(rospy.Publisher(self.robot_names[i] + '/robot_path', Path, queue_size=1))
            self.robot_twist_publishers.append(rospy.Publisher(self.robot_names[i] + '/mobile_base_controller/cmd_vel', Twist, queue_size=1))

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
        target_poses = [Pose() for i in range(len(self.robot_names))]
        current_vels = [0.0 for i in range(len(self.robot_names))]
        current_omegas = [0.0 for i in range(len(self.robot_names))]
        current_thetas = [0.0 for i in range(len(self.robot_names))]

        rate = rospy.Rate(self.control_rate)
        # main loop
        while path_index < len(self.path_array)-2 and not rospy.is_shutdown() :
            # compute distance to next point
            for i in range(len(self.robot_names)):
                distances[i] = math.sqrt((self.path_array[path_index][0] - self.mir_poses[i].position.x)**2 + (self.path_array[path_index][1] - self.mir_poses[i].position.y)**2)

            # compute target velocity
            for i in range(len(self.robot_names)):
                target_vels[i] = self.velocity_limit_lin #self.KP_vel * distances[i] * self.control_rate

            # limit target velocity
            vel_scaling_factor = 1.0
            for i in range(len(self.robot_names)):
                if abs(target_vels[i]) > self.velocity_limit_lin:
                    if (self.velocity_limit_lin / abs(target_vels[i])) < vel_scaling_factor:
                        vel_scaling_factor = self.velocity_limit_lin / abs(target_vels[i])

            # limit target acceleration
            for i in range(len(self.robot_names)):
                if abs(target_vels[i] - self.current_vel) > self.acceleration_limit_lin:
                    if (self.acceleration_limit_lin / abs(target_vels[i] - self.current_vel)) < vel_scaling_factor:
                        vel_scaling_factor = self.acceleration_limit_lin / abs(target_vels[i] - self.current_vel)

            # check if next point is reached
            for i in range(len(self.robot_names)):
                if distances[i] < target_vels[i]*vel_scaling_factor:
                    path_index += 1
                    print("Next point")
                    break

            # compute next target point
            for i in range(len(self.robot_names)):
                target_points[i][0] = self.robot_paths_x[i][path_index+2]*0.5 + self.robot_paths_x[i][path_index+1]*0.5
                target_points[i][1] = self.robot_paths_y[i][path_index+2]*0.5 + self.robot_paths_y[i][path_index+1]*0.5

            
            # compute angle to target point
            for i in range(len(self.robot_names)):
                target_angles[i] = math.atan2(target_points[i][1] - self.robot_paths_y[i][path_index], target_points[i][0] - self.robot_paths_x[i][path_index])

            # compute angle error
            for i in range(len(self.robot_names)):
                #current_theta = transformations.euler_from_quaternion([self.mir_poses[i].orientation.x, self.mir_poses[i].orientation.y, self.mir_poses[i].orientation.z, self.mir_poses[i].orientation.w])[2]
                angle_error = target_angles[i] - current_thetas[i]
                print("Angle error: " + str(angle_error))
                target_omegas[i] = self.KP_omega * angle_error

            # limit angular velocity
            for i in range(len(self.robot_names)):
                if abs(target_omegas[i]) > self.velocity_limit_ang:
                    if (self.velocity_limit_ang / abs(target_omegas[i])) < vel_scaling_factor:
                        vel_scaling_factor = self.velocity_limit_ang / abs(target_omegas[i])

            # limit angular acceleration
            for i in range(len(self.robot_names)):
                if abs(target_omegas[i] - self.current_omega) > self.acceleration_limit_ang:
                    if (self.acceleration_limit_ang / abs(target_omegas[i] - self.current_omega)) < vel_scaling_factor:
                        vel_scaling_factor = self.acceleration_limit_ang / abs(target_omegas[i] - self.current_omega)

            # apply velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] *= vel_scaling_factor
                target_omegas[i] *= vel_scaling_factor

            # compute target pose for each robot
            for i in range(len(self.robot_names)):
                target_poses[i].position.x += target_vels[i] * math.cos(target_angles[i]) * rate.sleep_dur.to_sec()
                target_poses[i].position.y += target_vels[i] * math.sin(target_angles[i]) * rate.sleep_dur.to_sec()
                q = transformations.quaternion_from_euler(0.0, 0.0, target_angles[i])
                target_poses[i].orientation.x = q[0]
                target_poses[i].orientation.y = q[1]
                target_poses[i].orientation.z = q[2]
                target_poses[i].orientation.w = q[3]

            # broadcast target poses
            for i in range(len(self.robot_names)):
                self.target_pose_broadcaster.sendTransform((target_poses[i].position.x, target_poses[i].position.y, 0.0),
                                            (target_poses[i].orientation.x, target_poses[i].orientation.y, target_poses[i].orientation.z, target_poses[i].orientation.w),
                                            rospy.Time.now(),
                                            "target_pose_" + str(i),
                                            "map")


            # compute control law and publish target velocities
            for i in range(len(self.robot_names)):
                target_velocity = Twist()
                target_velocity.linear.x = target_vels[i]
                target_velocity.angular.z = target_omegas[i]
                u_v, v_w = self.cartesian_controller(self.mir_poses[i],target_poses[i],target_velocity)

                # publish target velocities
                target_velocity.linear.x = u_v
                target_velocity.angular.z = v_w
                self.robot_twist_publishers[i].publish(target_velocity)

                # update current velocities
                current_vels[i] = u_v
                current_omegas[i] = v_w
                current_thetas[i] += v_w * rate.sleep_dur.to_sec()


            rate.sleep()

            

            
            

    def cartesian_controller(self,actual_pose = Pose(),target_pose = Pose(),target_velocity = Twist()):
        Kv = 0.2
        Ky = 0.2
        Kx = 0.2
        phi_act = transformations.euler_from_quaternion([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])
        phi_target = transformations.euler_from_quaternion([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w])

        e_x = (target_pose.position.x-actual_pose.position.x)
        e_y = (target_pose.position.x - actual_pose.position.y)
        #rospy.loginfo_throttle(1,[id,e_x,e_y])
        e_local_x = math.cos(phi_act[2]) * e_x + math.sin(phi_act[2]) * e_y
        e_local_y = math.cos(phi_act[2]) * e_y - math.sin(phi_act[2]) * e_x

        #print(target_velocity)

        u_w = target_velocity.angular.z + target_velocity.linear.x * Kv * e_local_y + Ky * math.sin(phi_target[2]-phi_act[2])
        u_v = target_velocity.linear.x * math.cos(phi_target[2]-phi_act[2]) + Kx*e_local_x

        return u_v, u_w



    def derive_robot_paths(self):
        # derive robot paths from path_array
        self.robot_paths_x = [ [] for i in range(len(self.robot_names))]
        self.robot_paths_y = [ [] for i in range(len(self.robot_names))]
        self.robot_paths_theta = [ [] for i in range(len(self.robot_names))]
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
            robot_path = Path()
            robot_path.header.frame_id = "map"
            robot_path.header.stamp = rospy.Time.now()
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
            rospy.sleep(0.5)

    def mir_pose_callback(self, msg, i):
        self.mir_poses[i] = msg
        # compute current theta
        


if __name__ == "__main__":
    try:
        rospy.init_node('formation_controller')
        rospy.loginfo('formation_controller node started')
        exe = Formation_controller()
        exe.run()
    except rospy.ROSInterruptException:
        pass