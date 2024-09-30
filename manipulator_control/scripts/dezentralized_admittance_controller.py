#!/usr/bin/env python3

# given a target position and velocity in the world frame and a target admittance, this controller calculates the velocity commands for the manipulators to reach the equlibrium position

import rospy

from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf import transformations, TransformBroadcaster, TransformListener
import tf
from geometry_msgs.msg import WrenchStamped, Wrench
import math
from numpy import transpose
from copy import deepcopy
import numpy as np



class DezentralizedAdmittanceController():

    def config(self):
        self.rate = rospy.get_param('~rate', 250.0)
        self.object_pose_topic = rospy.get_param('~object_pose_topic','/virtual_object/object_pose')
        self.object_vel_topic = rospy.get_param('~object_vel_topic','/virtual_object/object_vel')
        self.manipulator_global_pose_topic = rospy.get_param('~manipulator_global_pose_topic','/mur620a/UR10_l/global_tcp_pose')
        self.manipulator_local_pose_topic = rospy.get_param('~manipulator_local_pose_topic','/mur620a/UR10_l/ur_calibrated_pose')
        self.manipulator_vel_topic = rospy.get_param('~manipulator_vel_topic','manipulator_vel')
        self.manipulator_command_topic = rospy.get_param('~manipulator_command_topic','/mur620a/UR10_l/twist_controller/command_safe')
        self.wrench_topic = rospy.get_param('~wrench_topic','/mur620a/UR10_l/wrench')
        self.mir_pose_topic = rospy.get_param('~mir_pose_topic','/mur620a/mir_pose_simple')
        self.mir_cmd_vel_topic = rospy.get_param('~mir_cmd_vel_topic','/mur620a/cmd_vel')
        self.manipulator_base_frame = rospy.get_param('~manipulator_base_frame','mur620a/UR10_l/base_link')
        self.mir_base_frame = rospy.get_param('~mir_base_frame','mur620a/base_link')
        self.relative_pose_topic = rospy.get_param('~relative_pose_topic','/mur620a/UR10_l/relative_pose')
        self.ur_prefix = rospy.get_param('~ur_prefix','UR10_l')
        self.tf_prefix = rospy.get_param('~tf_prefix','mur620a')
        self.relative_pose = rospy.get_param('~relative_pose', [0.0,0.5,0.0,0,0,0])
        self.admittance = rospy.get_param('~admittance', [0.002,0.002,0.001,0.0,0.0,0.01])
        #self.admittance = rospy.get_param('~admittance', [0.001,0.0,0.001,0.0,0.0,0.0])
        self.wrench_filter_alpha = rospy.get_param('~wrench_filter_alpha', 0.07)
        self.floating_average_window_size = rospy.get_param('~floating_average_window_size', 2000)
        #self.position_error_gain = rospy.get_param('~position_error_gain', [0.3,0.3,0.3,0.1,0.1,0.1])
        self.position_error_gain = rospy.get_param('position_error_gain', [0.5,0.5,0.5,0.4,0.4,0.4])
        #self.position_error_gain = rospy.get_param('position_error_gain', [0.1,0.1,0.1,0.0,0.0,0.0])
        self.linear_velocity_limit = rospy.get_param('~linear_velocity_limit', 0.2)
        self.angular_velocity_limit = rospy.get_param('~angular_velocity_limit', 0.2)
        self.set_reference_at_runtime = rospy.get_param('~set_reference_at_runtime', False)
        self.free_drive_admittance = rospy.get_param('~free_drive_admittance', [0.001,0.001,0.001,0.0,0.0,0.0])
        self.external_localization = rospy.get_param('~external_localization', True)
        pass


    def __init__(self):
        rospy.init_node("dezentralized_admittance_controller")
        rospy.loginfo("dezentralized_admittance_controller running")
        self.config()
    
        # initialize variables
        self.object_pose = PoseStamped()
        self.object_vel = Twist()
        self.manipulator_pose = Pose()
        self.manipulator_vel = Twist()
        self.wrench = Twist()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.pose_error_global = Pose()
        self.pose_error_local = Pose()
        self.manipulator_local_pose = Pose()
        self.wrench_average = Wrench()
        self.admittance_position_offset = Pose()
        self.equilibrium_position_offset = Pose()
        self.grasping_point_velocity_local = Twist()
        self.grasping_point_velocity_global = Twist()
        self.mir_induced_tcp_velocity = Twist()
        self.grasping_point_velocity_manipulator = Twist()
        self.mir_cmd_vel = Twist()
        self.last_command_time = rospy.Time.now()
        self.old_wrench_msgs = np.zeros((self.floating_average_window_size,6))
        self.reference_set = False
        self.last_broadcast_time = rospy.Time.now()

        # initialize broadcaster
        self.br = TransformBroadcaster()
        self.tl = TransformListener()
        
        # initialize publisher  
        self.manipulator_command_pub = rospy.Publisher(self.manipulator_command_topic, Twist, queue_size=1)
        rospy.sleep(1)

        # start subscribers
        rospy.Subscriber(self.object_pose_topic, PoseStamped, self.object_pose_cb)
        rospy.Subscriber(self.object_vel_topic, Twist, self.object_vel_cb)
        rospy.Subscriber(self.manipulator_global_pose_topic, PoseStamped, self.manipulator_global_pose_cb)
        rospy.Subscriber(self.manipulator_local_pose_topic, PoseStamped, self.manipulator_local_pose_cb)
        rospy.Subscriber(self.manipulator_vel_topic, Twist, self.manipulator_vel_cb)
        rospy.Subscriber(self.mir_pose_topic, Pose, self.mir_pose_cb)
        rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_cb)
        rospy.Subscriber(self.mir_cmd_vel_topic, Twist, self.mir_cmd_vel_cb)
        rospy.Subscriber(self.relative_pose_topic, PoseStamped, self.relative_pose_cb)
        rospy.loginfo("Subscribers started" + self.relative_pose_topic)


        

    def run(self):
    
        # wait until first messages are received
        rospy.loginfo("Waiting for first messages")
        rospy.wait_for_message(self.object_pose_topic, PoseStamped)
        rospy.loginfo("got object pose")
        rospy.wait_for_message(self.object_vel_topic, Twist)
        rospy.loginfo("got object vel")
        rospy.wait_for_message(self.manipulator_global_pose_topic, PoseStamped)
        rospy.loginfo("got manipulator pose")
        #rospy.wait_for_message(self.manipulator_vel_topic, Twist)
        rospy.wait_for_message(self.mir_pose_topic, PoseStamped)
        rospy.loginfo("got mir pose")
        rospy.wait_for_message(self.wrench_topic, WrenchStamped)
        rospy.loginfo("First wrench received")
        print(self.manipulator_local_pose_topic)
        rospy.wait_for_message(self.manipulator_local_pose_topic, Pose)
        rospy.loginfo("got manipulator local pose")


        # get pose offset from mir to manipulator
        self.get_manipulator_pose_offset()

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


    def update(self):
        # check if data is fresh
        self.check_data_freshness()
        # compute target pose based on object pose and relative pose
        self.compute_target_pose()
        # compute pose error
        self.compute_pose_error()
        # compute virtual spring equilibrium position
        self.compute_admittance_position_offset()
        # compute equilibrium position based on pose error and admittance
        self.compute_equilibrium_position_offset()
        # compute grapsing point velocity
        self.compute_grasping_point_velocity_local()
        # compute mir induced TCP velocity
        self.compute_mir_induced_tcp_velocity()
        # transform grasping point velocity to global frame
        self.transform_grasping_point_velocity_global()
        # transform grasping point velocity to manipulator frame
        self.transform_grasping_point_velocity_manipulator()

        # compute manipulator velocity
        self.compute_manipulator_velocity()
        # limit and publish manipulator velocity
        self.limit_and_publish_manipulator_velocity()

    def compute_mir_induced_tcp_velocity(self):
        # compute distance between mir and tcp
        distance = math.sqrt((self.mir_pose.position.x - self.manipulator_pose.position.x)**2 + (self.mir_pose.position.y - self.manipulator_pose.position.y)**2)
        # compute absolute mir induced tcp velocity
        abs_mir_induced_tcp_velocity = self.mir_cmd_vel.angular.z * distance
        # compute direction of mir induced tcp velocity
        direction = math.atan2(self.mir_pose.position.y - self.manipulator_pose.position.y, self.mir_pose.position.x - self.manipulator_pose.position.x)
        # compute mir induced tcp velocity
        self.mir_induced_tcp_velocity.linear.x = abs_mir_induced_tcp_velocity * math.cos(direction) + self.mir_cmd_vel.linear.x
        self.mir_induced_tcp_velocity.linear.y = abs_mir_induced_tcp_velocity * math.sin(direction)
        self.mir_induced_tcp_velocity.linear.z = self.mir_cmd_vel.linear.z

        

    def limit_and_publish_manipulator_velocity(self):
        # limit manipulator velocity
        if abs(self.manipulator_vel.linear.x) > self.linear_velocity_limit:
            self.manipulator_vel.linear.x = self.linear_velocity_limit * self.manipulator_vel.linear.x / abs(self.manipulator_vel.linear.x)
        if abs(self.manipulator_vel.linear.y) > self.linear_velocity_limit:
            self.manipulator_vel.linear.y = self.linear_velocity_limit * self.manipulator_vel.linear.y / abs(self.manipulator_vel.linear.y)
        if abs(self.manipulator_vel.linear.z) > self.linear_velocity_limit:
            self.manipulator_vel.linear.z = self.linear_velocity_limit * self.manipulator_vel.linear.z / abs(self.manipulator_vel.linear.z)
        if abs(self.manipulator_vel.angular.x) > self.angular_velocity_limit:
            self.manipulator_vel.angular.x = self.angular_velocity_limit * self.manipulator_vel.angular.x / abs(self.manipulator_vel.angular.x)
        if abs(self.manipulator_vel.angular.y) > self.angular_velocity_limit:
            self.manipulator_vel.angular.y = self.angular_velocity_limit * self.manipulator_vel.angular.y / abs(self.manipulator_vel.angular.y)
        if abs(self.manipulator_vel.angular.z) > self.angular_velocity_limit:
            self.manipulator_vel.angular.z = self.angular_velocity_limit * self.manipulator_vel.angular.z / abs(self.manipulator_vel.angular.z)

        # publish manipulator velocity
        self.manipulator_command_pub.publish(self.manipulator_vel)

    def transform_grasping_point_velocity_manipulator(self):
        
        # transform grasping point velocity to manipulator frame
        R = transformations.quaternion_matrix([self.mir_pose.orientation.x,self.mir_pose.orientation.y,self.mir_pose.orientation.z,self.mir_pose.orientation.w])
        R = transpose(R)
        self.grasping_point_velocity_manipulator.linear.x = R[0,0]*self.grasping_point_velocity_global.linear.x + R[0,1]*self.grasping_point_velocity_global.linear.y + R[0,2]*self.grasping_point_velocity_global.linear.z
        self.grasping_point_velocity_manipulator.linear.y = R[1,0]*self.grasping_point_velocity_global.linear.x + R[1,1]*self.grasping_point_velocity_global.linear.y + R[1,2]*self.grasping_point_velocity_global.linear.z
        self.grasping_point_velocity_manipulator.linear.z = R[2,0]*self.grasping_point_velocity_global.linear.x + R[2,1]*self.grasping_point_velocity_global.linear.y + R[2,2]*self.grasping_point_velocity_global.linear.z
        self.grasping_point_velocity_manipulator.angular.x = R[0,0]*self.grasping_point_velocity_global.angular.x + R[0,1]*self.grasping_point_velocity_global.angular.y + R[0,2]*self.grasping_point_velocity_global.angular.z
        self.grasping_point_velocity_manipulator.angular.y = R[1,0]*self.grasping_point_velocity_global.angular.x + R[1,1]*self.grasping_point_velocity_global.angular.y + R[1,2]*self.grasping_point_velocity_global.angular.z
        self.grasping_point_velocity_manipulator.angular.z = R[2,0]*self.grasping_point_velocity_global.angular.x + R[2,1]*self.grasping_point_velocity_global.angular.y + R[2,2]*self.grasping_point_velocity_global.angular.z



    def transform_grasping_point_velocity_global(self):
        # transform grasping point velocity to global frame
        R = transformations.quaternion_matrix([self.object_pose.pose.orientation.x,self.object_pose.pose.orientation.y,self.object_pose.pose.orientation.z,self.object_pose.pose.orientation.w])
        self.grasping_point_velocity_global.linear.x = self.object_vel.linear.x + R[0,0]*self.grasping_point_velocity_local.linear.x + R[0,1]*self.grasping_point_velocity_local.linear.y + R[0,2]*self.grasping_point_velocity_local.linear.z
        self.grasping_point_velocity_global.linear.y = self.object_vel.linear.y + R[1,0]*self.grasping_point_velocity_local.linear.x + R[1,1]*self.grasping_point_velocity_local.linear.y + R[1,2]*self.grasping_point_velocity_local.linear.z
        self.grasping_point_velocity_global.linear.z = self.object_vel.linear.z + R[2,0]*self.grasping_point_velocity_local.linear.x + R[2,1]*self.grasping_point_velocity_local.linear.y + R[2,2]*self.grasping_point_velocity_local.linear.z
        
        self.grasping_point_velocity_global.angular.x = R[0,0]*self.grasping_point_velocity_local.angular.x + R[0,1]*self.grasping_point_velocity_local.angular.y + R[0,2]*self.grasping_point_velocity_local.angular.z
        self.grasping_point_velocity_global.angular.y = R[1,0]*self.grasping_point_velocity_local.angular.x + R[1,1]*self.grasping_point_velocity_local.angular.y + R[1,2]*self.grasping_point_velocity_local.angular.z
        self.grasping_point_velocity_global.angular.z = R[2,0]*self.grasping_point_velocity_local.angular.x + R[2,1]*self.grasping_point_velocity_local.angular.y + R[2,2]*self.grasping_point_velocity_local.angular.z

    def compute_grasping_point_velocity_local(self):
        # compute the local grasping point velocity based on the object velocity and the relative pose
        self.grasping_point_velocity_local.linear.x = self.relative_pose[2] * self.object_vel.angular.y - self.relative_pose[1] * self.object_vel.angular.z
        self.grasping_point_velocity_local.linear.y = self.relative_pose[0] * self.object_vel.angular.z - self.relative_pose[2] * self.object_vel.angular.x
        self.grasping_point_velocity_local.linear.z = self.relative_pose[1] * self.object_vel.angular.x - self.relative_pose[0] * self.object_vel.angular.y  
        self.grasping_point_velocity_local.angular.x = self.object_vel.angular.x
        self.grasping_point_velocity_local.angular.y = self.object_vel.angular.y
        self.grasping_point_velocity_local.angular.z = self.object_vel.angular.z

    def compute_equilibrium_position_offset(self):
        self.equilibrium_position_offset.position.x = self.pose_error_local.position.x + self.admittance_position_offset.position.x
        self.equilibrium_position_offset.position.y = self.pose_error_local.position.y + self.admittance_position_offset.position.y
        self.equilibrium_position_offset.position.z = self.pose_error_local.position.z + self.admittance_position_offset.position.z
        q = transformations.quaternion_multiply([self.pose_error_local.orientation.x,self.pose_error_local.orientation.y,self.pose_error_local.orientation.z,self.pose_error_local.orientation.w],
                        [self.admittance_position_offset.orientation.x,self.admittance_position_offset.orientation.y,self.admittance_position_offset.orientation.z,self.admittance_position_offset.orientation.w])
        self.equilibrium_position_offset.orientation.x = q[0]
        self.equilibrium_position_offset.orientation.y = q[1]
        self.equilibrium_position_offset.orientation.z = q[2]
        self.equilibrium_position_offset.orientation.w = q[3]

    def compute_admittance_position_offset(self):
        # compute equilibrium position based on wrench error and admittance
        self.admittance_position_offset.position.x = self.filtered_wrench.force.x * self.admittance[0]
        self.admittance_position_offset.position.y = self.filtered_wrench.force.y * self.admittance[1]
        self.admittance_position_offset.position.z = self.filtered_wrench.force.z * self.admittance[2]
        rx = self.filtered_wrench.torque.x * self.admittance[3]
        ry = self.filtered_wrench.torque.y * self.admittance[4]
        rz = self.filtered_wrench.torque.z * self.admittance[5]
        q = transformations.quaternion_from_euler(rx,ry,rz)
        self.admittance_position_offset.orientation.x = q[0]
        self.admittance_position_offset.orientation.y = q[1]
        self.admittance_position_offset.orientation.z = q[2]
        self.admittance_position_offset.orientation.w = q[3]
        self.admittance_rpy = [rx,ry,rz]


    def compute_manipulator_velocity(self):

        # compute manipulator velocity
        self.manipulator_vel.linear.x = self.grasping_point_velocity_manipulator.linear.x + self.equilibrium_position_offset.position.x * self.position_error_gain[0] + self.mir_induced_tcp_velocity.linear.x
        self.manipulator_vel.linear.y = self.grasping_point_velocity_manipulator.linear.y + self.equilibrium_position_offset.position.y * self.position_error_gain[1] - self.mir_induced_tcp_velocity.linear.y
        self.manipulator_vel.linear.z = self.grasping_point_velocity_manipulator.linear.z + self.equilibrium_position_offset.position.z * self.position_error_gain[2]
        euler = transformations.euler_from_quaternion([self.equilibrium_position_offset.orientation.x,self.equilibrium_position_offset.orientation.y,self.equilibrium_position_offset.orientation.z,self.equilibrium_position_offset.orientation.w])
        self.manipulator_vel.angular.x = self.grasping_point_velocity_manipulator.angular.x + euler[0] * self.position_error_gain[3] 
        self.manipulator_vel.angular.y = self.grasping_point_velocity_manipulator.angular.y + euler[1] * self.position_error_gain[4]
        self.manipulator_vel.angular.z = self.grasping_point_velocity_manipulator.angular.z + euler[2] * self.position_error_gain[5] + self.mir_induced_tcp_velocity.angular.z

        R = transformations.quaternion_matrix([self.manipulator_base_pose_offset.orientation.x,self.manipulator_base_pose_offset.orientation.y,self.manipulator_base_pose_offset.orientation.z,self.manipulator_base_pose_offset.orientation.w])
        R = transpose(R)

        vel_local = Twist()
        vel_local.linear.x = R[0,0]*self.manipulator_vel.linear.x + R[0,1]*self.manipulator_vel.linear.y + R[0,2]*self.manipulator_vel.linear.z
        vel_local.linear.y = R[1,0]*self.manipulator_vel.linear.x + R[1,1]*self.manipulator_vel.linear.y + R[1,2]*self.manipulator_vel.linear.z

        vel_local.angular.x = R[0,0]*self.manipulator_vel.angular.x + R[0,1]*self.manipulator_vel.angular.y + R[0,2]*self.manipulator_vel.angular.z
        vel_local.angular.y = R[1,0]*self.manipulator_vel.angular.x + R[1,1]*self.manipulator_vel.angular.y + R[1,2]*self.manipulator_vel.angular.z
        vel_local.angular.z = R[2,0]*self.manipulator_vel.angular.x + R[2,1]*self.manipulator_vel.angular.y + R[2,2]*self.manipulator_vel.angular.z

        # for some reason the angles are inverted when using MOCAP. This is a workaround. TODO: fix this
        if self.external_localization:
            self.manipulator_vel.linear.x =  vel_local.linear.x
            self.manipulator_vel.linear.y =  vel_local.linear.y

            self.manipulator_vel.angular.x =  vel_local.angular.x
            self.manipulator_vel.angular.y =  vel_local.angular.y
        else:
            self.manipulator_vel.linear.x =  vel_local.linear.x
            self.manipulator_vel.linear.y =  vel_local.linear.y

            self.manipulator_vel.angular.x =  -vel_local.angular.x
            self.manipulator_vel.angular.y =  -vel_local.angular.y

    def compute_pose_error(self):
        # 
        # rospy.loginfo("target pose: " + str(self.target_pose.pose.position.x) + " " + str(self.target_pose.pose.position.y) + " " + str(self.target_pose.pose.position.z))
        # rospy.loginfo("manipulator pose: " + str(self.manipulator_pose.position.x) + " " + str(self.manipulator_pose.position.y) + " " + str(self.manipulator_pose.position.z))


        # compute pose error between manipulator and target pose
        self.pose_error_global.position.x = self.target_pose.pose.position.x - self.manipulator_pose.position.x
        self.pose_error_global.position.y = self.target_pose.pose.position.y - self.manipulator_pose.position.y
        self.pose_error_global.position.z = self.target_pose.pose.position.z - self.manipulator_pose.position.z
        q = transformations.quaternion_multiply([self.target_pose.pose.orientation.x,self.target_pose.pose.orientation.y,self.target_pose.pose.orientation.z,self.target_pose.pose.orientation.w],
                        transformations.quaternion_inverse([self.manipulator_pose.orientation.x,self.manipulator_pose.orientation.y,self.manipulator_pose.orientation.z,self.manipulator_pose.orientation.w]))

        self.pose_error_global.orientation.x = q[0]
        self.pose_error_global.orientation.y = q[1]
        self.pose_error_global.orientation.z = q[2]
        self.pose_error_global.orientation.w = q[3]

        phi, theta, psi = transformations.euler_from_quaternion([self.pose_error_global.orientation.x,self.pose_error_global.orientation.y,self.pose_error_global.orientation.z,self.pose_error_global.orientation.w])
        # print("phi: " + str(phi) + " theta: " + str(theta) + " psi: " + str(psi))

        # transform pose error to local frame using the mir pose
        R = transformations.quaternion_matrix([self.mir_pose.orientation.x,self.mir_pose.orientation.y,self.mir_pose.orientation.z,self.mir_pose.orientation.w])
        R = transpose(R)
        self.pose_error_local.position.x = R[0,0]*self.pose_error_global.position.x + R[0,1]*self.pose_error_global.position.y 
        self.pose_error_local.position.y = R[1,0]*self.pose_error_global.position.x + R[1,1]*self.pose_error_global.position.y 
        self.pose_error_local.position.z = self.pose_error_global.position.z
        global_pose_euler = transformations.euler_from_quaternion([self.pose_error_global.orientation.x,self.pose_error_global.orientation.y,self.pose_error_global.orientation.z,self.pose_error_global.orientation.w])
        local_pose_euler = [0.0,0.0,0.0]
        local_pose_euler[0] = global_pose_euler[0] * R[0,0] + global_pose_euler[1] * R[0,1] + global_pose_euler[2] * R[0,2]
        local_pose_euler[1] = global_pose_euler[0] * R[1,0] + global_pose_euler[1] * R[1,1] + global_pose_euler[2] * R[1,2]
        local_pose_euler[2] = global_pose_euler[0] * R[2,0] + global_pose_euler[1] * R[2,1] + global_pose_euler[2] * R[2,2]
        q = transformations.quaternion_from_euler(local_pose_euler[0],local_pose_euler[1],local_pose_euler[2])

        self.pose_error_local.orientation.x = q[0]
        self.pose_error_local.orientation.y = q[1]
        self.pose_error_local.orientation.z = q[2]
        self.pose_error_local.orientation.w = q[3]

        #print(self.pose_error_local)

        if self.set_reference_at_runtime and self.reference_set == False:
            self.admittance = self.free_drive_admittance # make the robot easier to move while no reference is set
            self.pose_error_local = Pose()
            self.pose_error_local.orientation.w = 1.0
            rospy.logwarn_throttle(3,"Reference not set, ignoring pose error")


    def compute_target_pose(self):
        R = transformations.quaternion_matrix([self.object_pose.pose.orientation.x,self.object_pose.pose.orientation.y,self.object_pose.pose.orientation.z,self.object_pose.pose.orientation.w])
        p = [self.object_pose.pose.position.x,self.object_pose.pose.position.y,self.object_pose.pose.position.z]
        p = transformations.translation_matrix(p)
        T = transformations.concatenate_matrices(p,R)
        T = transformations.concatenate_matrices(T,transformations.translation_matrix(self.relative_pose))

        self.target_pose.pose.position.x = T[0,3]
        self.target_pose.pose.position.y = T[1,3]
        self.target_pose.pose.position.z = T[2,3]
        relative_pose_q = transformations.quaternion_from_euler(self.relative_pose[3],self.relative_pose[4],self.relative_pose[5])
        q = transformations.quaternion_multiply([self.object_pose.pose.orientation.x,self.object_pose.pose.orientation.y,self.object_pose.pose.orientation.z,self.object_pose.pose.orientation.w],relative_pose_q)
        self.target_pose.pose.orientation.x = q[0]
        self.target_pose.pose.orientation.y = q[1]
        self.target_pose.pose.orientation.z = q[2]
        self.target_pose.pose.orientation.w = q[3]
        self.target_pose.header.stamp = rospy.Time.now()
        
        #limiting broadcast rate
        if rospy.Time.now() - self.last_broadcast_time > rospy.Duration(0.2):
            #broadcast target pose
            self.br.sendTransform((self.target_pose.pose.position.x,self.target_pose.pose.position.y,self.target_pose.pose.position.z),
                        (self.target_pose.pose.orientation.x,self.target_pose.pose.orientation.y,self.target_pose.pose.orientation.z,self.target_pose.pose.orientation.w),
                        rospy.Time.now(),
                        self.tf_prefix+ "/" + self.ur_prefix + "/target_pose",
                        "map")
            
            # broadcast current pose
            self.br.sendTransform((self.manipulator_pose.position.x,self.manipulator_pose.position.y,self.manipulator_pose.position.z),
                        (self.manipulator_pose.orientation.x,self.manipulator_pose.orientation.y,self.manipulator_pose.orientation.z,self.manipulator_pose.orientation.w),
                        rospy.Time.now(),
                        self.tf_prefix+ "/" + self.ur_prefix + "/current_pose",
                        "map")
            self.last_broadcast_time = rospy.Time.now()
        

    def get_manipulator_pose_offset(self):
        while not rospy.is_shutdown():
            try:
                now = rospy.rostime.Time(0)
                self.tl.waitForTransform(self.mir_base_frame, self.manipulator_base_frame, now, rospy.Duration(4.0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Could not get transform from mir base frame to manipulator base frame")
                continue
        
        try:
            trans, rot = self.tl.lookupTransform(self.mir_base_frame, self.manipulator_base_frame, now)

            self.manipulator_base_pose_offset = Pose()
            self.manipulator_base_pose_offset.position.x = trans[0]
            self.manipulator_base_pose_offset.position.y = trans[1]
            self.manipulator_base_pose_offset.position.z = trans[2]

            self.manipulator_base_pose_offset.orientation.x = rot[0]
            self.manipulator_base_pose_offset.orientation.y = rot[1]
            self.manipulator_base_pose_offset.orientation.z = rot[2]
            self.manipulator_base_pose_offset.orientation.w = rot[3]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Could not get transform from mir base frame to manipulator base frame")
            return



    def filter_wrench(self,wrench = Wrench()):
        self.wrench_average.force.x = (1-self.wrench_filter_alpha)*self.wrench_average.force.x + self.wrench_filter_alpha*wrench.force.x
        self.wrench_average.force.y = (1-self.wrench_filter_alpha)*self.wrench_average.force.y + self.wrench_filter_alpha*wrench.force.y
        self.wrench_average.force.z = (1-self.wrench_filter_alpha)*self.wrench_average.force.z + self.wrench_filter_alpha*wrench.force.z
        self.wrench_average.torque.x = (1-self.wrench_filter_alpha)*self.wrench_average.torque.x + self.wrench_filter_alpha*wrench.torque.x
        self.wrench_average.torque.y = (1-self.wrench_filter_alpha)*self.wrench_average.torque.y + self.wrench_filter_alpha*wrench.torque.y
        self.wrench_average.torque.z = (1-self.wrench_filter_alpha)*self.wrench_average.torque.z + self.wrench_filter_alpha*wrench.torque.z       

    def check_data_freshness(self):
        if rospy.Time.now() - self.last_command_time > rospy.Duration(0.1):
            self.mir_cmd_vel = Twist()

    def object_pose_cb(self,data = PoseStamped()):
        self.object_pose = data

    def object_vel_cb(self,data = Twist()):
        self.object_vel = data

    def manipulator_global_pose_cb(self,data = PoseStamped()):
        self.manipulator_pose = data.pose

    def manipulator_local_pose_cb(self,data = PoseStamped()):
        self.manipulator_local_pose = data.pose

    def manipulator_vel_cb(self,data = Twist()):    
        self.manipulator_vel = data

    def mir_cmd_vel_cb(self,data = Twist()):
        self.mir_cmd_vel = data
        self.last_command_time = rospy.Time.now()   

    def relative_pose_cb(self,data = PoseStamped()):
        euler = transformations.euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        self.relative_pose = [data.pose.position.x,data.pose.position.y,data.pose.position.z,euler[0],euler[1],euler[2]]
        if self.reference_set == False:
            self.reference_set = True
            self.admittance = rospy.get_param('~admittance', [0.0005,0.0005,0.001,0.0,0.0,0.01])
            rospy.loginfo("Reference set")

    def wrench_cb(self,data = WrenchStamped()):
        wrench = data.wrench
        wrench_out = Wrench()
        # get wrench orientation from local pose frame
        # turn the orientation of the manipulator by 180 degrees around the z axis
        q_rot = transformations.quaternion_from_euler(0,0,math.pi)
        rot = transformations.quaternion_multiply([self.manipulator_local_pose.orientation.x,self.manipulator_local_pose.orientation.y,self.manipulator_local_pose.orientation.z,self.manipulator_local_pose.orientation.w],q_rot)

        # convert wrench to local frame
        R = transformations.quaternion_matrix(rot)
        R = transpose(R)
        wrench_out.force.x = R[0,0]*wrench.force.x + R[0,1]*wrench.force.y + R[0,2]*wrench.force.z
        wrench_out.force.y = R[1,0]*wrench.force.x + R[1,1]*wrench.force.y + R[1,2]*wrench.force.z
        wrench_out.force.z = R[2,0]*wrench.force.x + R[2,1]*wrench.force.y + R[2,2]*wrench.force.z
        wrench_out.torque.x = R[0,0]*wrench.torque.x + R[0,1]*wrench.torque.y + R[0,2]*wrench.torque.z
        wrench_out.torque.y = R[1,0]*wrench.torque.x + R[1,1]*wrench.torque.y + R[1,2]*wrench.torque.z
        wrench_out.torque.z = R[2,0]*wrench.torque.x + R[2,1]*wrench.torque.y + R[2,2]*wrench.torque.z

        #TODO fix this later! - double inversion is intentional
        if self.external_localization:
            wrench_out.force.x = -wrench_out.force.x
            wrench_out.force.y = -wrench_out.force.y
            wrench_out.torque.x = -wrench_out.torque.x
            wrench_out.torque.y = -wrench_out.torque.y
        if self.ur_prefix == 'UR10_r':
            wrench_out.force.x = -wrench_out.force.x
            wrench_out.force.y = -wrench_out.force.y
            wrench_out.torque.x = -wrench_out.torque.x
            wrench_out.torque.y = -wrench_out.torque.y

        # filter wrench
        self.filter_wrench(wrench_out)
        self.filtered_wrench = deepcopy(self.wrench_average)

    def mir_pose_cb(self,data = Pose()):
        self.mir_pose = data


if __name__ == "__main__":
    DezentralizedAdmittanceController().run()
    rospy.spin()