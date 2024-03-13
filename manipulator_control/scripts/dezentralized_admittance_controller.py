#!/usr/bin/env python3

# given a target position and velocity in the world frame and a target admittance, this controller calculates the velocity commands for the manipulators to reach the equlibrium position

import rospy

from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf import transformations, TransformBroadcaster, TransformListener
import tf
from geometry_msgs.msg import WrenchStamped, Wrench
import math
from numpy import transpose



class DezentralizedAdmittanceController():

    def config(self):
        self.rate = rospy.get_param('~rate', 100.0)
        self.object_pose_topic = rospy.get_param('~object_pose_topic','/virtual_object/object_pose')
        self.object_vel_topic = rospy.get_param('~object_vel_topic','/virtual_object/object_vel')
        self.manipulator_global_pose_topic = rospy.get_param('~manipulator_global_pose_topic','/mur620a/UR10_l/global_tcp_pose')
        self.manipulator_vel_topic = rospy.get_param('~manipulator_vel_topic','manipulator_vel')
        self.manipulator_command_topic = rospy.get_param('~manipulator_command_topic','/mur620a/UR10_l/twist_controller/command_safe')
        self.wrench_topic = rospy.get_param('~wrench_topic','/mur620a/UR10_l/wrench')
        self.wrench_frame = rospy.get_param('~wrench_frame','/mur620a/UR10_l/tool0')
        self.mir_pose_topic = rospy.get_param('~mir_pose_topic','/mur620a/mir_pose_simple')
        self.mir_cmd_vel_topic = rospy.get_param('~mir_cmd_vel_topic','/mur620a/cmd_vel')
        self.manipulator_base_frame = rospy.get_param('~manipulator_base_frame','mur620a/UR10_l/base_link')
        self.mir_base_frame = rospy.get_param('~mir_base_frame','mur620a/base_link')
        self.relative_pose = rospy.get_param('~relative_pose', [0.0,0.2,0.0,0,0,0])
        # self.admittance = rospy.get_param('admittance', [0.02,0.02,0.02,0.01,0.01,0.01])
        self.admittance = rospy.get_param('~admittance', [0.0,0.0,0.0,0.0,0.0,0.0])
        self.wrench_filter_alpha = rospy.get_param('~wrench_filter_alpha', 0.01)
        #self.position_error_gain = rospy.get_param('~position_error_gain', [0.3,0.3,0.3,0.1,0.1,0.1])
        self.position_error_gain = rospy.get_param('position_error_gain', [0.0,0.0,0.0,0.1,0.1,0.1])
        self.linear_velocity_limit = rospy.get_param('~linear_velocity_limit', 0.1)
        self.angular_velocity_limit = rospy.get_param('~angular_velocity_limit', 0.1)
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
        self.wrench_average = Wrench()
        self.admittance_position_offset = Pose()
        self.equilibrium_position_offset = Pose()
        self.grasping_point_velocity_local = Twist()
        self.grasping_point_velocity_global = Twist()
        self.mir_induced_tcp_velocity = Twist()
        self.grasping_point_velocity_manipulator = Twist()
        self.mir_cmd_vel = Twist()
        self.last_command_time = rospy.Time.now()

        # initialize broadcaster
        self.br = TransformBroadcaster()
        self.tl = TransformListener()
        
        # start subscribers
        rospy.Subscriber(self.object_pose_topic, PoseStamped, self.object_pose_cb)
        rospy.Subscriber(self.object_vel_topic, Twist, self.object_vel_cb)
        rospy.Subscriber(self.manipulator_global_pose_topic, PoseStamped, self.manipulator_pose_cb)
        rospy.Subscriber(self.manipulator_vel_topic, Twist, self.manipulator_vel_cb)
        rospy.Subscriber(self.mir_pose_topic, Pose, self.mir_pose_cb)
        rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_cb)
        rospy.Subscriber(self.mir_cmd_vel_topic, Twist, self.mir_cmd_vel_cb)

        # initialize publisher  
        self.manipulator_command_pub = rospy.Publisher(self.manipulator_command_topic, Twist, queue_size=10)
        

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
        rospy.loginfo("First messages received")

        rospy.loginfo("Waiting for transform from wrench frame to manipulator base frame")
        self.tl.waitForTransform(self.manipulator_base_frame, self.wrench_frame, rospy.Time(0), rospy.Duration(5.0))
        rospy.loginfo("Got transform from wrench frame to manipulator base frame")

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

        #print("Mir Induced TCP Velocity: ", self.mir_induced_tcp_velocity.linear.x, self.mir_induced_tcp_velocity.linear.y)
        
        #rospy.loginfo(self.mir_pose)
        #rospy.loginfo(self.manipulator_pose)
        #print("Distance: ", distance)
        

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
        # the robot frame is always turned 180 degrees around the z axis
        q = transformations.quaternion_from_euler(0,0,math.pi)
        mir_ur_q = transformations.quaternion_multiply([self.manipulator_base_pose_offset.orientation.x,self.manipulator_base_pose_offset.orientation.y,self.manipulator_base_pose_offset.orientation.z,self.manipulator_base_pose_offset.orientation.w],q)

        # add this transformation to the map / mir transformation
        q_map_mir_ur = transformations.quaternion_multiply([self.mir_pose.orientation.x,self.mir_pose.orientation.y,self.mir_pose.orientation.z,self.mir_pose.orientation.w],mir_ur_q)

        # transform grasping point velocity to manipulator frame
        # R = transformations.quaternion_matrix([self.mir_pose.orientation.x,self.mir_pose.orientation.y,self.mir_pose.orientation.z,self.mir_pose.orientation.w])
        R = transformations.quaternion_matrix(q_map_mir_ur)
        R = transpose(R)
        self.grasping_point_velocity_manipulator.linear.x = R[0,0]*self.grasping_point_velocity_global.linear.x + R[0,1]*self.grasping_point_velocity_global.linear.y + R[0,2]*self.grasping_point_velocity_global.linear.z
        self.grasping_point_velocity_manipulator.linear.y = R[1,0]*self.grasping_point_velocity_global.linear.x + R[1,1]*self.grasping_point_velocity_global.linear.y + R[1,2]*self.grasping_point_velocity_global.linear.z
        self.grasping_point_velocity_manipulator.linear.z = R[2,0]*self.grasping_point_velocity_global.linear.x + R[2,1]*self.grasping_point_velocity_global.linear.y + R[2,2]*self.grasping_point_velocity_global.linear.z
        self.grasping_point_velocity_manipulator.angular.x = self.grasping_point_velocity_global.angular.x
        self.grasping_point_velocity_manipulator.angular.y = self.grasping_point_velocity_global.angular.y
        self.grasping_point_velocity_manipulator.angular.z = self.grasping_point_velocity_global.angular.z

        print("Grasping Point Velocity: ", self.grasping_point_velocity_manipulator.linear.x, self.grasping_point_velocity_manipulator.linear.y, self.grasping_point_velocity_manipulator.linear.z)
        

    def transform_grasping_point_velocity_global(self):
        # transform grasping point velocity to global frame
        R = transformations.quaternion_matrix([self.object_pose.pose.orientation.x,self.object_pose.pose.orientation.y,self.object_pose.pose.orientation.z,self.object_pose.pose.orientation.w])
        R = transpose(R)
        self.grasping_point_velocity_global.linear.x = self.object_vel.linear.x + R[0,0]*self.grasping_point_velocity_local.linear.x + R[0,1]*self.grasping_point_velocity_local.linear.y + R[0,2]*self.grasping_point_velocity_local.linear.z
        self.grasping_point_velocity_global.linear.y = self.object_vel.linear.y + R[1,0]*self.grasping_point_velocity_local.linear.x + R[1,1]*self.grasping_point_velocity_local.linear.y + R[1,2]*self.grasping_point_velocity_local.linear.z
        self.grasping_point_velocity_global.linear.z = self.object_vel.linear.z + R[2,0]*self.grasping_point_velocity_local.linear.x + R[2,1]*self.grasping_point_velocity_local.linear.y + R[2,2]*self.grasping_point_velocity_local.linear.z
        self.grasping_point_velocity_global.angular.x = self.grasping_point_velocity_local.angular.x
        self.grasping_point_velocity_global.angular.y = self.grasping_point_velocity_local.angular.y
        self.grasping_point_velocity_global.angular.z = self.grasping_point_velocity_local.angular.z

        #print("Grasping Point Velocity: ", self.grasping_point_velocity_global.linear.x, self.grasping_point_velocity_global.linear.y, self.grasping_point_velocity_global.linear.z)

    def compute_grasping_point_velocity_local(self):
        # compute the local grasping point velocity based on the object velocity and the relative pose
        self.grasping_point_velocity_local.linear.x = self.relative_pose[2] * self.object_vel.angular.y - self.relative_pose[1] * self.object_vel.angular.z
        self.grasping_point_velocity_local.linear.y = self.relative_pose[0] * self.object_vel.angular.z - self.relative_pose[2] * self.object_vel.angular.x
        self.grasping_point_velocity_local.linear.z = self.relative_pose[1] * self.object_vel.angular.x - self.relative_pose[0] * self.object_vel.angular.y  
        self.grasping_point_velocity_local.angular.x = self.object_vel.angular.x
        self.grasping_point_velocity_local.angular.y = self.object_vel.angular.y
        self.grasping_point_velocity_local.angular.z = self.object_vel.angular.z

        # print("Grasping Point Velocity: ", self.grasping_point_velocity_local.linear.x, self.grasping_point_velocity_local.linear.y, self.grasping_point_velocity_local.linear.z)

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

        #phi, theta, psi = transformations.euler_from_quaternion([self.equilibrium_position_offset.orientation.x,self.equilibrium_position_offset.orientation.y,self.equilibrium_position_offset.orientation.z,self.equilibrium_position_offset.orientation.w])
        #print("Equilibrium Position Offset:" , phi, theta, psi)

        # print("Equilibrium Position Offset:", self.equilibrium_position_offset.position.x, self.equilibrium_position_offset.position.y, self.equilibrium_position_offset.position.z)

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


    def compute_manipulator_velocity(self):

        # compute manipulator velocity
        self.manipulator_vel.linear.x = self.grasping_point_velocity_manipulator.linear.x + self.equilibrium_position_offset.position.x * self.position_error_gain[0] + self.mir_induced_tcp_velocity.linear.x
        self.manipulator_vel.linear.y = self.grasping_point_velocity_manipulator.linear.y + self.equilibrium_position_offset.position.y * self.position_error_gain[1] - self.mir_induced_tcp_velocity.linear.y
        self.manipulator_vel.linear.z = self.grasping_point_velocity_manipulator.linear.z + self.equilibrium_position_offset.position.z * self.position_error_gain[2]
        euler = transformations.euler_from_quaternion([self.equilibrium_position_offset.orientation.x,self.equilibrium_position_offset.orientation.y,self.equilibrium_position_offset.orientation.z,self.equilibrium_position_offset.orientation.w])
        self.manipulator_vel.angular.x = self.object_vel.angular.x + euler[0] * self.position_error_gain[3] 
        self.manipulator_vel.angular.y = self.object_vel.angular.y + euler[1] * self.position_error_gain[4]
        self.manipulator_vel.angular.z = self.object_vel.angular.z + euler[2] * self.position_error_gain[5] + self.mir_induced_tcp_velocity.angular.z

        #print("Manipulator Velocity: ", self.manipulator_vel.linear.x, self.manipulator_vel.linear.y, self.manipulator_vel.linear.z, self.manipulator_vel.angular.z)

        # self.manipulator_vel.linear.z *= -1
        self.manipulator_vel.linear.x *= -1
        self.manipulator_vel.linear.y *= -1
        self.manipulator_vel.angular.y *= -1
        self.manipulator_vel.angular.x *= -1

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
        # print("Pose Error:" , phi, theta, psi)

        # transform pose error to local frame using the mir pose
        R = transformations.quaternion_matrix([self.mir_pose.orientation.x,self.mir_pose.orientation.y,self.mir_pose.orientation.z,self.mir_pose.orientation.w])
        R = transpose(R)
        self.pose_error_local.position.x = R[0,0]*self.pose_error_global.position.x + R[0,1]*self.pose_error_global.position.y 
        self.pose_error_local.position.y = R[1,0]*self.pose_error_global.position.x + R[1,1]*self.pose_error_global.position.y 
        self.pose_error_local.position.z = self.pose_error_global.position.z
        # q = transformations.quaternion_multiply([self.pose_error_global.orientation.x,self.pose_error_global.orientation.y,self.pose_error_global.orientation.z,self.pose_error_global.orientation.w],
        #                 transformations.quaternion_inverse([self.mir_pose.orientation.x,self.mir_pose.orientation.y,self.mir_pose.orientation.z,self.mir_pose.orientation.w]))
        # self.pose_error_local.orientation.x = q[0]
        # self.pose_error_local.orientation.y = q[1]
        # self.pose_error_local.orientation.z = q[2]
        # self.pose_error_local.orientation.w = q[3]
        self.pose_error_local.orientation.x = self.pose_error_global.orientation.x
        self.pose_error_local.orientation.y = self.pose_error_global.orientation.y
        self.pose_error_local.orientation.z = self.pose_error_global.orientation.z
        self.pose_error_local.orientation.w = self.pose_error_global.orientation.w

        # phi, theta, psi = transformations.euler_from_quaternion([self.pose_error_local.orientation.x,self.pose_error_local.orientation.y,self.pose_error_local.orientation.z,self.pose_error_local.orientation.w])
        # print("Pose Error:" , phi, theta, psi)
        # print("Pose Error:")
        # print("Position error global: ", self.pose_error_global.position.x, self.pose_error_global.position.y, self.pose_error_global.position.z)
        # print("Position error local: ", self.pose_error_local.position.x, self.pose_error_local.position.y, self.pose_error_local.position.z)



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
        
        #broadcast target pose
        self.br.sendTransform((self.target_pose.pose.position.x,self.target_pose.pose.position.y,self.target_pose.pose.position.z),
                    (self.target_pose.pose.orientation.x,self.target_pose.pose.orientation.y,self.target_pose.pose.orientation.z,self.target_pose.pose.orientation.w),
                    rospy.Time.now(),
                    "target_pose",
                    "map")
        

    def get_manipulator_pose_offset(self):
        try:
            now = rospy.Time.now()
            self.tl.waitForTransform(self.mir_base_frame, self.manipulator_base_frame, now, rospy.Duration(5.0))
            trans, rot = self.tl.lookupTransform(self.mir_base_frame, self.manipulator_base_frame, now)
            self.manipulator_base_pose_offset = Pose()
            self.manipulator_base_pose_offset.position.x = trans[0]
            self.manipulator_base_pose_offset.position.y = trans[1]
            self.manipulator_base_pose_offset.position.z = trans[2]
            q = transformations.quaternion_from_euler(rot[0],rot[1],rot[2])
            self.manipulator_base_pose_offset.orientation.x = q[0]
            self.manipulator_base_pose_offset.orientation.y = q[1]
            self.manipulator_base_pose_offset.orientation.z = q[2]
            self.manipulator_base_pose_offset.orientation.w = q[3]

            print("Manipulator Base Pose Offset: ", self.manipulator_base_pose_offset)

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
        return self.wrench_average

    def check_data_freshness(self):
        if rospy.Time.now() - self.last_command_time > rospy.Duration(0.1):
            self.mir_cmd_vel = Twist()

    def object_pose_cb(self,data = PoseStamped()):
        self.object_pose = data

    def object_vel_cb(self,data = Twist()):
        self.object_vel = data

    def manipulator_pose_cb(self,data = PoseStamped()):
        self.manipulator_pose = data.pose

    def manipulator_vel_cb(self,data = Twist()):    
        self.manipulator_vel = data

    def mir_cmd_vel_cb(self,data = Twist()):
        self.mir_cmd_vel = data
        self.last_command_time = rospy.Time.now()   

    def wrench_cb(self,data = WrenchStamped()):
        wrench = data.wrench
        # get transform from wrench frame to manipulator base frame
        try:
            trans, rot = self.tl.lookupTransform(self.manipulator_base_frame, self.wrench_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Could not get transform from wrench frame to manipulator base frame")
            return

        # convert wrench to local frame
        R = transformations.quaternion_matrix(rot)
        R = transpose(R)
        wrench.force.x = R[0,0]*wrench.force.x + R[0,1]*wrench.force.y + R[0,2]*wrench.force.z
        wrench.force.y = R[1,0]*wrench.force.x + R[1,1]*wrench.force.y + R[1,2]*wrench.force.z
        wrench.force.z = R[2,0]*wrench.force.x + R[2,1]*wrench.force.y + R[2,2]*wrench.force.z
        wrench.torque.x = R[0,0]*wrench.torque.x + R[0,1]*wrench.torque.y + R[0,2]*wrench.torque.z
        wrench.torque.y = R[1,0]*wrench.torque.x + R[1,1]*wrench.torque.y + R[1,2]*wrench.torque.z
        wrench.torque.z = R[2,0]*wrench.torque.x + R[2,1]*wrench.torque.y + R[2,2]*wrench.torque.z

        # filter wrench
        self.filtered_wrench = self.filter_wrench(wrench)

    def mir_pose_cb(self,data = Pose()):
        self.mir_pose = data


if __name__ == "__main__":
    DezentralizedAdmittanceController().run()
    rospy.spin()