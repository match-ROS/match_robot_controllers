#!/usr/bin/env python3

# given a target position and velocity in the world frame and a target admittance, this controller calculates the velocity commands for the manipulators to reach the equlibrium position

import rospy

from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf import transformations, TransformBroadcaster
from geometry_msgs.msg import WrenchStamped, Wrench
import math
from numpy import transpose



class DezentralizedAdmittanceController():

    def config(self):
        self.rate = rospy.get_param('rate', 100.0)
        self.object_pose_topic = rospy.get_param('object_pose_topic','/virtual_object/object_pose')
        self.object_vel_topic = rospy.get_param('object_vel_topic','/virtual_object/object_vel')
        self.manipulator_pose_topic = rospy.get_param('manipulator_pose_topic','/mur620a/UR10_l/global_tcp_pose')
        self.manipulator_vel_topic = rospy.get_param('manipulator_vel_topic','manipulator_vel')
        self.manipulator_command_topic = rospy.get_param('manipulator_command_topic','/mur620a/UR10_l/twist_controller/command_safe')
        self.wrench_topic = rospy.get_param('wrench_topic','/mur620a/UR10_l/wrench')
        self.mir_pose_topic = rospy.get_param('mir_pose_topic','/mur620a/mir_pose_simple')
        self.relative_pose = rospy.get_param('relative_pose', [0,0,0,0,0,3.1415])
        self.admittance = rospy.get_param('admittance', [0,1,1,1,1,1])
        self.wrench_filter_alpha = rospy.get_param('wrench_filter_alpha', 0.01)
        self.wrench_error_gain = rospy.get_param('wrench_error_gain', [0.02,0.01,0.01,0.01,0.01,0.01])
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

        
        # start subscribers
        rospy.Subscriber(self.object_pose_topic, PoseStamped, self.object_pose_cb)
        rospy.Subscriber(self.object_vel_topic, Twist, self.object_vel_cb)
        rospy.Subscriber(self.manipulator_pose_topic, PoseStamped, self.manipulator_pose_cb)
        rospy.Subscriber(self.manipulator_vel_topic, Twist, self.manipulator_vel_cb)
        rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_cb)
        rospy.Subscriber(self.mir_pose_topic, Pose, self.mir_pose_cb)


        # initialize broadcaster
        self.br = TransformBroadcaster()

        # initialize publisher  
        self.manipulator_command_pub = rospy.Publisher(self.manipulator_command_topic, Twist, queue_size=10)
        

    def run(self):
    
        # wait until first messages are received
        rospy.loginfo("Waiting for first messages")
        rospy.wait_for_message(self.object_pose_topic, PoseStamped)
        rospy.wait_for_message(self.object_vel_topic, Twist)
        rospy.wait_for_message(self.manipulator_pose_topic, PoseStamped)
        #rospy.wait_for_message(self.manipulator_vel_topic, Twist)
        rospy.wait_for_message(self.mir_pose_topic, PoseStamped)
        rospy.wait_for_message(self.wrench_topic, WrenchStamped)
        rospy.loginfo("First messages received")

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


    def update(self):
        # compute target pose based on object pose and relative pose
        self.compute_target_pose()
        # compute pose error
        self.compute_pose_error_global()
        # compute retentive force
        self.compute_retentive_force()
        # compute wrench error
        self.compute_wrench_error()
        # compute manipulator velocity
        self.compute_manipulator_velocity()


    def compute_manipulator_velocity(self):
        # compute manipulator velocity
        self.manipulator_vel.linear.x = self.object_vel.linear.x + self.relative_pose[1] * self.object_vel.angular.z - self.relative_pose[2] * self.object_vel.angular.y + self.wrench_error.force.x * self.wrench_error_gain[0]
        #self.manipulator_vel.linear.y = self.object_vel.linear.y + self.relative_pose[2] * self.object_vel.angular.x - self.relative_pose[0] * self.object_vel.angular.z + self.wrench_error.force.y * self.wrench_error_gain[1]
        #self.manipulator_vel.linear.z = self.object_vel.linear.z + self.relative_pose[0] * self.object_vel.angular.y - self.relative_pose[1] * self.object_vel.angular.x + self.wrench_error.force.z * self.wrench_error_gain[2]
        self.manipulator_vel.angular.x = self.object_vel.angular.x + self.wrench_error.torque.x * self.wrench_error_gain[3] 
        self.manipulator_vel.angular.y = self.object_vel.angular.y + self.wrench_error.torque.y * self.wrench_error_gain[4]
        self.manipulator_vel.angular.z = self.object_vel.angular.z + self.wrench_error.torque.z * self.wrench_error_gain[5]

        # self.manipulator_vel.linear.z *= -1
        #self.manipulator_vel.angular.x *= -1

        # print("Manipulator Velocity:")
        print("Linear Velocity: ", self.manipulator_vel.linear.x, self.manipulator_vel.linear.y, self.manipulator_vel.linear.z)

        # publish manipulator velocity
        #self.manipulator_command_pub.publish(self.manipulator_vel)

    def compute_wrench_error(self):
        # compute wrench error
        self.wrench_error = Wrench()
        self.wrench_error.force.x = self.retention_force.force.x - self.filtered_wrench.force.x
        self.wrench_error.force.y = self.retention_force.force.y - self.filtered_wrench.force.y
        self.wrench_error.force.z = self.retention_force.force.z - self.filtered_wrench.force.z
        self.wrench_error.torque.x = self.retention_force.torque.x - self.filtered_wrench.torque.x
        self.wrench_error.torque.y = self.retention_force.torque.y - self.filtered_wrench.torque.y
        self.wrench_error.torque.z = self.retention_force.torque.z - self.filtered_wrench.torque.z

        # print("Wrench Error:")
        print("Force error: ", self.wrench_error.force.x, self.wrench_error.force.y, self.wrench_error.force.z)

    def compute_retentive_force(self):
        # compute retentive force based on pose error and admittance
        self.retention_force = Wrench()
        self.retention_force.force.x = self.pose_error_local.position.x * self.admittance[0]
        self.retention_force.force.y = self.pose_error_local.position.y * self.admittance[1]
        self.retention_force.force.z = self.pose_error_local.position.z * self.admittance[2]
        eul = transformations.euler_from_quaternion([self.pose_error_local.orientation.x, self.pose_error_local.orientation.y, self.pose_error_local.orientation.z, self.pose_error_local.orientation.w])
        self.retention_force.torque.x = eul[0] * self.admittance[3]
        self.retention_force.torque.y = eul[1] * self.admittance[4]
        self.retention_force.torque.z = eul[2] * self.admittance[5]



    def compute_pose_error_global(self):
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

        # transform pose error to local frame using the mir pose
        R = transformations.quaternion_matrix([self.mir_pose.orientation.x,self.mir_pose.orientation.y,self.mir_pose.orientation.z,self.mir_pose.orientation.w])
        R = transpose(R)
        self.pose_error_local.position.x = R[0,0]*self.pose_error_global.position.x + R[0,1]*self.pose_error_global.position.y 
        self.pose_error_local.position.y = R[1,0]*self.pose_error_global.position.x + R[1,1]*self.pose_error_global.position.y 
        self.pose_error_local.position.z = self.pose_error_global.position.z
        q = transformations.quaternion_multiply([self.pose_error_global.orientation.x,self.pose_error_global.orientation.y,self.pose_error_global.orientation.z,self.pose_error_global.orientation.w],
                        transformations.quaternion_inverse([self.mir_pose.orientation.x,self.mir_pose.orientation.y,self.mir_pose.orientation.z,self.mir_pose.orientation.w]))
        self.pose_error_local.orientation.x = q[0]
        self.pose_error_local.orientation.y = q[1]
        self.pose_error_local.orientation.z = q[2]
        self.pose_error_local.orientation.w = q[3]


        # print("Pose Error:")
        print("Position error global: ", self.pose_error_global.position.x, self.pose_error_global.position.y, self.pose_error_global.position.z)
        print("Position error local: ", self.pose_error_local.position.x, self.pose_error_local.position.y, self.pose_error_local.position.z)



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
        
    def filter_wrench(self,wrench = Wrench()):
        self.wrench_average.force.x = (1-self.wrench_filter_alpha)*self.wrench_average.force.x + self.wrench_filter_alpha*wrench.force.x
        self.wrench_average.force.y = (1-self.wrench_filter_alpha)*self.wrench_average.force.y + self.wrench_filter_alpha*wrench.force.y
        self.wrench_average.force.z = (1-self.wrench_filter_alpha)*self.wrench_average.force.z + self.wrench_filter_alpha*wrench.force.z
        self.wrench_average.torque.x = (1-self.wrench_filter_alpha)*self.wrench_average.torque.x + self.wrench_filter_alpha*wrench.torque.x
        self.wrench_average.torque.y = (1-self.wrench_filter_alpha)*self.wrench_average.torque.y + self.wrench_filter_alpha*wrench.torque.y
        self.wrench_average.torque.z = (1-self.wrench_filter_alpha)*self.wrench_average.torque.z + self.wrench_filter_alpha*wrench.torque.z
        return self.wrench_average


    def object_pose_cb(self,data = PoseStamped()):
        self.object_pose = data

    def object_vel_cb(self,data = Twist()):
        self.object_vel = data

    def manipulator_pose_cb(self,data = PoseStamped()):
        self.manipulator_pose = data.pose

    def manipulator_vel_cb(self,data = Twist()):    
        self.manipulator_vel = data
        
    def wrench_cb(self,data = WrenchStamped()):
        self.wrench = data.wrench
        # filter wrench
        self.filtered_wrench = self.filter_wrench(self.wrench)

    def mir_pose_cb(self,data = Pose()):
        self.mir_pose = data

if __name__ == "__main__":
    DezentralizedAdmittanceController().run()
    rospy.spin()