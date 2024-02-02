#!/usr/bin/env python3

# given a target position and velocity in the world frame and a target admittance, this controller calculates the velocity commands for the manipulators to reach the equlibrium position

import rospy

from geometry_msgs.msg import PoseStamped, Twist
from tf import transformations, TransformBroadcaster



class DezentralizedAdmittanceController():

    def config(self):
        self.rate = rospy.get_param('rate', 100.0)
        self.object_pose_topic = rospy.get_param('object_pose_topic','/virtual_object/object_pose')
        self.object_vel_topic = rospy.get_param('object_vel_topic','/virtual_object/object_vel')
        self.manipulator_pose_topic = rospy.get_param('manipulator_pose_topic','manipulator_pose')
        self.manipulator_vel_topic = rospy.get_param('manipulator_vel_topic','manipulator_vel')
        self.wrench_topic = rospy.get_param('wrench_topic','wrench')
        self.relative_pose = rospy.get_param('relative_pose', [1,0,0,0,0,0])
        pass


    def __init__(self):
        rospy.init_node("dezentralized_admittance_controller")
        rospy.loginfo("dezentralized_admittance_controller running")
        self.config()
    
        # initialize variables
        self.object_pose = PoseStamped()
        self.object_vel = Twist()
        self.manipulator_pose = PoseStamped()
        self.manipulator_vel = Twist()
        self.wrench = Twist()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'

        
        # start subscribers
        rospy.Subscriber(self.object_pose_topic, PoseStamped, self.object_pose_cb)
        rospy.Subscriber(self.object_vel_topic, Twist, self.object_vel_cb)
        rospy.Subscriber(self.manipulator_pose_topic, PoseStamped, self.manipulator_pose_cb)
        rospy.Subscriber(self.manipulator_vel_topic, Twist, self.manipulator_vel_cb)
        rospy.Subscriber(self.wrench_topic, Twist, self.wrench_cb)

        # initialize broadcaster
        self.br = TransformBroadcaster()
        

    def run(self):
    
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


    def update(self):
        # compute target pose based on object pose and relative pose
        R = transformations.quaternion_matrix([self.object_pose.pose.orientation.x,self.object_pose.pose.orientation.y,self.object_pose.pose.orientation.z,self.object_pose.pose.orientation.w])
        p = [self.object_pose.pose.position.x,self.object_pose.pose.position.y,self.object_pose.pose.position.z]
        p = transformations.translation_matrix(p)
        T = transformations.concatenate_matrices(p,R)
        T = transformations.concatenate_matrices(T,transformations.translation_matrix(self.relative_pose))
        self.target_pose.pose.position.x = T[0,3]
        self.target_pose.pose.position.y = T[1,3]
        self.target_pose.pose.position.z = T[2,3]
        q = transformations.quaternion_from_matrix(T)
        self.target_pose.pose.orientation.x = q[0]
        self.target_pose.pose.orientation.y = q[1]
        self.target_pose.pose.orientation.z = q[2]
        self.target_pose.pose.orientation.w = q[3]
        self.target_pose.header.stamp = rospy.Time.now()
        
        # broadcast target pose
        self.br.sendTransform((self.target_pose.pose.position.x,self.target_pose.pose.position.y,self.target_pose.pose.position.z),
                    (self.target_pose.pose.orientation.x,self.target_pose.pose.orientation.y,self.target_pose.pose.orientation.z,self.target_pose.pose.orientation.w),
                    rospy.Time.now(),
                    "target_pose",
                    "map")




    def object_pose_cb(self,data = PoseStamped()):
        self.object_pose = data

    def object_vel_cb(self,data = Twist()):
        self.object_vel = data

    def manipulator_pose_cb(self,data = PoseStamped()):
        self.manipulator_pose = data

    def manipulator_vel_cb(self,data = Twist()):    
        self.manipulator_vel = data
        
    def wrench_cb(self,data = Twist()):
        self.wrench = data

if __name__ == "__main__":
    DezentralizedAdmittanceController().run()
    rospy.spin()