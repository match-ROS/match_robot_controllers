#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose



class Formation_controller():

    def __init__(self):
        path_array = rospy.get_param("~path_array", [])
        print(len(path_array))
        rospy.Subscriber('mir_pose_simple', Pose, self.mir_pose_callback)   
        pass


    def run(self):
        pass



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